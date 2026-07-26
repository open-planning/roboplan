#!/usr/bin/env python3

import queue
import sys
import time
import tyro
import xacro

from dataclasses import replace

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

from common import ObstacleConfig, get_model_data

try:
    import coal
except ModuleNotFoundError:
    import hppfcl as coal

from roboplan.core import CartesianConfiguration, JointConfiguration, Scene
from roboplan.example_models import get_package_share_dir
from roboplan.rrt import (
    ConstraintProjector,
    PoseConstraint,
    RRT,
    RRTOptions,
    visualizeTree,
)
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions
from roboplan.visualization import addPositionPolyline, visualizeJointTrajectory


# The safe zone the gripper must stay inside, as (min, max) world coordinates in meters. The
# values below are tuned per robot so the whole box is comfortably reachable with the gripper
# pointing straight down, which keeps the demo about the constraint rather than about reachability.
SAFE_ZONE = {
    "ur5": (np.array([0.30, -0.45, 0.25]), np.array([0.75, 0.45, 0.70])),
    "franka": (np.array([0.30, -0.45, 0.25]), np.array([0.75, 0.45, 0.70])),
    "kinova": (np.array([0.30, -0.45, 0.25]), np.array([0.75, 0.45, 0.70])),
}


def make_scene(model_data, package_paths):
    """Builds the planning scene and the redundant Pinocchio models used for visualization."""
    # Pre-process with xacro. This is not necessary for raw URDFs.
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    # Specify argument names to distinguish overloaded Scene constructors from python.
    scene = Scene(
        "constrained_rrt_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )

    # Create a redundant Pinocchio model just for visualization with mimic joints.
    # When Pinocchio 4.x releases nanobind bindings, we should be able to directly grab the model
    # from the scene instead.
    model = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )
    return scene, model, collision_model, visual_model


def build_obstacles(model_data, zone_min, zone_max, inset, include_pillar):
    """Builds the obstacle set: a ground plane, and optionally a pillar inside the safe zone."""
    # A ground plane keeps the unconstrained comparison honest: without it the baseline planner
    # happily swings the gripper below the floor, which flatters the constraint. Reuse the model's
    # own ground plane rather than inventing one, so the contacts it legitimately allows (the link
    # the robot is bolted to, and so on) are right for each robot. It also names obstacles from the
    # shared example scene that this example does not add, so drop those: disabling a collision
    # against a body that is not in the scene is an error.
    ground_plane = next(
        obstacle for obstacle in model_data.obstacles if obstacle.name == "ground_plane"
    )
    unused_obstacles = {
        obstacle.name
        for obstacle in model_data.obstacles
        if obstacle.name != "ground_plane"
    }
    obstacles = [
        replace(
            ground_plane,
            disabled_collisions=[
                body
                for body in (ground_plane.disabled_collisions or [])
                if body not in unused_obstacles
            ],
        )
    ]

    if include_pillar:
        # A pillar rising through the safe zone at the start's x, so goals on the far side of it
        # cannot be reached in a straight line. Clearing it over the top means climbing nearly the
        # whole zone, which the ceiling caps, so the interesting route is around it -- at a
        # constant, level gripper pose.
        height = 0.85 * zone_max[2]
        obstacles.append(
            ObstacleConfig(
                name="pillar",
                geom=coal.Cylinder(0.06, height),
                parent_frame="universe",
                tform=pin.SE3(
                    np.eye(3),
                    np.array(
                        [
                            zone_min[0] + inset[0],
                            0.5 * (zone_min[1] + zone_max[1]),
                            0.5 * height,
                        ]
                    ),
                ).homogeneous,
                color=np.array([0.8, 0.2, 0.2, 0.8]),
                disabled_collisions=["ground_plane"],
            )
        )
    return obstacles


def solve_upright_ik(ik_solver, scene, model_data, position, rotation, seed):
    """Solves IK for a world-frame pose, seeded from a given configuration.

    The seed matters more than it looks. Two IK solutions reached from unrelated seeds can sit in
    different connected components of the constrained configuration space -- elbow up versus elbow
    down, say -- and no amount of planning time bridges that, because crossing between them means
    tipping the gripper over. Seeding the goal solve from the start configuration keeps the two in
    the same branch whenever the solver can find one there.

    Args:
        ik_solver: The IK solver to use.
        scene: The planning scene.
        model_data: The robot model configuration.
        position: The desired world-frame position of the end effector.
        rotation: The desired world-frame rotation of the end effector.
        seed: The full joint configuration to seed the solve from.

    Returns:
        The full joint positions of the solution, or None if IK failed.
    """
    goal = CartesianConfiguration()
    goal.base_frame = model_data.base_link
    goal.tip_frame = model_data.ee_names[0]
    goal.tform = (
        np.linalg.inv(scene.forwardKinematics(seed, model_data.base_link))
        @ pin.SE3(rotation, position).homogeneous
    )

    group_name = model_data.default_joint_group
    start = JointConfiguration()
    start.positions = seed[scene.getJointGroupInfo(group_name).q_indices]
    solution = JointConfiguration()
    if not ik_solver.solveIk(goal, start, solution):
        return None
    return scene.toFullJointPositions(group_name, solution.positions)


def sample_goal(
    rng,
    ik_solver,
    projector,
    scene,
    model_data,
    upright_rotation,
    seed,
    zone_min,
    zone_max,
    margin,
    max_tries=50,
):
    """Draws a random goal inside the safe zone with the gripper upright.

    Yaw is sampled too. The constraint leaves it free, so a random yaw exercises the part of the
    projection that is allowed to slide rather than being pinned, and it keeps successive goals
    from all looking alike.

    Returns:
        The full joint positions of a collision-free, constraint-satisfying goal, or None if no
        sample worked out within `max_tries`.
    """
    for _ in range(max_tries):
        position = rng.uniform(zone_min + margin, zone_max - margin)
        rotation = upright_rotation @ pin.rpy.rpyToMatrix(
            0.0, 0.0, rng.uniform(-np.pi, np.pi)
        )

        q_goal = solve_upright_ik(
            ik_solver, scene, model_data, position, rotation, seed
        )
        if q_goal is None:
            continue

        # IK converges to its own tolerance, not exactly onto the constraint, so project before
        # handing the configuration to the planner -- which refuses endpoints that are off the
        # constraint rather than silently moving the configuration the caller asked for.
        if not projector.satisfies(q_goal):
            q_goal = projector.project(q_goal)
            if q_goal is None:
                continue
        if scene.hasCollisions(q_goal):
            continue
        return q_goal
    return None


def measure_path(scene, path, constraint, group_name, ee_name, samples_per_segment=8):
    """Densely resamples a joint path and measures how it behaves against the constraint.

    Resampling matters: the planner only guarantees the constraint at the resolution it checked,
    so sampling more finely than that is what actually tests the claim.

    Returns:
        A dict of arrays: arc length, gripper tilt from nominal (degrees), the constrained roll and
        pitch (degrees), and end effector position along the path.
    """
    dense = []
    for idx in range(len(path.positions) - 1):
        q_a = scene.toFullJointPositions(group_name, path.positions[idx])
        q_b = scene.toFullJointPositions(group_name, path.positions[idx + 1])
        for step in range(samples_per_segment):
            dense.append(scene.interpolate(q_a, q_b, step / samples_per_segment))
    dense.append(scene.toFullJointPositions(group_name, path.positions[-1]))

    nominal_z = constraint.getTransform()[:3, 2]
    arc, tilt, roll, pitch, positions = [0.0], [], [], [], []
    for idx, q in enumerate(dense):
        if idx > 0:
            arc.append(arc[-1] + scene.configurationDistance(dense[idx - 1], q))
        tform = scene.forwardKinematics(q, ee_name)
        positions.append(tform[:3, 3])

        # Tilt is the angle between the gripper's approach axis and its nominal direction, which is
        # what "upright" means physically. Roll and pitch are the coordinates the constraint
        # actually bounds; tilt is their combined consequence.
        tilt.append(
            np.degrees(np.arccos(np.clip(np.dot(tform[:3, 2], nominal_z), -1.0, 1.0)))
        )
        displacement = constraint.computeDisplacement(q)
        roll.append(np.degrees(displacement[3]))
        pitch.append(np.degrees(displacement[4]))

    return {
        "arc": np.array(arc),
        "tilt": np.array(tilt),
        "roll": np.array(roll),
        "pitch": np.array(pitch),
        "positions": np.array(positions),
    }


def satisfied_at_planning_resolution(scene, projector, path, group_name, step_size):
    """Re-checks a path against the constraints at the resolution the planner used."""
    for idx in range(len(path.positions) - 1):
        q_a = scene.toFullJointPositions(group_name, path.positions[idx])
        q_b = scene.toFullJointPositions(group_name, path.positions[idx + 1])
        if not projector.satisfiesAlongPath(q_a, q_b, step_size):
            return False
    return True


def report(label, metrics, checked, zone_min, zone_max, constraint, max_tilt_degrees):
    """Prints how far a path strays from the constraints.

    Two different questions get answered here, and they answer differently on purpose. `checked` is
    the planner's own promise: the constraints hold, within tolerance, at every configuration it
    looked at. The dense measurements resample much more finely than that, so they also pick up the
    small bulges between check points -- the same discretization caveat collision checking carries,
    and worth showing rather than rounding away.

    Every limit printed alongside a measurement includes the constraint's tolerance, since that
    tolerance is precisely how far outside the bounds a configuration is still allowed to sit.
    """
    positions = metrics["positions"]
    zone_breach = max((zone_min - positions).max(), (positions - zone_max).max(), 0.0)

    # A roll and a pitch of theta each compose into a total tilt of acos(cos^2(theta)).
    angle_slack = np.degrees(constraint.getOrientationTolerance())
    angle_limit = max_tilt_degrees + angle_slack
    tilt_limit = np.degrees(np.arccos(np.cos(np.deg2rad(angle_limit)) ** 2))
    position_slack = constraint.getPositionTolerance() * 1000.0

    print(f"\n  {label}")
    print(f"    satisfied at planning resolution : {'YES' if checked else 'NO'}")
    print(
        f"    max |roll|                       : {np.abs(metrics['roll']).max():6.2f} deg "
        f"(limit {angle_limit:.2f})"
    )
    print(
        f"    max |pitch|                      : {np.abs(metrics['pitch']).max():6.2f} deg "
        f"(limit {angle_limit:.2f})"
    )
    print(
        f"    max gripper tilt                 : {metrics['tilt'].max():6.2f} deg "
        f"(limit {tilt_limit:.2f})"
    )
    print(
        f"    max safe zone breach             : {zone_breach * 1000.0:6.2f} mm "
        f"(limit {position_slack:.2f})"
    )
    return tilt_limit


def main(
    model: str = "ur5",
    max_tilt_degrees: float = 5.0,
    constraint_step_size: float = 0.1,
    max_connection_distance: float = 0.5,
    max_nodes: int = 40000,
    max_planning_time: float = 2.0,
    collision_check_step_size: float = 0.05,
    rrt_star: bool = True,
    rewire_distance: float = 1.0,
    fast_return: bool = False,
    compare_unconstrained: bool = True,
    include_pillar: bool = True,
    host: str = "localhost",
    port: str = "8000",
    rng_seed: int = 1234,
):
    """
    Plans RRT paths that keep the gripper upright and inside a safe zone.

    Each press of "Plan path" draws a random goal inside a box-shaped safe zone and plans to it
    without ever tipping the gripper over or leaving the box, as if the robot were carrying a full
    cup of coffee. Both requirements are one `PoseConstraint`: bounds on the end effector's
    position, plus bounds on its roll and pitch relative to a straight-down nominal orientation.
    Yaw is left free, so the wrist may still spin about the vertical, and the sampled goals use
    that freedom.

    The planner enforces this with the constrained extension of CBiRRT2: it grows the tree in short
    hops and projects each one back onto the constraint before accepting it, rather than letting
    the tree wander off and repairing it afterward.

    Note that bounding roll and pitch to +/- theta admits a total tilt of up to
    acos(cos^2(theta)), because the two rotations compose -- so the default 5 degree box allows the
    gripper to lean at most 7.07 degrees off vertical. Both numbers are reported.

    Parameters:
        model: The name of the model to use.
        max_tilt_degrees: The roll and pitch bound on the gripper, in degrees.
        constraint_step_size: Configuration-space step size taken between constraint projections.
            Smaller values track the constraint more faithfully but add more nodes per unit of
            progress.
        max_connection_distance: Maximum configuration distance covered by a single extension.
        max_nodes: The maximum number of nodes to add to the search trees. Constrained planning
            needs far more than unconstrained planning, since each projected hop becomes a node.
        max_planning_time: The maximum time (in seconds) to search for a path. With `fast_return`
            off this is not a deadline but a budget: every plan spends all of it optimizing.
        collision_check_step_size: Configuration-space step size for collision checking, which is
            also the resolution at which constraints are verified along an edge.
        rrt_star: Whether to rewire the trees as they grow. Worth keeping on here: walking in small
            projected hops leaves a visibly zigzagging path, and rewiring collapses it for roughly
            a quarter shorter paths.
        rewire_distance: Configuration-space radius searched for RRT* rewiring candidates. Larger
            values find shorter paths but cost more, and under constraints the long candidates are
            rejected anyway for drifting off the constraint set.
        fast_return: If true, return the first path found. If false (the default here), spend the
            whole `max_planning_time` rewiring and return the cheapest path found.
        compare_unconstrained: Whether to also plan the same problem with no constraints, to show
            what the constraint is buying.
        include_pillar: Whether to place a pillar inside the safe zone, so the gripper has to
            travel around it while staying level.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
        rng_seed: The seed for the planner, the scene, and the random goal sampling.
    """
    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        sys.exit(1)
    if model not in SAFE_ZONE:
        print(
            f"No safe zone is defined for model '{model}'. Available: {sorted(SAFE_ZONE)}"
        )
        sys.exit(1)

    group_name = model_data.default_joint_group
    ee_name = model_data.ee_names[0]
    package_paths = [get_package_share_dir()]
    zone_min, zone_max = SAFE_ZONE[model]
    zone_center = 0.5 * (zone_min + zone_max)
    zone_size = zone_max - zone_min
    inset = 0.12 * zone_size

    scene, pin_model, collision_model, visual_model = make_scene(
        model_data, package_paths
    )
    scene.setRngSeed(rng_seed)
    rng = np.random.default_rng(rng_seed)
    q_indices = scene.getJointGroupInfo(group_name).q_indices

    for obstacle in build_obstacles(
        model_data, zone_min, zone_max, inset, include_pillar
    ):
        obstacle.addToScene(scene)
        obstacle.addToPinocchioModels(pin_model, collision_model, visual_model)

    viz = ViserVisualizer(pin_model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    # Draw the safe zone, translucent so the robot stays visible inside it.
    viz.viewer.scene.add_box(
        "/safe_zone_wireframe",
        dimensions=tuple(zone_size),
        position=zone_center,
        color=(40, 120, 220),
        opacity=0.35,
        side="back",
    )

    # The nominal "upright" orientation: the gripper's approach (z) axis pointing straight down,
    # which is how a top-down grasp holds a cup level. The constraint measures roll, pitch, and yaw
    # relative to this frame, so a zero displacement means perfectly upright.
    upright_rotation = pin.rpy.rpyToMatrix(np.pi, 0.0, 0.0)
    region_tform = np.eye(4)
    region_tform[:3, :3] = upright_rotation

    # Position bounds are expressed in that same region frame, not the world. Its origin is the
    # world origin and its rotation is a half turn about x, which flips the y and z axes, so a
    # world interval [lo, hi] on those axes becomes [-hi, -lo]. Writing the flip out beats rotating
    # the box corners and taking their extent: an axis-aligned box only stays an axis-aligned box
    # under an axis-aligned rotation, and the corner trick would quietly inflate the zone for any
    # other nominal orientation.
    region_min = np.array([zone_min[0], -zone_max[1], -zone_max[2]])
    region_max = np.array([zone_max[0], -zone_min[1], -zone_min[2]])

    tilt_bound = np.deg2rad(max_tilt_degrees)
    constraint = PoseConstraint(
        scene,
        group_name,
        ee_name,
        lower_bounds=np.concatenate([region_min, [-tilt_bound, -tilt_bound, -np.pi]]),
        upper_bounds=np.concatenate([region_max, [tilt_bound, tilt_bound, np.pi]]),
        tform=region_tform,
    )
    projector = ConstraintProjector(scene, group_name, [constraint])

    print(f"\n=== Constrained RRT: {model} ===")
    print(
        f"  safe zone (world) : x {zone_min[0]:.2f}..{zone_max[0]:.2f}, "
        f"y {zone_min[1]:.2f}..{zone_max[1]:.2f}, z {zone_min[2]:.2f}..{zone_max[2]:.2f} m"
    )
    print(
        f"  gripper roll/pitch: +/- {max_tilt_degrees:.1f} deg "
        f"(total tilt up to {np.degrees(np.arccos(np.cos(tilt_bound) ** 2)):.2f} deg), yaw free"
    )
    print(
        f"  tolerances        : {constraint.getPositionTolerance() * 1000.0:.2f} mm, "
        f"{np.degrees(constraint.getOrientationTolerance()):.3f} deg -- how far outside those "
        f"bounds a configuration may still sit"
    )

    # Two IK solvers, differing only in whether they may restart from a random configuration.
    # Reaching the very first upright pose from the robot's home configuration usually needs a few
    # restarts. Goals must not restart: a restart lands in whatever IK branch it happens to find,
    # and two branches of the constrained space are often not connected to each other -- crossing
    # between them would mean tipping the gripper over, which is exactly what is forbidden. Seeding
    # each goal solve from the start configuration and refusing restarts keeps every goal in the
    # branch the start lives in, so a failure means "resample a different goal", not "plan forever".
    ik_solver = SimpleIk(
        scene, SimpleIkOptions(group_name=group_name, max_restarts=20, max_time=0.5)
    )
    goal_ik_solver = SimpleIk(
        scene, SimpleIkOptions(group_name=group_name, max_restarts=0, max_time=0.1)
    )
    goal_margin = 0.1 * zone_size

    # Anchor the start in a corner of the zone, so the pillar sits between it and a good share of
    # the goals sampled later.
    start_position = np.array(
        [zone_min[0] + inset[0], zone_min[1] + inset[1], zone_min[2] + inset[2]]
    )
    q_start = solve_upright_ik(
        ik_solver,
        scene,
        model_data,
        start_position,
        upright_rotation,
        scene.getCurrentJointPositions(),
    )
    if q_start is None:
        print(f"\nCould not solve IK for the start pose at {start_position}.")
        sys.exit(1)
    if not projector.satisfies(q_start):
        q_start = projector.project(q_start)
        if q_start is None:
            print("\nCould not project the start configuration onto the constraint.")
            sys.exit(1)
    print(f"  start EE position : {scene.forwardKinematics(q_start, ee_name)[:3, 3]}")

    start = JointConfiguration()
    start.positions = q_start[q_indices]

    options = RRTOptions(
        group_name=group_name,
        max_nodes=max_nodes,
        max_connection_distance=max_connection_distance,
        collision_check_step_size=collision_check_step_size,
        max_planning_time=max_planning_time,
        rrt_connect=True,
        rrt_star=rrt_star,
        rewire_distance=rewire_distance,
        fast_return=fast_return,
        constraint_step_size=constraint_step_size,
    )
    rrt = RRT(scene, options)
    rrt.setRngSeed(rng_seed)

    toppra = PathParameterizerTOPPRA(scene, group_name)
    traj_dt = 0.01

    viz.viewer.scene.add_icosphere(
        "/constrained_rrt/start",
        radius=0.025,
        color=(0, 200, 0),
        position=scene.forwardKinematics(q_start, ee_name)[:3, 3],
    )
    viz.display(q_start)
    scene.setJointPositions(q_start)

    traj_queue = queue.Queue()
    metrics_queue = queue.Queue()
    cur_traj = None
    animate = False

    plan_button = viz.viewer.gui.add_button("Plan path")
    animate_button = viz.viewer.gui.add_button("Animate trajectory")
    animate_button.disabled = True

    @plan_button.on_click
    def plan_path(_):
        nonlocal animate
        animate = False
        plan_button.disabled = True
        animate_button.disabled = True
        try:
            q_goal = sample_goal(
                rng,
                goal_ik_solver,
                projector,
                scene,
                model_data,
                upright_rotation,
                q_start,
                zone_min,
                zone_max,
                goal_margin,
            )
            if q_goal is None:
                print(
                    "\nCould not sample a reachable goal inside the safe zone. Try again."
                )
                return

            goal = JointConfiguration()
            goal.positions = q_goal[q_indices]
            goal_position = scene.forwardKinematics(q_goal, ee_name)[:3, 3]
            viz.viewer.scene.add_icosphere(
                "/constrained_rrt/goal",
                radius=0.025,
                color=(200, 0, 0),
                position=goal_position,
            )
            print(f"\nSampled goal EE position: {goal_position}")

            print("Planning with constraints...")
            t_start = time.time()
            try:
                path = rrt.plan(start, goal, [constraint])
            except RuntimeError as ex:
                print(f"  Constrained planning failed: {ex}")
                return
            print(
                f"  Found a path in {time.time() - t_start:.3f} s "
                f"({len(path.positions)} waypoints)"
            )

            constrained_metrics = measure_path(
                scene, path, constraint, group_name, ee_name
            )
            tilt_limit = report(
                "constrained",
                constrained_metrics,
                satisfied_at_planning_resolution(
                    scene, projector, path, group_name, collision_check_step_size
                ),
                zone_min,
                zone_max,
                constraint,
                max_tilt_degrees,
            )

            unconstrained_metrics = None
            if compare_unconstrained:
                print("\nPlanning the same problem without constraints...")
                t_start = time.time()
                try:
                    baseline = rrt.plan(start, goal)
                    print(
                        f"  Found a path in {time.time() - t_start:.3f} s "
                        f"({len(baseline.positions)} waypoints)"
                    )
                    unconstrained_metrics = measure_path(
                        scene, baseline, constraint, group_name, ee_name
                    )
                    report(
                        "unconstrained",
                        unconstrained_metrics,
                        satisfied_at_planning_resolution(
                            scene,
                            projector,
                            baseline,
                            group_name,
                            collision_check_step_size,
                        ),
                        zone_min,
                        zone_max,
                        constraint,
                        max_tilt_degrees,
                    )
                except RuntimeError as ex:
                    print(f"  Unconstrained planning failed: {ex}")

            traj = toppra.generate(
                path, TOPPRAOptions(dt=traj_dt, mode=SplineFittingMode.Adaptive)
            )

            viz.display(q_start)
            # visualizeTree(viz, scene, rrt, [ee_name], 0.05)
            visualizeJointTrajectory(
                viz, scene, traj, [ee_name], (0, 180, 0), "/constrained_rrt/path"
            )
            if unconstrained_metrics is not None:
                addPositionPolyline(
                    viz,
                    "/constrained_rrt/unconstrained_path",
                    unconstrained_metrics["positions"],
                    (200, 60, 60),
                    3.0,
                )

            traj_queue.put(traj)
            metrics_queue.put((constrained_metrics, unconstrained_metrics, tilt_limit))
        finally:
            plan_button.disabled = False
            animate_button.disabled = False

    @animate_button.on_click
    def animate_trajectory(_):
        nonlocal animate
        plan_button.disabled = True
        animate_button.disabled = True
        animate = True

    def plot_metrics(constrained_metrics, unconstrained_metrics, tilt_limit):
        """Plots gripper tilt and height against path progress, with the limits drawn in."""
        plt.clf()
        fig = plt.gcf()
        fig.set_size_inches(9, 7)
        axes = fig.subplots(2, 1, sharex=True)

        series = [("constrained", constrained_metrics, "tab:green")]
        if unconstrained_metrics is not None:
            series.append(("unconstrained", unconstrained_metrics, "tab:red"))

        for label, metrics, color in series:
            progress = metrics["arc"] / metrics["arc"][-1]
            axes[0].plot(progress, metrics["tilt"], color=color, label=label)
            axes[1].plot(progress, metrics["positions"][:, 2], color=color, label=label)

        axes[0].axhline(
            tilt_limit,
            color="k",
            linestyle="--",
            linewidth=1,
            label=f"tilt limit ({tilt_limit:.2f} deg)",
        )
        axes[0].set_ylabel("gripper tilt from vertical [deg]")
        axes[0].set_title("Gripper stays upright and inside the safe zone")
        axes[0].legend(loc="upper right", fontsize="small")
        axes[0].grid(alpha=0.3)

        axes[1].axhline(
            zone_min[2], color="k", linestyle="--", linewidth=1, label="safe zone z"
        )
        axes[1].axhline(zone_max[2], color="k", linestyle="--", linewidth=1)
        axes[1].set_ylabel("end effector height [m]")
        axes[1].set_xlabel("path progress")
        axes[1].legend(loc="upper right", fontsize="small")
        axes[1].grid(alpha=0.3)
        return fig

    # Main display and animation loop.
    plt.figure()
    plt.ion()
    while True:
        if not metrics_queue.empty():
            fig = plot_metrics(*metrics_queue.get())
            cur_traj = traj_queue.get()
            plt.draw()
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.1)
        elif animate and cur_traj is not None:
            print("\nAnimating trajectory...")
            for q in cur_traj.positions:
                viz.display(scene.toFullJointPositions(group_name, q))
                time.sleep(traj_dt)
            viz.display(q_start)
            animate = False
            plan_button.disabled = False
            animate_button.disabled = False
            print("...done!")
        else:
            time.sleep(0.1)


if __name__ == "__main__":
    tyro.cli(main)
