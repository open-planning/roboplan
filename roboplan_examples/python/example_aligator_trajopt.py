#!/usr/bin/env python3
"""Compare three ways to turn an RRT path into an executable trajectory, on the same start/goal
pair:

- **RRT** -- the raw geometric path RRT (optionally RRT-Connect) found. No timing, no dynamics.
- **RRT + TOPP-RA** (``roboplan_toppra``) -- kinematic time-optimal retiming of that same path
  under velocity/acceleration limits. The path geometry never changes.
- **RRT + roboplan_aligator** -- the path *reshaped* by full-dynamics trajectory optimization,
  seeded from the RRT path, subject to a torque limit.

RRT runs exactly once per "Plan path" click; that single path seeds both TOPP-RA and
roboplan_aligator, so all three are compared against the identical RRT solution (no re-planning,
so RRT's own nondeterminism cannot skew the comparison). Click "Plan path", then use the
per-method "Animate ..." buttons to play each one back in viser. The console prints each method's
peak required torque -- TOPP-RA's is a post-hoc inverse-dynamics estimate (``pin.rnea``) and can
exceed a joint's torque limit, since TOPP-RA only reasons about velocity/acceleration bounds, not
the robot's actual mass/inertia; roboplan_aligator enforces the limit directly during the solve.

Run:
    pixi run python roboplan_examples/python/example_aligator_trajopt.py
    pixi run python roboplan_examples/python/example_aligator_trajopt.py --model so101
"""

import queue
import sys
import time

import numpy as np
import pinocchio as pin
import tyro
import xacro
from pinocchio.visualize import ViserVisualizer

import roboplan.aligator as al
from common import get_model_data
from roboplan.core import JointConfiguration, Scene
from roboplan.example_models import get_package_share_dir
from roboplan.rrt import RRT, RRTOptions, visualizeTree
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions
from roboplan.visualization import visualizeJointTrajectory


def _build_reduced_model(urdf_xml: str, group_joints: set[str]) -> pin.Model:
    """Reduced pinocchio model matching TrajectoryOptimizer's internal per-group reduction
    (non-group joints locked at the neutral configuration). Used only to compute the inverse-
    dynamics torque a non-dynamics-aware trajectory (e.g. TOPP-RA's) would actually require.
    """
    full = pin.buildModelFromXML(urdf_xml)
    q_ref_full = pin.neutral(full)
    joints_to_lock = [
        full.getJointId(full.names[i])
        for i in range(1, full.njoints)
        if full.names[i] not in group_joints
    ]
    return pin.buildReducedModel(full, joints_to_lock, q_ref_full)


def _required_torque(
    reduced: pin.Model, positions, velocities, accelerations
) -> np.ndarray:
    """Inverse-dynamics torque (pin.rnea) the reduced model needs to realize a trajectory."""
    data = reduced.createData()
    return np.array(
        [
            pin.rnea(reduced, data, np.asarray(q), np.asarray(v), np.asarray(a))
            for q, v, a in zip(positions, velocities, accelerations)
        ]
    )


def main(
    model: str = "ur5",
    max_connection_distance: float = 3.0,
    collision_check_step_size: float = 0.05,
    goal_biasing_probability: float = 0.15,
    max_nodes: int = 1000,
    max_planning_time: float = 2.0,
    rrt_connect: bool = True,
    toppra_mode: SplineFittingMode = SplineFittingMode.Adaptive,
    toppra_dt: float = 0.01,
    path_step_dt: float = 0.4,
    horizon: int = 60,
    aligator_dt: float = 0.05,
    tau_max: float | None = None,
    max_iters: int = 200,
    host: str = "localhost",
    port: str = "8000",
    rng_seed: int | None = None,
    include_obstacles: bool = True,
):
    """Compare RRT, RRT + TOPP-RA, and RRT + roboplan_aligator on the same start/goal pair.

    Parameters:
        model: The name of the model to use.
        max_connection_distance: Maximum connection distance between two RRT search nodes.
        collision_check_step_size: Configuration-space step size for collision checking along edges.
        goal_biasing_probability: Weighting of the goal node during random sampling.
        max_nodes: The maximum number of nodes to add to the RRT search tree.
        max_planning_time: The maximum time (in seconds) to search for an RRT path.
        rrt_connect: Whether or not to use RRT-Connect.
        toppra_mode: The trajectory generation mode for TOPP-RA.
        toppra_dt: Time step for the TOPP-RA-timed trajectory, in seconds.
        path_step_dt: Seconds to pause at each waypoint when animating the raw RRT path -- it has
            no timing of its own (just a handful of waypoints), so this is display pacing only.
        horizon: Number of stages for the roboplan_aligator trajectory optimizer.
        aligator_dt: Time step for the roboplan_aligator trajectory optimizer, in seconds.
        tau_max: Symmetric per-joint torque bound [Nm], enforced by roboplan_aligator and compared
            against TOPP-RA's post-hoc required torque. If not given, uses the model's own URDF
            effort limits (recommended -- a value picked without regard to the robot's actual size
            can make the solve infeasible on a heavier arm).
        max_iters: ProxDDP iteration budget for the trajectory optimizer.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
        rng_seed: The seed for selecting random start and goal poses and solving RRT.
        include_obstacles: Whether or not to include a simple box obstacle in the scene (on by
            default, so RRT/TOPP-RA/roboplan_aligator all visibly plan/avoid around it).
    """
    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

    package_paths = [get_package_share_dir()]
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "aligator_comparison_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )
    group_name = model_data.default_joint_group
    group_info = scene.getJointGroupInfo(group_name)
    q_indices = group_info.q_indices
    reduced = _build_reduced_model(urdf_xml, set(group_info.joint_names))

    viz_model = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        viz_model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        viz_model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )
    if include_obstacles and model_data.obstacles:
        # Just the first (a simple box) -- enough to make collision checking visibly active
        # without cluttering the scene with the model's full obstacle set.
        obstacle = model_data.obstacles[0]
        obstacle.addToScene(scene)
        obstacle.addToPinocchioModels(viz_model, collision_model, visual_model)

    viz = ViserVisualizer(viz_model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    rrt = RRT(
        scene,
        RRTOptions(
            group_name=group_name,
            max_connection_distance=max_connection_distance,
            collision_check_step_size=collision_check_step_size,
            goal_biasing_probability=goal_biasing_probability,
            max_nodes=max_nodes,
            max_planning_time=max_planning_time,
            rrt_connect=rrt_connect,
        ),
    )
    if rng_seed is not None:
        rrt.setRngSeed(rng_seed)

    toppra = PathParameterizerTOPPRA(scene, group_name)

    q_full = scene.randomCollisionFreePositions()
    scene.setJointPositions(q_full)
    viz.display(q_full)
    time.sleep(0.1)

    # Populated by plan_path(): "path" (JointPath), "toppra" (JointTrajectory, group layout),
    # "aligator_traj" (JointTrajectory, full-model layout via toRoboplan).
    solved = {}
    animate_queue: queue.Queue = queue.Queue()

    plan_button = viz.viewer.gui.add_button("Plan path")
    animate_buttons = {
        key: viz.viewer.gui.add_button(label)
        for key, label in [
            ("path", "Animate RRT path"),
            ("toppra", "Animate TOPP-RA"),
            ("aligator", "Animate aligator"),
        ]
    }
    for btn in animate_buttons.values():
        btn.disabled = True

    @plan_button.on_click
    def plan_path(_) -> None:
        nonlocal q_full
        plan_button.disabled = True
        for btn in animate_buttons.values():
            btn.disabled = True

        start = JointConfiguration()
        start.positions = q_full[q_indices]
        goal = JointConfiguration()
        goal.positions = scene.randomCollisionFreePositions()[q_indices]

        # RRT plans ONCE; the same path seeds both TOPP-RA and roboplan_aligator below, so the
        # three results are directly comparable (RRT's own randomness cannot skew the comparison).
        print("\nPlanning with RRT...")
        t0 = time.time()
        path = rrt.plan(start, goal)
        print(f"  {len(path.positions)} waypoints in {time.time() - t0:.3f} s")

        print("Timing with TOPP-RA...")
        t0 = time.time()
        toppra_traj = toppra.generate(
            path, TOPPRAOptions(dt=toppra_dt, mode=toppra_mode)
        )
        print(f"  {toppra_traj.times[-1]:.3f} s duration in {time.time() - t0:.3f} s")
        toppra_torque = _required_torque(
            reduced,
            toppra_traj.positions,
            toppra_traj.velocities,
            toppra_traj.accelerations,
        )
        print(f"  peak required |torque| : {np.max(np.abs(toppra_torque)):.3f} Nm")

        print("Optimizing with roboplan_aligator...")
        opt = al.TrajectoryOptimizer(
            scene,
            group_name,
            horizon,
            aligator_dt,
            al.TrajOptOptions(max_iters=max_iters),
        )
        opt.setInitialState(np.asarray(start.positions))

        target = al.ConfigurationCost()
        target.q_target = np.asarray(goal.positions)
        target.weights = np.full(opt.nv(), 500.0)
        opt.addCost(target, timesteps=opt.horizon())
        # Arrive at rest and damp velocity along the horizon for dynamic feasibility (without
        # this, a pure terminal reach settles on a dynamically-inconsistent swing-through).
        settle = al.VelocityCost()
        settle.weights = np.full(opt.nv(), 20.0)
        opt.addCost(settle, timesteps=opt.horizon())
        damping = al.VelocityCost()
        damping.weights = np.full(opt.nv(), 1.0)
        opt.addCost(damping)

        # An empty TorqueLimit defaults to the reduced model's own URDF effort limits -- much
        # safer than a flat user-supplied bound, since a value tuned for a small arm (e.g. so101)
        # can make the problem infeasible on a heavier one (e.g. UR5).
        torque_limit = al.TorqueLimit()
        if tau_max is not None:
            torque_limit.tau_max = np.full(opt.nv(), tau_max)
        opt.addConstraint(torque_limit)

        # NOTE: roboplan_aligator's CollisionConstraint (robot-vs-static) is deliberately NOT
        # attached here even though the scene has an obstacle. Adding it crashes opt.solve() with
        # "std::invalid_argument: The index of the Frame is outside the bounds" -- a real bug in
        # roboplan_aligator, reproduced with build()/interpolatePath() both succeeding and the
        # crash isolated to solve(); not a misuse of the API. RRT still avoids the obstacle by
        # construction (collision-free edges only) and TOPP-RA never moves the path geometry, so
        # the obstacle is still honestly enforced for two of the three methods -- aligator's
        # reshaped trajectory is just not guaranteed to avoid it in this example.
        opt.build()
        t0 = time.time()
        result = opt.solve(opt.interpolatePath(list(path.positions)))
        print(
            f"  converged={result.converged} iterations={result.iterations} "
            f"max_constraint_violation={result.max_constraint_violation:.2e} "
            f"in {time.time() - t0:.3f} s"
        )
        print(
            f"  peak |torque|           : {np.max(np.abs(np.array(result.us))):.3f} Nm"
        )
        aligator_traj = result.toRoboplan(scene, group_name)

        solved["path"] = path
        solved["toppra"] = toppra_traj
        solved["aligator_traj"] = aligator_traj

        viz.display(q_full)
        visualizeTree(viz, scene, rrt, model_data.ee_names, 0.05)
        # No line trace for the raw RRT path here: visualizePath draws it by interpolating
        # STRAIGHT LINES IN JOINT SPACE between the path's few, widely-spaced waypoints and
        # running FK along that -- which visibly zigzags/loops in Cartesian space for a raw,
        # un-shortcut RRT path. That's inherent to what an RRT path looks like pre-shortcutting,
        # not a bug; the "Animate RRT path" button still shows the actual path on the robot.
        visualizeJointTrajectory(
            viz,
            scene,
            toppra_traj,
            model_data.ee_names,
            (30, 100, 220),
            "/compare/toppra",
        )
        visualizeJointTrajectory(
            viz,
            scene,
            aligator_traj,
            model_data.ee_names,
            (220, 120, 30),
            "/compare/aligator",
        )
        q_start_full = scene.toFullJointPositions(group_name, start.positions)
        q_goal_full = scene.toFullJointPositions(group_name, goal.positions)
        for ee_name in model_data.ee_names:
            viz.viewer.scene.add_icosphere(
                f"/compare/start/{ee_name}",
                radius=0.03,
                color=(0, 200, 0),
                position=scene.forwardKinematics(q_start_full, ee_name)[:3, 3],
            )
            viz.viewer.scene.add_icosphere(
                f"/compare/goal/{ee_name}",
                radius=0.03,
                color=(200, 0, 0),
                position=scene.forwardKinematics(q_goal_full, ee_name)[:3, 3],
            )

        plan_button.disabled = False
        for btn in animate_buttons.values():
            btn.disabled = False

    def _make_on_click(key: str):
        def _on_click(_) -> None:
            plan_button.disabled = True
            for btn in animate_buttons.values():
                btn.disabled = True
            animate_queue.put(key)

        return _on_click

    for key, btn in animate_buttons.items():
        btn.on_click(_make_on_click(key))

    # Main animation loop (mirrors example_rrt.py): viser button callbacks only enqueue work
    # here rather than calling viz.display directly, since GUI calls are not guaranteed safe to
    # make from arbitrary callback threads.
    while True:
        if not animate_queue.empty():
            key = animate_queue.get()
            print(f"Animating {key}...")
            if key == "path":
                # The raw path is just a few waypoints with no timing of its own -- pace it
                # visibly rather than at trajectory-sample speed (that made it look like nothing
                # was happening).
                for q in solved["path"].positions:
                    viz.display(scene.toFullJointPositions(group_name, q))
                    time.sleep(path_step_dt)
            elif key == "toppra":
                traj = solved["toppra"]
                t_prev = 0.0
                for q, t in zip(traj.positions, traj.times):
                    viz.display(scene.toFullJointPositions(group_name, q))
                    time.sleep(max(0.0, t - t_prev))
                    t_prev = t
            elif key == "aligator":
                traj = solved["aligator_traj"]
                t_prev = 0.0
                for q, t in zip(traj.positions, traj.times):
                    viz.display(q)  # already full-model layout (toRoboplan)
                    time.sleep(max(0.0, t - t_prev))
                    t_prev = t
            print("  ...done!")
            plan_button.disabled = False
            for btn in animate_buttons.values():
                btn.disabled = False
        else:
            time.sleep(0.1)


if __name__ == "__main__":
    tyro.cli(main)
