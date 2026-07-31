"""UC2 pick-and-place example for roboplan_aligator (design §6): windowed constraints on an
RRT-seeded trajectory, demonstrating composition with an existing planner.

A ``roboplan_rrt`` path (collision-free, in the arm group's configuration space) seeds a
``TrajectoryOptimizer`` that then refines it into a dynamically-feasible pick-and-place motion with
*windowed* terms (all through the installed ``roboplan.aligator`` bindings):

- a HARD ``FramePoseConstraint`` grasp via-point at a mid-horizon stage (the tip must pass through the
  grasp pose within tolerance),
- a ``FrameAxisCost`` over the approach window ``(a, b)`` just before the grasp (the gripper points
  down as it approaches),
- a terminal ``FramePoseCost`` at the place pose,
- horizon-wide ``VelocityLimit`` and ``TorqueLimit`` (collision is opt-in via ``--with-collision``;
  the RRT seed is already collision-free).

Run:
    pixi run python roboplan_examples/python/example_aligator_pick.py
    pixi run python roboplan_examples/python/example_aligator_pick.py --headless
"""

import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
import tyro
import xacro

import roboplan.aligator as al
from roboplan.core import JointConfiguration, Scene
from roboplan.example_models import get_package_share_dir
from roboplan.rrt import RRT, RRTOptions

from common import get_model_data

GROUP = "arm"  # UR5 arm: 6 revolute joints, nq == nv == 6, group order == reduced-model order.
TIP = "tool0"


def _tip_pose(scene: Scene, group_config: np.ndarray) -> np.ndarray:
    """World pose (4x4) of the tool frame at an arm-group configuration."""
    return scene.forwardKinematics(scene.toFullJointPositions(GROUP, group_config), TIP)


def _sample_arm_config(scene: Scene, q_indices: np.ndarray) -> np.ndarray:
    """A collision-free arm-group configuration, or a clear error if sampling fails.

    ``randomCollisionFreePositions()`` returns ``None`` when it cannot find a collision-free
    configuration within its internal budget; raise (rather than let ``None[q_indices]`` throw an
    opaque ``TypeError``) so the guarded ``main`` reports it and suggests a different seed.
    """
    q_full = scene.randomCollisionFreePositions()
    if q_full is None:
        raise RuntimeError(
            "randomCollisionFreePositions() found no collision-free configuration; "
            "try a different --seed"
        )
    return np.asarray(q_full)[q_indices]


def main(
    headless: bool = False,
    horizon: int = 100,
    dt: float = 0.07,
    max_iters: int = 400,
    with_collision: bool = False,
    seed: int = 42,
    host: str = "localhost",
    port: str = "8000",
) -> None:
    """Solve UC2 (windowed pick-and-place) on UR5 through the roboplan.aligator bindings.

    Args:
        headless: Skip viser/interactive plots (save a dashboard, run sanity checks) so the example
            runs end-to-end without a display.
        horizon: Number of stages N.
        dt: Time step, in seconds.
        max_iters: ProxDDP iteration budget.
        with_collision: Add a hard SelfCollisionConstraint (slower — mesh distance per stage).
        seed: RNG seed for the (reproducible) start/grasp/place configurations.
        host: Host for the ViserVisualizer.
        port: Port for the ViserVisualizer.
    """
    print(f"roboplan.aligator version: {al.__version__}")

    model_data = get_model_data()["ur5"]
    package_paths = [get_package_share_dir()]
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()
    scene = Scene(
        "aligator_pick_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )
    for obstacle in model_data.obstacles:
        obstacle.addToScene(scene)

    scene.setRngSeed(seed)
    q_indices = np.array(scene.getJointGroupInfo(GROUP).q_indices)

    # Three reproducible, collision-free arm configurations: where the motion starts, the grasp
    # via-point, and the place target. grasp/place poses are the tool poses at those configurations
    # (reachable by construction, so the hard grasp constraint is feasible).
    q_start = _sample_arm_config(scene, q_indices)
    q_grasp = _sample_arm_config(scene, q_indices)
    q_place = _sample_arm_config(scene, q_indices)
    grasp_pose = _tip_pose(scene, q_grasp)
    place_pose = _tip_pose(scene, q_place)

    # --- Seed: an RRT-Connect path routed THROUGH the grasp (start -> grasp -> place), so the seed
    # already visits the grasp config and the hard grasp via-point starts near-feasible. A seed that
    # skips the grasp forces the solver to bend the whole trajectory onto an unrelated mid-horizon
    # pose, which does not converge under the torque/velocity limits. ---
    rrt = RRT(
        scene, RRTOptions(group_name=GROUP, rrt_connect=True, max_planning_time=5.0)
    )
    rrt.setRngSeed(seed)

    def _jc(q: np.ndarray) -> JointConfiguration:
        jc = JointConfiguration()
        jc.positions = q
        return jc

    t0 = time.perf_counter()
    path_to_grasp = rrt.plan(_jc(q_start), _jc(q_grasp))
    path_to_place = rrt.plan(_jc(q_grasp), _jc(q_place))
    plan_ms = (time.perf_counter() - t0) * 1e3
    # Join the two legs, dropping the duplicated grasp waypoint at the seam.
    waypoints = list(path_to_grasp.positions) + list(path_to_place.positions)[1:]
    print(
        f"\nRRT-Connect routed start->grasp->place: {len(path_to_grasp.positions)} + "
        f"{len(path_to_place.positions)} waypoints ({len(waypoints)} after joining) in "
        f"{plan_ms:.0f} ms"
    )

    # interpolatePath spreads the horizon uniformly across waypoint segments (verified:
    # roboplan_aligator/src/trajectory_optimizer.cpp:455-457), so the grasp waypoint at joined index
    # grasp_idx lands at this stage. Anchor the hard grasp via-point exactly there.
    grasp_idx = len(path_to_grasp.positions) - 1
    num_segments = max(1, len(waypoints) - 1)
    grasp_stage = max(1, min(horizon - 1, round(horizon * grasp_idx / num_segments)))
    approach_start = max(0, grasp_stage - 10)

    # --- Optimizer: windowed pick-and-place over the RRT seed ---
    opt = al.TrajectoryOptimizer(
        scene, GROUP, horizon, dt, al.TrajOptOptions(max_iters=max_iters)
    )
    nv = opt.nv()
    opt.setInitialState(q_start)

    # HARD grasp via-point: the tip must reach the grasp pose at the grasp stage (within tolerance).
    grasp = al.FramePoseConstraint()
    grasp.frame = TIP
    grasp.target = grasp_pose
    grasp.tol_pos = 0.02
    grasp.tol_rot = 0.30
    opt.addConstraint(
        grasp, timesteps=(grasp_stage, grasp_stage + 1)
    )  # a single-stage window

    # Approach alignment: over the window before the grasp, point the tool's local z-axis down.
    approach = al.FrameAxisCost()
    approach.frame = TIP
    approach.axis_local = np.array([0.0, 0.0, 1.0])
    approach.axis_world_target = np.array([0.0, 0.0, -1.0])
    approach.weight = 5.0
    opt.addCost(approach, timesteps=(approach_start, grasp_stage))

    # Terminal place pose (soft) + arrive at rest + running velocity damping for dynamic feasibility.
    place = al.FramePoseCost()
    place.frame = TIP
    place.target = place_pose
    place.position_cost = np.full(3, 20000.0)
    place.orientation_cost = np.full(3, 100.0)
    opt.addCost(place, timesteps=opt.horizon())
    settle = al.VelocityCost()
    settle.weights = np.full(nv, 20.0)
    opt.addCost(settle, timesteps=opt.horizon())
    damping = al.VelocityCost()
    damping.weights = np.full(nv, 1.0)
    opt.addCost(damping)

    # Horizon-wide limits.
    opt.addConstraint(al.VelocityLimit())
    opt.addConstraint(al.TorqueLimit())
    if with_collision:
        sc = al.SelfCollisionConstraint()
        sc.n_pairs = 4
        opt.addConstraint(sc)
        print(
            "  (self-collision constraint enabled — the solve will take noticeably longer)"
        )

    opt.build()
    seed_traj = opt.interpolatePath(
        waypoints
    )  # joined RRT group waypoints -> N-grid seed
    t0 = time.perf_counter()
    result = opt.solve(seed_traj)
    solve_ms = (time.perf_counter() - t0) * 1e3

    # --- Introspection ---
    grasp_reached = _tip_pose(scene, np.asarray(result.xs[grasp_stage])[:nv])[:3, 3]
    place_reached = _tip_pose(scene, np.asarray(result.xs[-1])[:nv])[:3, 3]
    grasp_err = float(np.linalg.norm(grasp_reached - grasp_pose[:3, 3]))
    place_err = float(np.linalg.norm(place_reached - place_pose[:3, 3]))

    print("\n=== UC2 pick-and-place solve summary ===")
    print(f"  converged                : {result.converged}")
    print(f"  iterations               : {result.iterations}")
    print(f"  cost                     : {result.cost:.4f}")
    print(f"  max_constraint_violation : {result.max_constraint_violation:.3e}")
    print(f"  solve time               : {solve_ms:.1f} ms")
    # grasp_err is the 3-axis Euclidean norm; the hard FramePoseConstraint is a per-axis box, so a
    # satisfied constraint permits up to sqrt(3) * tol_pos here (~35 mm at tol_pos = 20 mm/axis).
    print(
        f"  grasp via-point error    : {grasp_err * 1e3:.1f} mm "
        f"(||pos||; hard box is {grasp.tol_pos * 1e3:.0f} mm/axis)"
    )
    print(f"  terminal place error     : {place_err * 1e3:.1f} mm (soft terminal cost)")
    print(f"  peak |torque|            : {np.max(np.abs(np.array(result.us))):.3f} Nm")

    joint_traj = result.toRoboplan(scene, GROUP)
    print(
        f"  toRoboplan -> JointTrajectory with {len(joint_traj.positions)} full-model waypoints"
    )

    if headless:
        # Light sanity checks so a headless run is a real integration check (design §7).
        assert np.isfinite(result.cost), "solve returned a non-finite cost"
        assert (
            result.max_constraint_violation < 1e-2
        ), "constraints not satisfied within tolerance"
        assert grasp_err < 0.05, f"grasp via-point not met: {grasp_err * 1e3:.1f} mm"
        # New invariant: place.position_cost was raised from 500 -> 20000 (Issue 2b) specifically to
        # tighten the terminal place error, which regularly measured ~8.4 mm at this weight (down from
        # ~126.4 mm at 500); 0.03 (30 mm) gives comfortable margin above run-to-run solver noise while
        # still catching a regression back toward the old under-tuned weight.
        assert (
            place_err < 0.03
        ), f"terminal place error too large: {place_err * 1e3:.1f} mm"
        _plot(
            result,
            scene,
            q_indices,
            dt,
            grasp_stage,
            grasp_pose,
            place_pose,
            headless=True,
        )
        print("\n  Sanity checks passed.")
        return

    _plot(
        result,
        scene,
        q_indices,
        dt,
        grasp_stage,
        grasp_pose,
        place_pose,
        headless=False,
    )
    _animate(
        scene,
        model_data,
        urdf_xml,
        package_paths,
        result,
        nv,
        grasp_pose,
        place_pose,
        dt,
        host,
        port,
        grasp.tol_pos,
    )


def _plot(
    result, scene, q_indices, dt, grasp_stage, grasp_pose, place_pose, headless
) -> None:
    """Torque profile + tip distance to the grasp via-point and the place target over time."""
    times_u = np.arange(len(result.us)) * dt
    controls = np.array(result.us)
    times_x = np.array(result.trajectory.times)
    tips = np.array(
        [_tip_pose(scene, np.asarray(x)[: len(q_indices)])[:3, 3] for x in result.xs]
    )
    d_grasp = np.linalg.norm(tips - grasp_pose[:3, 3], axis=1)
    d_place = np.linalg.norm(tips - place_pose[:3, 3], axis=1)

    fig, axes = plt.subplots(2, 1, figsize=(9, 7))
    for i in range(controls.shape[1]):
        axes[0].plot(times_u, controls[:, i], label=f"u[{i}]")
    axes[0].set_title("N x nv torque profile")
    axes[0].set_ylabel("torque [Nm]")
    axes[0].legend(loc="best", fontsize="x-small")
    axes[0].grid(True)

    axes[1].plot(times_x, d_grasp, label="tip -> grasp via-point")
    axes[1].plot(times_x, d_place, label="tip -> place target")
    axes[1].axvline(grasp_stage * dt, color="k", ls="--", lw=0.8, label="grasp stage")
    axes[1].set_title(
        "Tip distance to the windowed grasp via-point and the terminal place target"
    )
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("distance [m]")
    axes[1].legend(loc="best", fontsize="x-small")
    axes[1].grid(True)

    fig.tight_layout()
    if headless:
        out = "aligator_pick_dashboard.png"
        fig.savefig(out, dpi=110)
        print(f"  Saved dashboard to {out}")
        plt.close(fig)
    else:
        print("  Close the matplotlib window to continue to the viser animation.")
        plt.show()


def _animate(
    scene,
    model_data,
    urdf_xml,
    package_paths,
    result,
    nv,
    grasp_pose,
    place_pose,
    dt,
    host,
    port,
    grasp_tol_pos,
) -> None:
    from pinocchio.visualize import ViserVisualizer

    full = pin.buildModelFromXML(urdf_xml)
    collision_model = pin.buildGeomFromUrdfString(
        full, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        full, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )
    viz = ViserVisualizer(full, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)
    # The hard grasp constraint is a per-axis box of half-width tol_pos, so a satisfying solve can
    # miss the grasp point by up to its circumscribing sphere radius sqrt(3) * tol_pos (~35 mm at
    # tol_pos = 20 mm/axis) — size the marker to that so a satisfying solve always visibly touches it.
    grasp_marker_radius = np.sqrt(3.0) * grasp_tol_pos
    viz.viewer.scene.add_icosphere(
        "/pick/grasp",
        radius=grasp_marker_radius,
        color=(0, 200, 0),
        position=tuple(grasp_pose[:3, 3]),
    )
    viz.viewer.scene.add_icosphere(
        "/pick/place", radius=0.03, color=(200, 0, 0), position=tuple(place_pose[:3, 3])
    )

    print("  Animating the optimized pick-and-place in viser (Ctrl+C to stop)...")
    try:
        while True:
            for x in result.xs:
                t_start = time.perf_counter()
                viz.display(scene.toFullJointPositions(GROUP, np.asarray(x)[:nv]))
                time.sleep(max(0.0, dt - (time.perf_counter() - t_start)))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n  Stopped.")


if __name__ == "__main__":
    try:
        tyro.cli(main)
    except RuntimeError as exc:  # e.g. the RRT failed to find a path within the budget
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
