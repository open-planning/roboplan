"""UC1 "dynamically feasible reach" example for roboplan_aligator (design doc §6, §7).

The default mode solves the complete UC1: a terminal ``FramePoseCost`` reach with a ``TorqueLimit``
and a ``SelfCollisionConstraint``, warm-started by a straight-line seed, through the *installed
Python bindings* (``roboplan.aligator.TrajectoryOptimizer``). It prints the solve summary
(converged / iterations / cost / max constraint violation / solve time), plots the headline N x nv
torque profile against the limit band, the joint trajectory q(t), and the minimum self-collision
distance vs. time (>= d_min), animates the optimized trajectory in viser, and round-trips the result
through ``toRoboplan``.

Other modes retained from the incremental prompts:
- ``--dump-window``  print the half-open StageWindow / ``timesteps`` convention and exit.
- ``--rollout``      forward-simulate the reduced-model ABA dynamics under a constant torque and
                     animate the free rollout (a dynamics sanity check).

Run:
    pixi run python roboplan_examples/python/example_aligator_reach.py --model so101
    pixi run python roboplan_examples/python/example_aligator_reach.py --model so101 --headless
    pixi run python roboplan_examples/python/example_aligator_reach.py --model so101 --rollout
"""

import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
import tyro
import xacro
from pinocchio.visualize import ViserVisualizer

import roboplan.aligator as al
from roboplan.core import Scene
from roboplan.example_models import get_package_share_dir

from common import get_model_data


def _dump_timesteps(horizon: int) -> None:
    """Print how the Python ``timesteps`` convenience maps to stage windows (design §3.3)."""
    n = horizon
    print(
        f"\n=== timesteps -> StageWindow for horizon N={n} (half-open [a, b), b excluded) ==="
    )
    print(f"  timesteps=None      -> All      -> stages {list(range(0, n))}")
    print(
        f"  timesteps=(1, 4)    -> Range    -> stages {list(range(1, 4))}   # stage 4 EXCLUDED"
    )
    print(
        f"  timesteps={n:<9} -> Terminal -> the terminal node only (an int selects Terminal)"
    )


def _build_reduced_model(
    urdf_xml: str, group_joints: set[str], package_paths: list[str], srdf_xml: str
) -> tuple[pin.Model, pin.Model, pin.GeometryModel, np.ndarray, list[int]]:
    """Build the full + reduced pinocchio models and the reduced self-collision geometry.

    Mirrors the C++ ``ReducedGroupModel`` (design §3.1): non-group joints are locked into a reduced
    ``pinocchio.Model``; the collision geometry is reduced alongside (SRDF-disabled self pairs
    removed, as the ``Scene`` does). Returns ``(full, reduced, reduced_geom, q_ref_full,
    joints_to_lock)``.
    """
    full = pin.buildModelFromXML(urdf_xml)
    q_ref_full = pin.neutral(full)
    joints_to_lock = [
        full.getJointId(full.names[i])
        for i in range(1, full.njoints)
        if full.names[i] not in group_joints
    ]
    full_geom = pin.buildGeomFromUrdfString(
        full, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    full_geom.addAllCollisionPairs()
    pin.removeCollisionPairsFromXML(full, full_geom, srdf_xml)
    reduced, reduced_geom = pin.buildReducedModel(
        full, full_geom, joints_to_lock, q_ref_full
    )
    return full, reduced, reduced_geom, q_ref_full, joints_to_lock


def _expand_reduced_to_full(
    full: pin.Model, reduced: pin.Model, q_ref_full: np.ndarray, q_reduced: np.ndarray
) -> np.ndarray:
    """Place a reduced-model configuration back into the full model (locked joints stay at ref)."""
    q_full = q_ref_full.copy()
    for j in range(1, reduced.njoints):
        fj = full.getJointId(reduced.names[j])
        q_full[full.idx_qs[fj] : full.idx_qs[fj] + full.nqs[fj]] = q_reduced[
            reduced.idx_qs[j] : reduced.idx_qs[j] + reduced.nqs[j]
        ]
    return q_full


def _min_self_distance(
    reduced: pin.Model, reduced_geom: pin.GeometryModel, q: np.ndarray
) -> float:
    """Minimum signed distance over the reduced model's self-collision pairs at configuration q."""
    data = reduced.createData()
    geom_data = reduced_geom.createData()
    pin.computeDistances(reduced, data, reduced_geom, geom_data, q)
    if not geom_data.distanceResults:
        return float("inf")
    return min(r.min_distance for r in geom_data.distanceResults)


def _rollout_semi_implicit_euler(
    reduced: pin.Model, horizon: int, dt: float, seed_torque: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Forward-simulate the reduced-model ABA dynamics under a constant torque (semi-implicit Euler)."""
    data = reduced.createData()
    q = pin.neutral(reduced)
    v = np.zeros(reduced.nv)
    tau = np.full(reduced.nv, seed_torque)
    times = np.arange(horizon + 1) * dt
    qs = np.zeros((horizon + 1, reduced.nq))
    vs = np.zeros((horizon + 1, reduced.nv))
    qs[0], vs[0] = q, v
    for k in range(horizon):
        a = pin.aba(reduced, data, q, v, tau)
        v = v + dt * a
        q = pin.integrate(reduced, q, dt * v)
        qs[k + 1], vs[k + 1] = q, v
    return times, qs, vs


def _plot_reach(
    result: al.TrajOptResult,
    reduced: pin.Model,
    reduced_geom: pin.GeometryModel,
    joint_names: list[str],
    dt: float,
    tau_max: float,
    d_min: float,
    headless: bool,
) -> None:
    """Plot the headline torque profile, the joint trajectory, and min self-distance vs. time."""
    times_u = np.arange(len(result.us)) * dt
    controls = np.array(result.us)  # (N, nv)
    times_x = np.array(result.trajectory.times)
    positions = np.array(result.trajectory.positions)  # (N + 1, nq)
    distances = [_min_self_distance(reduced, reduced_geom, q) for q in positions]

    fig, axes = plt.subplots(3, 1, figsize=(9, 9))

    for i in range(controls.shape[1]):
        axes[0].plot(
            times_u,
            controls[:, i],
            label=joint_names[i] if i < len(joint_names) else f"u[{i}]",
        )
    axes[0].axhspan(
        -tau_max, tau_max, color="green", alpha=0.08, label=f"|tau| <= {tau_max} Nm"
    )
    axes[0].axhline(tau_max, color="green", ls="--", lw=0.8)
    axes[0].axhline(-tau_max, color="green", ls="--", lw=0.8)
    axes[0].set_title("Headline: N x nv torque profile vs. the TorqueLimit band")
    axes[0].set_ylabel("torque [Nm]")
    axes[0].legend(loc="best", fontsize="x-small")
    axes[0].grid(True)

    for i in range(positions.shape[1]):
        axes[1].plot(
            times_x,
            positions[:, i],
            label=joint_names[i] if i < len(joint_names) else f"q[{i}]",
        )
    axes[1].set_title("Optimized joint trajectory q(t)")
    axes[1].set_ylabel("position [rad]")
    axes[1].legend(loc="best", fontsize="x-small")
    axes[1].grid(True)

    axes[2].plot(
        times_x, distances, color="tab:red", label="min self-collision distance"
    )
    axes[2].axhline(d_min, color="k", ls="--", lw=0.8, label=f"d_min = {d_min} m")
    axes[2].set_title(
        "Minimum self-collision distance vs. time (enforced at knots, design §5)"
    )
    axes[2].set_xlabel("time [s]")
    axes[2].set_ylabel("distance [m]")
    axes[2].legend(loc="best", fontsize="x-small")
    axes[2].grid(True)

    fig.tight_layout()
    if headless:
        out = "aligator_reach_dashboard.png"
        fig.savefig(out, dpi=110)
        print(f"  Saved dashboard to {out}")
        plt.close(fig)
    else:
        print("  Close the matplotlib window to continue to the viser animation.")
        plt.show()


def main(
    model: str = "so101",
    dump_window: bool = False,
    rollout: bool = False,
    headless: bool = False,
    with_collision: bool = False,
    horizon: int = 40,
    dt: float = 0.05,
    tau_max: float = 3.0,
    d_min: float = 0.01,
    seed_torque: float = 0.0,
    host: str = "localhost",
    port: str = "8000",
) -> None:
    """Solve UC1 (a dynamically feasible reach) through the roboplan.aligator Python bindings.

    Args:
        model: Robot key from ``common.get_model_data`` (e.g. ``so101``, ``ur5``).
        dump_window: Print the ``timesteps`` -> StageWindow convention and exit.
        rollout: Instead of the reach, animate a free ABA rollout under ``seed_torque``.
        headless: Skip interactive viz (save plots to file, no viser) so the example runs end-to-end.
        with_collision: Add a hard SelfCollisionConstraint (slower — mesh distance per stage).
        horizon: Number of stages N.
        dt: Time step, in seconds.
        tau_max: Symmetric torque bound [Nm] for the TorqueLimit.
        d_min: Minimum self-collision clearance [m] for the SelfCollisionConstraint.
        seed_torque: Constant torque [Nm] for ``--rollout`` (0 = free fall under gravity).
        host: Host for the ViserVisualizer.
        port: Port for the ViserVisualizer.
    """
    print(f"roboplan.aligator version: {al.__version__}")

    if dump_window:
        _dump_timesteps(horizon=5)
        return

    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

    package_paths = [get_package_share_dir()]
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()
    scene = Scene(
        "aligator_reach_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )

    group_name = model_data.default_joint_group
    group = scene.getJointGroupInfo(group_name)
    group_joints = set(group.joint_names)
    tip_frame = model_data.ee_names[0]

    full, reduced, reduced_geom, q_ref_full, _ = _build_reduced_model(
        urdf_xml, group_joints, package_paths, srdf_xml
    )

    print(f"\n=== Reduced model for group '{group_name}' (model: {model}) ===")
    print(f"  reduced nq={reduced.nq}, nv={reduced.nv}; tip frame '{tip_frame}'")
    locked = [name for name in scene.getJointNames() if name not in group_joints]
    print(f"  locked joints: {locked if locked else '(none — no-op reduction)'}")

    if rollout:
        _run_rollout(
            full,
            reduced,
            q_ref_full,
            group,
            horizon,
            dt,
            seed_torque,
            headless,
            package_paths,
            urdf_xml,
            host,
            port,
        )
        return

    # --- UC1: terminal FramePoseCost reach + TorqueLimit + SelfCollisionConstraint ---
    opt = al.TrajectoryOptimizer(
        scene, group_name, horizon, dt, al.TrajOptOptions(max_iters=200)
    )

    # A reachable target: forward-kinematics of the tip at a target reduced configuration.
    q_start = pin.neutral(reduced)
    q_goal = np.clip(
        np.full(reduced.nq, 0.4), reduced.lowerPositionLimit, reduced.upperPositionLimit
    )
    target_pose = scene.forwardKinematics(
        _expand_reduced_to_full(full, reduced, q_ref_full, q_goal), tip_frame, ""
    )

    opt.setInitialState(q_start)
    pose = al.FramePoseCost()
    pose.frame = tip_frame
    pose.target = target_pose
    pose.position_cost = np.full(3, 500.0)
    pose.orientation_cost = np.full(3, 500.0)
    opt.addCost(pose, timesteps=opt.horizon())  # int -> terminal

    # Arrive at rest and damp velocity along the horizon: without this a pure terminal reach under
    # gravity settles on a dynamically-inconsistent swing-through (high terminal velocity), so the AL
    # cannot close the dynamics gaps -> a large max_constraint_violation.
    settle = al.VelocityCost()
    settle.weights = np.full(opt.nv(), 20.0)
    opt.addCost(settle, timesteps=opt.horizon())
    damping = al.VelocityCost()
    damping.weights = np.full(opt.nv(), 1.0)
    opt.addCost(damping)  # all stages

    torque = al.TorqueLimit()
    torque.tau_max = np.full(opt.nv(), tau_max)
    opt.addConstraint(torque)  # None -> all stages

    # The hard self-collision constraint is opt-in: mesh signed-distance is evaluated per pair per
    # stage per solver iteration, so it is markedly slower (design §5 / Prompt 8). The min-distance
    # plot below is computed from the solved trajectory either way, so clearance is always visible.
    if with_collision:
        self_collision = al.SelfCollisionConstraint()
        self_collision.n_pairs = (
            4  # track only the closest few pairs to keep it tractable
        )
        self_collision.d_min = d_min
        opt.addConstraint(self_collision)
        print(
            "  (self-collision constraint enabled — the solve will take noticeably longer)"
        )

    opt.build()
    seed = opt.interpolatePath([q_start, q_goal])
    t0 = time.perf_counter()
    result = opt.solve(seed)
    solve_time = time.perf_counter() - t0

    reached = scene.forwardKinematics(
        _expand_reduced_to_full(full, reduced, q_ref_full, result.xs[-1][: reduced.nq]),
        tip_frame,
        "",
    )
    pos_err = float(np.linalg.norm(reached[:3, 3] - target_pose[:3, 3]))

    print("\n=== UC1 reach solve summary ===")
    print(f"  converged                : {result.converged}")
    print(f"  iterations               : {result.iterations}")
    print(f"  cost                     : {result.cost:.6f}")
    print(f"  max_constraint_violation : {result.max_constraint_violation:.3e}")
    print(f"  solve time               : {solve_time * 1e3:.1f} ms")
    print(f"  terminal position error  : {pos_err * 1e3:.1f} mm")
    print(
        f"  peak |torque|            : {np.max(np.abs(np.array(result.us))):.3f} Nm (bound {tau_max})"
    )

    # toRoboplan round-trip (full-model JointTrajectory).
    joint_traj = result.toRoboplan(scene, group_name)
    print(
        f"  toRoboplan -> JointTrajectory with {len(joint_traj.positions)} full-model waypoints"
    )

    _plot_reach(
        result,
        reduced,
        reduced_geom,
        list(group.joint_names),
        dt,
        tau_max,
        d_min,
        headless,
    )

    if headless:
        return
    _animate(
        full,
        reduced,
        q_ref_full,
        [x[: reduced.nq] for x in result.xs],
        dt,
        package_paths,
        urdf_xml,
        host,
        port,
        "optimized reach",
    )


def _run_rollout(
    full,
    reduced,
    q_ref_full,
    group,
    horizon,
    dt,
    seed_torque,
    headless,
    package_paths,
    urdf_xml,
    host,
    port,
) -> None:
    times, qs, vs = _rollout_semi_implicit_euler(reduced, horizon, dt, seed_torque)
    print(
        f"\n=== ABA rollout: N={horizon}, dt={dt}s, constant torque={seed_torque} Nm ==="
    )
    print(
        f"  q(0)={np.array2string(qs[0], precision=3)}  q(T)={np.array2string(qs[-1], precision=3)}"
    )
    print(f"  |v(T)|={np.linalg.norm(vs[-1]):.4f} rad/s")
    if not headless:
        plt.figure()
        for i in range(qs.shape[1]):
            names = list(group.joint_names)
            plt.plot(times, qs[:, i], label=names[i] if i < len(names) else f"q[{i}]")
        plt.title(f"ABA rollout under constant torque = {seed_torque} Nm")
        plt.xlabel("time [s]")
        plt.ylabel("joint position [rad]")
        plt.legend(loc="best", fontsize="small")
        plt.grid(True)
        plt.show()
        _animate(
            full,
            reduced,
            q_ref_full,
            list(qs),
            dt,
            package_paths,
            urdf_xml,
            host,
            port,
            "free rollout",
        )


def _animate(
    full,
    reduced,
    q_ref_full,
    qs_reduced,
    dt,
    package_paths,
    urdf_xml,
    host,
    port,
    label,
) -> None:
    """Play a reduced-model trajectory in viser (locked joints stay frozen)."""
    collision_model = pin.buildGeomFromUrdfString(
        full, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        full, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )
    viz = ViserVisualizer(full, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)
    print(f"  Animating {label} in viser (Ctrl+C to stop)...")
    try:
        while True:
            for q in qs_reduced:
                t_start = time.perf_counter()
                viz.display(
                    _expand_reduced_to_full(full, reduced, q_ref_full, np.asarray(q))
                )
                time.sleep(max(0.0, dt - (time.perf_counter() - t_start)))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n  Stopped.")


if __name__ == "__main__":
    tyro.cli(main)
