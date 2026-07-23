"""UC3 receding-horizon MPC example for roboplan_aligator ("chasing the carrot", design §6).

The optimizer runs a fixed-horizon problem and, each tick, updates the initial state to the measured
plant state, retargets a terminal ``FramePoseCost`` to a moving Cartesian goal, and re-solves warm-
started from the shifted previous solution (``solve(shift(prev))``). It applies ``controls[0]`` to the
plant (a semi-implicit-Euler step of the reduced ABA dynamics — the same discretization the optimizer
uses) and advances. Mirrors ``example_oink.py``'s interactive loop.

- Interactive (default): drag the viser transform-control marker; the arm receding-horizon-tracks it.
- ``--headless``: a scripted goal sweeps a small circle; the loop prints per-tick solve time and the
  tip tracking error, then exits — so the example runs end-to-end without a display.

Run:
    pixi run python roboplan_examples/python/example_aligator_mpc.py --model so101
    pixi run python roboplan_examples/python/example_aligator_mpc.py --model so101 --headless
"""

import sys
import time

import numpy as np
import pinocchio as pin
import tyro
import xacro

import roboplan.aligator as al
from roboplan.core import Scene
from roboplan.example_models import get_package_share_dir

from common import get_model_data


def _build_reduced_model(
    urdf_xml: str, group_joints: set[str]
) -> tuple[pin.Model, pin.Model, np.ndarray]:
    """Full + reduced pinocchio models for the planning group (mirrors the C++ ReducedGroupModel)."""
    full = pin.buildModelFromXML(urdf_xml)
    q_ref_full = pin.neutral(full)
    joints_to_lock = [
        full.getJointId(full.names[i])
        for i in range(1, full.njoints)
        if full.names[i] not in group_joints
    ]
    reduced = pin.buildReducedModel(full, joints_to_lock, q_ref_full)
    return full, reduced, q_ref_full


def _expand_reduced_to_full(full, reduced, q_ref_full, q_reduced) -> np.ndarray:
    q_full = q_ref_full.copy()
    for j in range(1, reduced.njoints):
        fj = full.getJointId(reduced.names[j])
        q_full[full.idx_qs[fj] : full.idx_qs[fj] + full.nqs[fj]] = q_reduced[
            reduced.idx_qs[j] : reduced.idx_qs[j] + reduced.nqs[j]
        ]
    return q_full


def _tip_position(reduced: pin.Model, data, tip_id: int, q: np.ndarray) -> np.ndarray:
    """World position of the tip frame at reduced configuration q."""
    pin.forwardKinematics(reduced, data, q)
    pin.updateFramePlacement(reduced, data, tip_id)
    return data.oMf[tip_id].translation.copy()


def _plant_step(
    reduced: pin.Model, data, q: np.ndarray, v: np.ndarray, u: np.ndarray, dt: float
):
    """One semi-implicit-Euler step of the reduced ABA dynamics under torque u (the plant)."""
    a = pin.aba(reduced, data, q, v, u)
    v_next = v + dt * a
    q_next = pin.integrate(reduced, q, dt * v_next)
    return q_next, v_next


def _make_optimizer(
    scene: Scene, group_name: str, tip_frame: str, horizon: int, dt: float, nv: int
):
    """Build the fixed-horizon MPC problem: a terminal FramePoseCost (retargeted per tick) tracking
    the goal, terminal velocity regulation (arrive at rest), and running velocity damping for a
    well-posed, dynamically-feasible track."""
    opt = al.TrajectoryOptimizer(
        scene, group_name, horizon, dt, al.TrajOptOptions(max_iters=15)
    )
    pose = al.FramePoseCost()
    pose.frame = tip_frame
    pose.target = np.eye(4)
    pose.position_cost = np.full(3, 300.0)
    pose.orientation_cost = np.zeros(3)  # track position only ("chasing the carrot")
    handle = opt.addCost(pose, timesteps=opt.horizon())  # int -> terminal
    settle = al.VelocityCost()
    settle.weights = np.full(nv, 5.0)
    opt.addCost(settle, timesteps=opt.horizon())
    damping = al.VelocityCost()
    damping.weights = np.full(nv, 0.5)
    opt.addCost(damping)
    opt.build()
    return opt, handle


def main(
    model: str = "so101",
    headless: bool = False,
    horizon: int = 20,
    dt: float = 0.05,
    ticks: int = 40,
    host: str = "localhost",
    port: str = "8000",
) -> None:
    """Receding-horizon-track a Cartesian goal through the roboplan.aligator bindings.

    Args:
        model: Robot key from ``common.get_model_data`` (e.g. ``so101``).
        headless: Run a scripted goal sweep (no viser), printing per-tick solve time + tracking error.
        horizon: MPC horizon N.
        dt: Time step / tick period, in seconds.
        ticks: Number of MPC ticks to run in headless mode.
        host: Host for the ViserVisualizer.
        port: Port for the ViserVisualizer.
    """
    print(f"roboplan.aligator version: {al.__version__}")

    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

    package_paths = [get_package_share_dir()]
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()
    scene = Scene(
        "aligator_mpc_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )

    group_name = model_data.default_joint_group
    group_joints = set(scene.getJointGroupInfo(group_name).joint_names)
    tip_frame = model_data.ee_names[0]

    full, reduced, q_ref_full = _build_reduced_model(urdf_xml, group_joints)
    data = reduced.createData()
    tip_id = reduced.getFrameId(tip_frame)

    opt, handle = _make_optimizer(scene, group_name, tip_frame, horizon, dt, reduced.nv)

    # Plant state (reduced) and the tip's home position (the goal orbits this).
    q = pin.neutral(reduced)
    v = np.zeros(reduced.nv)
    home = _tip_position(reduced, data, tip_id, q)

    def goal_pose(t: float) -> np.ndarray:
        """A goal that orbits the tip's home position in a small circle."""
        pose = np.eye(4)
        pose[:3, 3] = home + np.array(
            [0.06 * np.cos(t), 0.06 * np.sin(t), 0.04 * np.sin(0.5 * t)]
        )
        return pose

    # First solve to prime the warm start.
    opt.setInitialState(q, v)
    handle.setTarget(goal_pose(0.0))
    solved = opt.solve(opt.interpolatePath([q, q]))

    if headless:
        _run_headless(
            opt, handle, reduced, data, tip_id, q, v, dt, ticks, goal_pose, solved
        )
        return

    _run_interactive(
        opt,
        handle,
        reduced,
        data,
        tip_id,
        full,
        q_ref_full,
        q,
        v,
        dt,
        home,
        urdf_xml,
        package_paths,
        host,
        port,
    )


def _run_headless(
    opt, handle, reduced, data, tip_id, q, v, dt, ticks, goal_pose, prev
) -> None:
    print(
        f"\n=== Receding-horizon MPC: {ticks} ticks, dt={dt}s, horizon={opt.horizon()} ==="
    )
    print("  tick   solve[ms]   tracking[mm]")
    solve_times = []
    for tick in range(ticks):
        target = goal_pose(tick * dt)
        handle.setTarget(target)
        opt.setInitialState(q, v)
        t0 = time.perf_counter()
        solved = opt.solve(opt.shift(prev))
        solve_ms = (time.perf_counter() - t0) * 1e3
        solve_times.append(solve_ms)
        # Apply controls[0]: step the plant one dt.
        q, v = _plant_step(reduced, data, q, v, np.asarray(solved.controls[0]), dt)
        err_mm = (
            np.linalg.norm(_tip_position(reduced, data, tip_id, q) - target[:3, 3])
            * 1e3
        )
        if tick % 5 == 0 or tick == ticks - 1:
            print(f"  {tick:4d}   {solve_ms:8.1f}   {err_mm:10.1f}")
        prev = solved
    print(
        f"\n  mean solve time: {np.mean(solve_times):.1f} ms  (max {np.max(solve_times):.1f} ms)"
    )
    # The tracker keeps up: the final tracking error is a small fraction of the orbit radius (60 mm).
    final_err = np.linalg.norm(
        _tip_position(reduced, data, tip_id, q) - goal_pose(ticks * dt)[:3, 3]
    )
    print(f"  final tracking error: {final_err * 1e3:.1f} mm")


def _run_interactive(
    opt,
    handle,
    reduced,
    data,
    tip_id,
    full,
    q_ref_full,
    q,
    v,
    dt,
    home,
    urdf_xml,
    package_paths,
    host,
    port,
) -> None:
    from pinocchio.visualize import ViserVisualizer

    collision_model = pin.buildGeomFromUrdfString(
        full, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        full, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )
    viz = ViserVisualizer(full, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)
    viz.display(_expand_reduced_to_full(full, reduced, q_ref_full, q))

    # A draggable marker the optimizer chases (initialized at the tip's home).
    marker_pose = np.eye(4)
    marker_pose[:3, 3] = home
    controls = viz.viewer.scene.add_transform_controls("/mpc_goal", scale=0.15)
    controls.position = tuple(home)
    latest = {"pose": marker_pose.copy()}

    @controls.on_update
    def _on_update(_) -> None:
        pose = np.eye(4)
        pose[:3, 3] = np.array(controls.position)
        latest["pose"] = pose

    print(
        "\n  Drag the marker in viser; the arm receding-horizon-tracks it (Ctrl+C to stop)."
    )
    prev = opt.solve(opt.interpolatePath([q, q]))
    try:
        while True:
            t_start = time.perf_counter()
            handle.setTarget(latest["pose"])
            opt.setInitialState(q, v)
            solved = opt.solve(opt.shift(prev))
            q, v = _plant_step(reduced, data, q, v, np.asarray(solved.controls[0]), dt)
            prev = solved
            viz.display(_expand_reduced_to_full(full, reduced, q_ref_full, q))
            time.sleep(max(0.0, dt - (time.perf_counter() - t_start)))
    except KeyboardInterrupt:
        print("\n  Stopped.")


if __name__ == "__main__":
    tyro.cli(main)
