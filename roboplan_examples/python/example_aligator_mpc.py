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
    urdf_xml: str, locked_joint_names: list[str]
) -> tuple[pin.Model, pin.Model, np.ndarray]:
    """Full + reduced pinocchio models for the planning group (mirrors the C++ ReducedGroupModel).
    `locked_joint_names` comes from Scene.getLockedJointNames."""
    full = pin.buildModelFromXML(urdf_xml)
    q_ref_full = pin.neutral(full)
    joints_to_lock = [full.getJointId(name) for name in locked_joint_names]
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


def _tip_pose(reduced: pin.Model, data, tip_id: int, q: np.ndarray) -> np.ndarray:
    """World pose (4x4) of the tip frame at reduced configuration q."""
    pin.forwardKinematics(reduced, data, q)
    pin.updateFramePlacement(reduced, data, tip_id)
    return data.oMf[tip_id].homogeneous.copy()


def _rotation_error_deg(r_reached: np.ndarray, r_target: np.ndarray) -> float:
    """Geodesic angle between two rotation matrices, in degrees."""
    return float(np.degrees(np.linalg.norm(pin.log3(r_reached.T @ r_target))))


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
    # position_cost was 300 with orientation tracking disabled (orientation_cost=0, "chasing the
    # carrot", position only). Now that the carrot also has an orientation, position_cost is raised
    # to 900 alongside a nonzero orientation_cost=50: the two costs compete for the same limited
    # control authority each tick (max_iters=15, 1s horizon), and at 300/50 position tracking
    # regressed (never settled below its own initial gap); re-tuned empirically on the headless
    # circle+wobble goal to a combination where both position and orientation visibly converge
    # toward a bounded tracking lag instead of drifting away.
    pose.position_cost = np.full(3, 900.0)
    pose.orientation_cost = np.full(3, 50.0)
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
    tip_frame = model_data.ee_names[0]

    full, reduced, q_ref_full = _build_reduced_model(
        urdf_xml, scene.getLockedJointNames(group_name)
    )
    data = reduced.createData()
    tip_id = reduced.getFrameId(tip_frame)

    opt, handle = _make_optimizer(scene, group_name, tip_frame, horizon, dt, reduced.nv)

    # Plant state (reduced) and the tip's home pose (the goal orbits/wobbles around this).
    q = pin.neutral(reduced)
    v = np.zeros(reduced.nv)
    home_pose = _tip_pose(reduced, data, tip_id, q)
    home = home_pose[:3, 3]
    home_rot = home_pose[:3, :3]

    def goal_pose(t: float) -> np.ndarray:
        """A goal that orbits the tip's home position in a small circle and wobbles its orientation
        (+/- ~17 deg about a fixed local axis) so the example exercises orientation tracking too, not
        just position (Issue 3)."""
        pose = np.eye(4)
        pose[:3, 3] = home + np.array(
            [0.06 * np.cos(t), 0.06 * np.sin(t), 0.04 * np.sin(0.5 * t)]
        )
        angle = 0.3 * np.sin(0.3 * t)
        pose[:3, :3] = home_rot @ pin.exp3(angle * np.array([0.0, 1.0, 0.0]))
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
        home_pose,
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
    print("  tick   solve[ms]   pos err[mm]   rot err[deg]")
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
        reached = _tip_pose(reduced, data, tip_id, q)
        err_mm = np.linalg.norm(reached[:3, 3] - target[:3, 3]) * 1e3
        rot_err_deg = _rotation_error_deg(reached[:3, :3], target[:3, :3])
        if tick % 5 == 0 or tick == ticks - 1:
            print(
                f"  {tick:4d}   {solve_ms:8.1f}   {err_mm:10.1f}   {rot_err_deg:11.1f}"
            )
        prev = solved
    print(
        f"\n  mean solve time: {np.mean(solve_times):.1f} ms  (max {np.max(solve_times):.1f} ms)"
    )
    # The tracker keeps up: the final tracking error is a small fraction of the orbit radius (60 mm)
    # and the wobble amplitude (~17 deg).
    final_target = goal_pose(ticks * dt)
    final_reached = _tip_pose(reduced, data, tip_id, q)
    final_err = np.linalg.norm(final_reached[:3, 3] - final_target[:3, 3])
    final_rot_err_deg = _rotation_error_deg(final_reached[:3, :3], final_target[:3, :3])
    print(
        f"  final tracking error: {final_err * 1e3:.1f} mm, {final_rot_err_deg:.1f} deg"
    )


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
    home_pose,
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

    # A draggable marker the optimizer chases (initialized at the tip's home pose, incl. orientation).
    home = home_pose[:3, 3]
    controls = viz.viewer.scene.add_transform_controls("/mpc_goal", scale=0.15)
    controls.position = tuple(home)
    # wxyz is viser's [w, x, y, z] quaternion; pin.Quaternion(matrix).coeffs() returns [x, y, z, w]
    # (verified pattern: example_oink.py:345, example_ik.py:138).
    controls.wxyz = pin.Quaternion(home_pose[:3, :3]).coeffs()[[3, 0, 1, 2]]
    latest = {"pose": home_pose.copy()}

    @controls.on_update
    def _on_update(_) -> None:
        # controls.wxyz is viser's [w, x, y, z] quaternion; pin.Quaternion takes [x, y, z, w]
        # (verified pattern: example_oink.py:241-243, example_ik.py:104).
        pose = pin.SE3(
            pin.Quaternion(controls.wxyz[[1, 2, 3, 0]]), np.array(controls.position)
        ).homogeneous
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
