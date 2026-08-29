"""Unit tests for the roboplan_aligator Python bindings (design doc §4)."""

import numpy as np
import pytest

from roboplan.aligator import (
    CollisionConstraint,
    ConfigurationCost,
    ControlCost,
    FrameAxisCost,
    FramePoseConstraint,
    IntegratorType,
    PositionLimit,
    SelfCollisionConstraint,
    TorqueLimit,
    TrajectoryOptimizer,
    TrajOptOptions,
    TrajOptResult,
    TrajOptSeed,
    VelocityLimit,
)
from roboplan.core import JointTrajectory, Scene
from roboplan.example_models import get_package_models_dir, get_package_share_dir

GROUP_NAME = "arm"  # SO-101 arm group (gripper joint locked): nq == nv == 5.


@pytest.fixture
def scene() -> Scene:
    models_dir = get_package_models_dir()
    return Scene(
        "test_scene",
        models_dir / "so101_robot_model" / "so101.urdf",
        models_dir / "so101_robot_model" / "so101.srdf",
        [get_package_share_dir()],
    )


def make_optimizer(scene: Scene, **kwargs) -> TrajectoryOptimizer:
    options = TrajOptOptions(max_iters=kwargs.pop("max_iters", 100))
    return TrajectoryOptimizer(
        scene,
        GROUP_NAME,
        kwargs.pop("horizon", 15),
        kwargs.pop("dt", 0.05),
        options,
    )


def test_construction_and_introspection(scene: Scene) -> None:
    opt = make_optimizer(scene, horizon=20, dt=0.02)
    assert opt.horizon() == 20
    assert opt.dt() == pytest.approx(0.02)
    assert opt.nq() == 5
    assert opt.nv() == 5
    assert opt.nx() == 10


def test_options_defaults_and_fields() -> None:
    options = TrajOptOptions()
    assert options.max_iters == 100
    assert options.integrator == IntegratorType.SemiImplicitEuler
    options.integrator = IntegratorType.RK2
    options.control_reg = 0.0
    assert options.integrator == IntegratorType.RK2
    assert options.control_reg == 0.0


def test_timesteps_maps_to_stage_windows(scene: Scene) -> None:
    # None -> all stages, (a, b) -> range, int -> terminal (design §3.3). All three must attach
    # without error; an out-of-range range must raise.
    opt = make_optimizer(scene, horizon=10)
    cost = ControlCost()
    cost.weights = np.ones(opt.nv())
    opt.addCost(cost)  # timesteps=None -> all
    opt.addCost(cost, timesteps=(2, 6))  # range
    opt.addCost(cost, timesteps=opt.horizon())  # int -> terminal
    with pytest.raises(ValueError):
        opt.addCost(cost, timesteps=(0, 999))  # end past the horizon


def test_lifecycle_build_gate(scene: Scene) -> None:
    opt = make_optimizer(scene)
    cost = ControlCost()
    cost.weights = np.ones(opt.nv())
    opt.addCost(cost)

    # solve() before build() is a recoverable error.
    with pytest.raises(RuntimeError):
        opt.solve(TrajOptSeed())

    opt.build()
    # addCost after build() is illegal until resetProblem().
    with pytest.raises(RuntimeError):
        opt.addCost(cost)
    result = opt.solve(TrajOptSeed())  # solves fine after build
    assert len(result.xs) == opt.horizon() + 1

    opt.resetProblem()
    opt.addCost(cost)  # legal again after reset


def test_interpolate_path_seed(scene: Scene) -> None:
    opt = make_optimizer(scene, horizon=10)
    q0 = np.zeros(opt.nq())
    q1 = np.full(opt.nq(), 0.3)
    seed = opt.interpolatePath([q0, q1])
    assert len(seed.xs) == opt.horizon() + 1
    assert len(seed.us) == opt.horizon()
    # Endpoints match the waypoints; velocities are zero.
    assert np.allclose(seed.xs[0][: opt.nq()], q0)
    assert np.allclose(seed.xs[-1][: opt.nq()], q1)
    assert np.allclose(seed.xs[0][opt.nq() :], 0.0)


def test_shift_repeats_last_knot(scene: Scene) -> None:
    opt = make_optimizer(scene, horizon=8, max_iters=20)
    cost = ConfigurationCost()
    cost.q_target = np.full(opt.nq(), 0.2)
    cost.weights = np.full(opt.nv(), 20.0)
    opt.addCost(cost, timesteps=opt.horizon())
    opt.build()
    result = opt.solve(TrajOptSeed())
    shifted = opt.shift(result, 1)
    assert len(shifted.xs) == opt.horizon() + 1
    # The tail holds the final knot (repeat-last convention).
    assert np.allclose(shifted.xs[-1], result.xs[-1])
    assert np.allclose(shifted.xs[0], result.xs[1])


def test_reach_converges_and_is_deterministic(scene: Scene) -> None:
    def solve_reach() -> TrajOptResult:
        opt = make_optimizer(scene, horizon=40, dt=0.05, max_iters=200)
        cost = ConfigurationCost()
        cost.q_target = np.full(opt.nq(), 0.3)
        cost.weights = np.full(opt.nv(), 100.0)
        opt.addCost(cost, timesteps=opt.horizon())
        opt.build()
        return opt.solve(TrajOptSeed())

    result = solve_reach()
    # The terminal configuration reaches the target neighborhood. 0.05 rad: a terminal-only reach
    # need not be exact (no terminal velocity cost), but must clearly converge.
    assert np.linalg.norm(result.xs[-1][:5] - 0.3) < 0.05

    # Same-seed determinism (testing rule): a fresh identical solve reproduces the trajectory.
    # 1e-9 is ~5 orders below the solver tolerance yet above FP/OpenMP reduction-order noise.
    other = solve_reach()
    assert len(other.us) == len(result.us)
    for a, b in zip(result.us, other.us):
        assert np.max(np.abs(a - b)) < 1e-9


def test_set_target_hot_path_and_solve_from_result(scene: Scene) -> None:
    opt = make_optimizer(scene, horizon=30, dt=0.05, max_iters=200)
    cost = ConfigurationCost()
    cost.q_target = np.full(opt.nq(), 0.3)
    cost.weights = np.full(opt.nv(), 100.0)
    handle = opt.addCost(cost, timesteps=opt.horizon())
    opt.build()

    first = opt.solve(TrajOptSeed())
    handle.setTarget(np.full(opt.nq(), -0.3))  # hot-path retarget
    # solve(previous result) dispatch warm-starts from the first solution.
    second = opt.solve(first)
    assert np.linalg.norm(second.xs[-1][:5] - (-0.3)) < np.linalg.norm(
        first.xs[-1][:5] - (-0.3)
    )


def test_result_trajectory_and_to_roboplan(scene: Scene) -> None:
    opt = make_optimizer(scene, horizon=12, dt=0.05, max_iters=20)
    opt.build()
    result = opt.solve(TrajOptSeed())
    # trajectory is a typed TrajOptTrajectory (not a dict).
    assert len(result.trajectory.times) == opt.horizon() + 1
    assert len(result.trajectory.positions) == opt.horizon() + 1
    assert result.trajectory.times[1] == pytest.approx(0.05)
    # toRoboplan yields a core JointTrajectory in full-model layout, positions and velocities
    # both expanded via Scene::toFullJoint{Positions,Velocities}.
    joint_traj = result.toRoboplan(scene, GROUP_NAME)
    assert isinstance(joint_traj, JointTrajectory)
    assert len(joint_traj.positions) == opt.horizon() + 1
    assert len(joint_traj.velocities) == opt.horizon() + 1
    assert len(joint_traj.accelerations) == 0


def test_constraint_specs_attach(scene: Scene) -> None:
    # Every constraint spec is constructible and attaches through the public surface.
    opt = make_optimizer(scene, horizon=10, max_iters=5)
    opt.addConstraint(PositionLimit())
    opt.addConstraint(VelocityLimit())
    opt.addConstraint(TorqueLimit())
    reach = FramePoseConstraint()
    reach.frame = "gripper_link"
    reach.tol_pos = 0.05
    reach.tol_rot = 0.2
    opt.addConstraint(reach, timesteps=opt.horizon())
    self_collision = SelfCollisionConstraint()
    self_collision.n_pairs = 2
    self_collision.d_min = 0.005
    opt.addConstraint(self_collision)
    opt.addConstraint(CollisionConstraint(), timesteps=opt.horizon())
    opt.build()
    assert np.isfinite(opt.solve(TrajOptSeed()).max_constraint_violation)


def test_torque_limit_rejects_terminal(scene: Scene) -> None:
    opt = make_optimizer(scene, horizon=8)
    # The terminal node has no control, so a torque box there is ill-defined.
    with pytest.raises(ValueError):
        opt.addConstraint(TorqueLimit(), timesteps=opt.horizon())


def test_frame_axis_cost_attaches_and_retargets(scene: Scene) -> None:
    # FrameAxisCost -> a handle whose vector setTarget(3) retargets the world axis (hot-path).
    opt = make_optimizer(scene, horizon=10, max_iters=5)
    axis = FrameAxisCost()
    axis.frame = "gripper_link"
    axis.axis_local = np.array([0.0, 0.0, 1.0])
    axis.axis_world_target = np.array([1.0, 0.0, 0.0])
    handle = opt.addCost(axis, timesteps=opt.horizon())
    opt.build()
    opt.solve(TrajOptSeed())
    handle.setTarget(np.array([0.0, 1.0, 0.0]))  # 3-vector world axis
    result = opt.solve(TrajOptSeed())  # solves after retarget
    assert len(result.us) == opt.horizon()


def test_cost_handle_keeps_optimizer_alive() -> None:
    # A CostHandle references the optimizer's in-problem residuals; keep_alive must keep the
    # optimizer alive so setTarget on a handle from a temporary optimizer does not use-after-free.
    import gc

    models_dir = get_package_models_dir()
    local_scene = Scene(
        "ka_scene",
        models_dir / "so101_robot_model" / "so101.urdf",
        models_dir / "so101_robot_model" / "so101.srdf",
        [get_package_share_dir()],
    )
    cost = ConfigurationCost()
    cost.q_target = np.zeros(5)
    cost.weights = np.ones(5)
    # The optimizer is a temporary; only the handle is kept.
    handle = TrajectoryOptimizer(local_scene, GROUP_NAME, 8, 0.05).addCost(
        cost, timesteps=8
    )
    gc.collect()
    handle.setTarget(np.zeros(5))  # must not crash (keep_alive<0, 1>)
