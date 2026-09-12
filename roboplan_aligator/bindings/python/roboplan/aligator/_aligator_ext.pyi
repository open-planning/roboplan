from collections.abc import Sequence
import enum
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray
import roboplan.core._core_ext


class IntegratorType(enum.Enum):
    """Which aligator integrator discretizes the multibody dynamics."""

    SemiImplicitEuler = 0

    RK2 = 1

class TrajOptOptions:
    """Options controlling the ProxDDP trajectory optimizer."""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, max_iters: int = 100, tol: float = 0.0001, mu_init: float = 0.01, integrator: IntegratorType = IntegratorType.SemiImplicitEuler, verbose: bool = False, control_reg: float = 0.001) -> None: ...

    @property
    def max_iters(self) -> int:
        """Maximum ProxDDP outer iterations."""

    @max_iters.setter
    def max_iters(self, arg: int, /) -> None: ...

    @property
    def tol(self) -> float:
        """Convergence tolerance."""

    @tol.setter
    def tol(self, arg: float, /) -> None: ...

    @property
    def mu_init(self) -> float:
        """Augmented-Lagrangian penalty initialization."""

    @mu_init.setter
    def mu_init(self, arg: float, /) -> None: ...

    @property
    def integrator(self) -> IntegratorType:
        """Dynamics integrator."""

    @integrator.setter
    def integrator(self, arg: IntegratorType, /) -> None: ...

    @property
    def verbose(self) -> bool:
        """Whether the solver prints per-iteration progress."""

    @verbose.setter
    def verbose(self, arg: bool, /) -> None: ...

    @property
    def control_reg(self) -> float:
        """
        Weight of the default quadratic control regularization (0 disables it).
        """

    @control_reg.setter
    def control_reg(self, arg: float, /) -> None: ...

class ConfigurationCost:
    """Penalize deviation of the reduced-group configuration from a target."""

    def __init__(self) -> None: ...

    @property
    def q_target(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Target reduced-group configuration (size nq)."""

    @q_target.setter
    def q_target(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def weights(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Per-DoF tangent weights (size nv)."""

    @weights.setter
    def weights(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class VelocityCost:
    """Penalize reduced-group velocity deviation from a target."""

    def __init__(self) -> None: ...

    @property
    def weights(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Per-DoF velocity weights (size nv)."""

    @weights.setter
    def weights(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def v_target(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Target velocity (size nv); empty means zero."""

    @v_target.setter
    def v_target(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class CostHandle:
    """
    Mutable handle to an attached cost, for target updates between solves. Returned by addCost; dangles if the optimizer is destroyed or resetProblem() is called.
    """

    def setTarget(self, target: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Set a new target vector for a ConfigurationCost/VelocityCost handle."""

class TorqueLimit:
    """
    Symmetric box limit on the control torque (defaults from the model effort).
    """

    def __init__(self) -> None: ...

    @property
    def tau_max(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Symmetric torque bound (size nv); empty means model effort limits."""

    @tau_max.setter
    def tau_max(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class TrajOptSeed:
    """Warm-start states/controls on the horizon grid (reduced-group layout)."""

    def __init__(self) -> None: ...

    @property
    def xs(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Per-knot state guesses x = [q; v] (size N + 1)."""

    @xs.setter
    def xs(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

    @property
    def us(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Per-stage control (torque) guesses (size N)."""

    @us.setter
    def us(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

class TrajOptTrajectory:
    """The optimized state trajectory sampled at dt (reduced-group layout)."""

    def __init__(self) -> None: ...

    @property
    def times(self) -> list[float]:
        """Sample times k*dt (size N + 1)."""

    @times.setter
    def times(self, arg: Sequence[float], /) -> None: ...

    @property
    def positions(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Reduced-group positions q at each time."""

    @positions.setter
    def positions(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

    @property
    def velocities(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Reduced-group velocities v at each time."""

    @velocities.setter
    def velocities(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

class TrajOptResult:
    """Result of a trajectory optimization solve."""

    def __init__(self) -> None: ...

    @property
    def converged(self) -> bool:
        """Whether the solver reached its tolerance."""

    @converged.setter
    def converged(self, arg: bool, /) -> None: ...

    @property
    def iterations(self) -> int:
        """Number of ProxDDP outer iterations taken."""

    @iterations.setter
    def iterations(self, arg: int, /) -> None: ...

    @property
    def cost(self) -> float:
        """Final total cost."""

    @cost.setter
    def cost(self, arg: float, /) -> None: ...

    @property
    def max_constraint_violation(self) -> float:
        """Largest constraint violation at the returned solution."""

    @max_constraint_violation.setter
    def max_constraint_violation(self, arg: float, /) -> None: ...

    @property
    def xs(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Raw solver state trajectory (size N + 1)."""

    @xs.setter
    def xs(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

    @property
    def us(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Raw solver control trajectory (size N)."""

    @us.setter
    def us(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

    @property
    def controls(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Joint-torque profile (size N); equals us for B = I."""

    @controls.setter
    def controls(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

    @property
    def trajectory(self) -> TrajOptTrajectory:
        """Optimized state trajectory sampled at dt."""

    @trajectory.setter
    def trajectory(self, arg: TrajOptTrajectory, /) -> None: ...

    def toRoboplan(self, scene: roboplan.core._core_ext.Scene, group_name: str) -> roboplan.core._core_ext.JointTrajectory:
        """
        Convert the optimized trajectory to a full-model roboplan.JointTrajectory (positions + times; velocities/accelerations empty; torques dropped).
        """

class TrajectoryOptimizer:
    """
    Trajectory optimizer wrapping aligator's proximal-DDP solver over reduced-model free-space multibody dynamics.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, group_name: str, horizon: int, dt: float, options: TrajOptOptions = ...) -> None: ...

    def horizon(self) -> int:
        """Number of stages N."""

    def dt(self) -> float:
        """Time step dt, in seconds."""

    def nq(self) -> int:
        """Reduced-model configuration size nq."""

    def nv(self) -> int:
        """Reduced-model tangent size nv."""

    def nx(self) -> int:
        """State dimension nx = nq + nv."""

    def setInitialState(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Set the fixed initial configuration state x0 = [q; 0] (hot-path)."""

    @overload
    def addCost(self, cost: ConfigurationCost, timesteps: object | None = None, weight: float = 1.0) -> CostHandle: ...

    @overload
    def addCost(self, cost: "xyz::polymorphic<aligator::CostAbstractTpl<double>, std::allocator<aligator::CostAbstractTpl<double> > >", timesteps: object | None = None, weight: float = 1.0) -> CostHandle: ...

    @overload
    def addConstraint(self, constraint: TorqueLimit, timesteps: object | None = None) -> None: ...

    @overload
    def addConstraint(self, residual: "xyz::polymorphic<aligator::StageFunctionTpl<double>, std::allocator<aligator::StageFunctionTpl<double> > >", set: "xyz::polymorphic<aligator::ConstraintSetTpl<double>, std::allocator<aligator::ConstraintSetTpl<double> > >", timesteps: object | None = None) -> None: ...

    def build(self) -> None:
        """
        Finalize the problem (allocate the solver workspace and freeze the structure); required before solve.
        """

    def resetProblem(self) -> None:
        """
        Rebuild the empty shell, re-enabling addCost/addConstraint (a fresh build() is required).
        """

    def interpolatePath(self, waypoints: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]) -> TrajOptSeed:
        """
        Straight-line warm-start seed through reduced-group waypoints onto the horizon grid.
        """

    def solve(self, seed: TrajOptSeed) -> TrajOptResult:
        """Run the ProxDDP solver from a warm-start seed (requires build())."""
