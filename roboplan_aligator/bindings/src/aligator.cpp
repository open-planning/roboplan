#include <nanobind/nanobind.h>

#include <stdexcept>
#include <string>
#include <vector>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>  // JointTrajectory (returned by TrajOptResult::toRoboplan)

#include <roboplan_aligator/constraints.hpp>
#include <roboplan_aligator/costs.hpp>
#include <roboplan_aligator/trajectory_optimizer.hpp>
#include <roboplan_aligator/types.hpp>

#include <modules/aligator.hpp>
#include <roboplan_bindings/expected.hpp>

namespace roboplan {

namespace nb = nanobind;
using namespace nanobind::literals;

namespace {

// Maps the Python `timesteps` convenience to a StageWindow (design §3.3): `None` -> all stages;
// an `(begin, end)` tuple -> the half-open range; an int -> the terminal node. StageWindow itself
// is not exposed to Python — `timesteps` is the whole public surface for windowing.
StageWindow windowFromTimesteps(const nb::object& timesteps) {
  if (timesteps.is_none()) {
    return StageWindow::all();
  }
  if (nb::isinstance<nb::tuple>(timesteps)) {
    const auto range = nb::cast<nb::tuple>(timesteps);
    if (range.size() != 2) {
      throw std::invalid_argument(
          "timesteps: a tuple must be (begin, end) for a half-open stage range.");
    }
    return StageWindow::range(nb::cast<int>(range[0]), nb::cast<int>(range[1]));
  }
  if (nb::isinstance<nb::int_>(timesteps)) {
    return StageWindow::terminal();  // an int selects the terminal node (§3.3)
  }
  throw std::invalid_argument(
      "timesteps must be None (all stages), an (begin, end) tuple (a stage range), or an int (the "
      "terminal node).");
}

}  // namespace

void init_aligator(nb::module_& m) {
  // --- Options + integrator (design §4.1) -----------------------------------------------------

  nb::enum_<IntegratorType>(m, "IntegratorType",
                            "Which aligator integrator discretizes the multibody dynamics.")
      .value("SemiImplicitEuler", IntegratorType::SemiImplicitEuler)
      .value("RK2", IntegratorType::RK2);

  nb::class_<TrajOptOptions>(m, "TrajOptOptions",
                             "Options controlling the ProxDDP trajectory optimizer (design §4.1).")
      .def(nb::init<>())
      .def(
          "__init__",
          [](TrajOptOptions* self, int max_iters, double tol, double mu_init,
             IntegratorType integrator, bool verbose, double control_reg) {
            new (self) TrajOptOptions{max_iters, tol, mu_init, integrator, verbose, control_reg};
          },
          "max_iters"_a = 100, "tol"_a = 1e-4, "mu_init"_a = 1e-2,
          "integrator"_a = IntegratorType::SemiImplicitEuler, "verbose"_a = false,
          "control_reg"_a = 1e-3)
      .def_rw("max_iters", &TrajOptOptions::max_iters, "Maximum ProxDDP outer iterations.")
      .def_rw("tol", &TrajOptOptions::tol, "Convergence tolerance.")
      .def_rw("mu_init", &TrajOptOptions::mu_init, "Augmented-Lagrangian penalty initialization.")
      .def_rw("integrator", &TrajOptOptions::integrator, "Dynamics integrator.")
      .def_rw("verbose", &TrajOptOptions::verbose,
              "Whether the solver prints per-iteration progress.")
      .def_rw("control_reg", &TrajOptOptions::control_reg,
              "Weight of the default quadratic control regularization (0 disables it).");

  // --- Costs (soft) (design §4.3) -------------------------------------------------------------

  nb::class_<FramePoseCost>(m, "FramePoseCost",
                            "Penalize the SE3 placement error of a frame from a target pose.")
      .def(nb::init<>())
      .def_rw("frame", &FramePoseCost::frame, "Name of the frame whose pose is penalized.")
      .def_rw("target", &FramePoseCost::target, "Target pose as a 4x4 homogeneous transform.")
      .def_rw("position_cost", &FramePoseCost::position_cost,
              "Per-axis translation weights (x, y, z).")
      .def_rw("orientation_cost", &FramePoseCost::orientation_cost,
              "Per-axis rotation-log weights (rx, ry, rz).");

  nb::class_<FrameAxisCost>(m, "FrameAxisCost",
                            "Align a body-fixed axis with a world-target direction.")
      .def(nb::init<>())
      .def_rw("frame", &FrameAxisCost::frame, "Name of the frame carrying the body-fixed axis.")
      .def_rw("axis_local", &FrameAxisCost::axis_local,
              "The body-fixed axis, expressed in the frame.")
      .def_rw("axis_world_target", &FrameAxisCost::axis_world_target,
              "The desired world-space direction for the axis.")
      .def_rw("weight", &FrameAxisCost::weight, "Scalar weight on the 3-vector residual.");

  nb::class_<ConfigurationCost>(
      m, "ConfigurationCost",
      "Penalize deviation of the reduced-group configuration from a target.")
      .def(nb::init<>())
      .def_rw("q_target", &ConfigurationCost::q_target,
              "Target reduced-group configuration (size nq).")
      .def_rw("weights", &ConfigurationCost::weights, "Per-DoF tangent weights (size nv).");

  nb::class_<ControlCost>(m, "ControlCost",
                          "Penalize control (joint torque) deviation from a target.")
      .def(nb::init<>())
      .def_rw("weights", &ControlCost::weights, "Per-DoF control weights (size nv).")
      .def_rw("u_target", &ControlCost::u_target, "Target control (size nv); empty means zero.");

  nb::class_<VelocityCost>(m, "VelocityCost",
                           "Penalize reduced-group velocity deviation from a target.")
      .def(nb::init<>())
      .def_rw("weights", &VelocityCost::weights, "Per-DoF velocity weights (size nv).")
      .def_rw("v_target", &VelocityCost::v_target, "Target velocity (size nv); empty means zero.");

  nb::class_<CostHandle>(
      m, "CostHandle",
      "Mutable handle to an attached cost, for hot-path target updates (design §3.5). Returned by "
      "addCost; dangles if the optimizer is destroyed or resetProblem() is called.")
      .def(
          "setTarget",
          [](CostHandle& self, const Eigen::Matrix4d& target_pose) { self.setTarget(target_pose); },
          "target_pose"_a, "Set a new target pose (4x4) for a FramePoseCost handle.")
      .def(
          "setTarget",
          [](CostHandle& self, const Eigen::VectorXd& target) { self.setTarget(target); },
          "target"_a,
          "Set a new target vector for a ConfigurationCost/ControlCost/VelocityCost/FrameAxisCost "
          "handle.");

  // --- Constraints (hard) (design §4.4) -------------------------------------------------------

  nb::class_<PositionLimit>(
      m, "PositionLimit", "Box limit on the reduced-group configuration (defaults from the model).")
      .def(nb::init<>())
      .def_rw("q_min", &PositionLimit::q_min,
              "Lower position bound (size nq); empty means model default.")
      .def_rw("q_max", &PositionLimit::q_max,
              "Upper position bound (size nq); empty means model default.");

  nb::class_<VelocityLimit>(
      m, "VelocityLimit",
      "Symmetric box limit on the reduced-group velocity (defaults from the model).")
      .def(nb::init<>())
      .def_rw("v_max", &VelocityLimit::v_max,
              "Symmetric velocity bound (size nv); empty means model default.");

  nb::class_<TorqueLimit>(
      m, "TorqueLimit",
      "Symmetric box limit on the control torque (defaults from the model effort).")
      .def(nb::init<>())
      .def_rw("tau_max", &TorqueLimit::tau_max,
              "Symmetric torque bound (size nv); empty means model effort limits.");

  nb::class_<FramePoseConstraint>(m, "FramePoseConstraint",
                                  "Hard bound on a frame's SE3 placement error from a target pose.")
      .def(nb::init<>())
      .def_rw("frame", &FramePoseConstraint::frame, "Name of the frame whose pose is constrained.")
      .def_rw("target", &FramePoseConstraint::target, "Target pose as a 4x4 homogeneous transform.")
      .def_rw("tol_pos", &FramePoseConstraint::tol_pos,
              "Allowed translation error half-width (metres).")
      .def_rw("tol_rot", &FramePoseConstraint::tol_rot,
              "Allowed rotation-log error half-width (radians).");

  nb::class_<SelfCollisionConstraint>(
      m, "SelfCollisionConstraint", "Keep the robot's articulated links clear of each other (§5).")
      .def(nb::init<>())
      .def_rw("n_pairs", &SelfCollisionConstraint::n_pairs,
              "Number of closest self-collision pairs to constrain (<= 0 tracks all).")
      .def_rw("d_min", &SelfCollisionConstraint::d_min,
              "Minimum allowed signed distance (metres).");

  nb::class_<CollisionConstraint>(
      m, "CollisionConstraint", "Keep the robot's articulated links clear of static geometry (§5).")
      .def(nb::init<>())
      .def_rw("n_pairs", &CollisionConstraint::n_pairs,
              "Number of closest robot-vs-static pairs to constrain (<= 0 tracks all).")
      .def_rw("d_min", &CollisionConstraint::d_min, "Minimum allowed signed distance (metres).");

  // --- Seed / result (design §3.6, §4.5) ------------------------------------------------------

  nb::class_<TrajOptSeed>(m, "TrajOptSeed",
                          "Warm-start states/controls on the horizon grid (reduced-group layout).")
      .def(nb::init<>())
      .def_rw("xs", &TrajOptSeed::xs, "Per-knot state guesses x = [q; v] (size N + 1).")
      .def_rw("us", &TrajOptSeed::us, "Per-stage control (torque) guesses (size N).");

  nb::class_<TrajOptTrajectory>(
      m, "TrajOptTrajectory",
      "The optimized state trajectory sampled at dt (reduced-group layout).")
      .def(nb::init<>())
      .def_rw("times", &TrajOptTrajectory::times, "Sample times k*dt (size N + 1).")
      .def_rw("positions", &TrajOptTrajectory::positions, "Reduced-group positions q at each time.")
      .def_rw("velocities", &TrajOptTrajectory::velocities,
              "Reduced-group velocities v at each time.");

  nb::class_<TrajOptResult>(m, "TrajOptResult",
                            "Result of a trajectory optimization solve (design §4.5).")
      .def(nb::init<>())
      .def_rw("converged", &TrajOptResult::converged, "Whether the solver reached its tolerance.")
      .def_rw("iterations", &TrajOptResult::iterations, "Number of ProxDDP outer iterations taken.")
      .def_rw("cost", &TrajOptResult::cost, "Final total cost.")
      .def_rw("max_constraint_violation", &TrajOptResult::max_constraint_violation,
              "Largest constraint violation at the returned solution.")
      .def_rw("xs", &TrajOptResult::xs, "Raw solver state trajectory (size N + 1).")
      .def_rw("us", &TrajOptResult::us, "Raw solver control trajectory (size N).")
      .def_rw("controls", &TrajOptResult::controls,
              "Joint-torque profile (size N); equals us for B = I.")
      .def_rw("trajectory", &TrajOptResult::trajectory, "Optimized state trajectory sampled at dt.")
      .def("toRoboplan", &TrajOptResult::toRoboplan, "scene"_a, "group_name"_a,
           "Convert the optimized trajectory to a full-model roboplan.JointTrajectory (positions + "
           "times; velocities/accelerations empty; torques dropped).");

  // --- The optimizer (design §4.2) ------------------------------------------------------------

  nb::class_<TrajectoryOptimizer>(
      m, "TrajectoryOptimizer",
      "Trajectory optimizer wrapping aligator's proximal-DDP solver over reduced-model free-space "
      "multibody dynamics (design §4.2).")
      .def(nb::init<std::shared_ptr<Scene>, std::string, int, double, TrajOptOptions>(), "scene"_a,
           "group_name"_a, "horizon"_a, "dt"_a, "options"_a = TrajOptOptions{})
      .def("horizon", &TrajectoryOptimizer::horizon, "Number of stages N.")
      .def("dt", &TrajectoryOptimizer::dt, "Time step dt, in seconds.")
      .def("nq", &TrajectoryOptimizer::nq, "Reduced-model configuration size nq.")
      .def("nv", &TrajectoryOptimizer::nv, "Reduced-model tangent size nv.")
      .def("nx", &TrajectoryOptimizer::nx, "State dimension nx = nq + nv.")
      .def("setInitialState", &TrajectoryOptimizer::setInitialState, "q"_a,
           "v"_a = Eigen::VectorXd(),
           "Set the fixed initial state x0 = [q; v] (hot-path; empty v means zero velocity).")
      // addCost overloads: return a CostHandle whose setTarget mutates the in-problem residual.
      .def(
          "addCost",
          [](TrajectoryOptimizer& self, const FramePoseCost& cost, const nb::object& timesteps,
             double weight) { return self.addCost(cost, windowFromTimesteps(timesteps), weight); },
          "cost"_a, "timesteps"_a = nb::none(), "weight"_a = 1.0,
          nb::keep_alive<0, 1>())  // the CostHandle references the optimizer's in-problem residuals
      .def(
          "addCost",
          [](TrajectoryOptimizer& self, const FrameAxisCost& cost, const nb::object& timesteps,
             double weight) { return self.addCost(cost, windowFromTimesteps(timesteps), weight); },
          "cost"_a, "timesteps"_a = nb::none(), "weight"_a = 1.0,
          nb::keep_alive<0, 1>())  // the CostHandle references the optimizer's in-problem residuals
      .def(
          "addCost",
          [](TrajectoryOptimizer& self, const ConfigurationCost& cost, const nb::object& timesteps,
             double weight) { return self.addCost(cost, windowFromTimesteps(timesteps), weight); },
          "cost"_a, "timesteps"_a = nb::none(), "weight"_a = 1.0,
          nb::keep_alive<0, 1>())  // the CostHandle references the optimizer's in-problem residuals
      .def(
          "addCost",
          [](TrajectoryOptimizer& self, const ControlCost& cost, const nb::object& timesteps,
             double weight) { return self.addCost(cost, windowFromTimesteps(timesteps), weight); },
          "cost"_a, "timesteps"_a = nb::none(), "weight"_a = 1.0,
          nb::keep_alive<0, 1>())  // the CostHandle references the optimizer's in-problem residuals
      .def(
          "addCost",
          [](TrajectoryOptimizer& self, const VelocityCost& cost, const nb::object& timesteps,
             double weight) { return self.addCost(cost, windowFromTimesteps(timesteps), weight); },
          "cost"_a, "timesteps"_a = nb::none(), "weight"_a = 1.0,
          nb::keep_alive<0, 1>())  // the CostHandle references the optimizer's in-problem residuals
      // addConstraint overloads.
      .def(
          "addConstraint",
          [](TrajectoryOptimizer& self, const PositionLimit& c, const nb::object& timesteps) {
            self.addConstraint(c, windowFromTimesteps(timesteps));
          },
          "constraint"_a, "timesteps"_a = nb::none())
      .def(
          "addConstraint",
          [](TrajectoryOptimizer& self, const VelocityLimit& c, const nb::object& timesteps) {
            self.addConstraint(c, windowFromTimesteps(timesteps));
          },
          "constraint"_a, "timesteps"_a = nb::none())
      .def(
          "addConstraint",
          [](TrajectoryOptimizer& self, const TorqueLimit& c, const nb::object& timesteps) {
            self.addConstraint(c, windowFromTimesteps(timesteps));
          },
          "constraint"_a, "timesteps"_a = nb::none())
      .def(
          "addConstraint",
          [](TrajectoryOptimizer& self, const FramePoseConstraint& c, const nb::object& timesteps) {
            self.addConstraint(c, windowFromTimesteps(timesteps));
          },
          "constraint"_a, "timesteps"_a = nb::none())
      .def(
          "addConstraint",
          [](TrajectoryOptimizer& self, const SelfCollisionConstraint& c,
             const nb::object& timesteps) {
            self.addConstraint(c, windowFromTimesteps(timesteps));
          },
          "constraint"_a, "timesteps"_a = nb::none())
      .def(
          "addConstraint",
          [](TrajectoryOptimizer& self, const CollisionConstraint& c, const nb::object& timesteps) {
            self.addConstraint(c, windowFromTimesteps(timesteps));
          },
          "constraint"_a, "timesteps"_a = nb::none())
      .def(
          "build", &TrajectoryOptimizer::build,
          "Finalize the problem (allocate the solver workspace and freeze the structure); required "
          "before solve.")
      .def("resetProblem", &TrajectoryOptimizer::resetProblem,
           "Rebuild the empty shell, re-enabling addCost/addConstraint (a fresh build() is "
           "required).")
      .def("interpolatePath", &TrajectoryOptimizer::interpolatePath, "waypoints"_a,
           "Straight-line warm-start seed through reduced-group waypoints onto the horizon grid.")
      .def("shift", &TrajectoryOptimizer::shift, "result"_a, "n_steps"_a = 1,
           "Receding-horizon shift of a solved result into a warm-start seed for the next tick.")
      .def(
          "solve",
          [](TrajectoryOptimizer& self, const TrajOptSeed& seed) {
            return handle_expected(self.solve(seed));
          },
          "seed"_a, "Run the ProxDDP solver from a warm-start seed (requires build()).")
      .def(
          "solve",
          [](TrajectoryOptimizer& self, const TrajOptResult& previous) {
            return handle_expected(self.solve(previous));
          },
          "result"_a, "Run the solver warm-started from a previous result (requires build()).");
}

}  // namespace roboplan
