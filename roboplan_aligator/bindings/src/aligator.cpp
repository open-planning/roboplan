#include <nanobind/nanobind.h>

#include <string>

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include <aligator/core/constraint-set.hpp>
#include <aligator/core/cost-abstract.hpp>
#include <aligator/core/function-abstract.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>  // JointTrajectory (returned by TrajOptResult::toRoboplan)

#include <roboplan_aligator/constraint_spec.hpp>
#include <roboplan_aligator/constraints/torque_limit.hpp>
#include <roboplan_aligator/cost_spec.hpp>
#include <roboplan_aligator/costs/configuration_cost.hpp>
#include <roboplan_aligator/costs/velocity_cost.hpp>
#include <roboplan_aligator/trajectory_optimizer.hpp>
#include <roboplan_aligator/types.hpp>

#include <modules/aligator.hpp>
#include <roboplan_bindings/expected.hpp>

namespace roboplan {

namespace nb = nanobind;
using namespace nanobind::literals;

void init_aligator(nb::module_& m) {
  // --- Options + integrator -------------------------------------------------------------------

  nb::enum_<IntegratorType>(m, "IntegratorType",
                            "Which aligator integrator discretizes the multibody dynamics.")
      .value("SemiImplicitEuler", IntegratorType::SemiImplicitEuler)
      .value("RK2", IntegratorType::RK2);

  nb::enum_<aligator::LQSolverChoice>(m, "LQSolverChoice", "Riccati backend for the LQ subproblem.")
      .value("Serial", aligator::LQSolverChoice::SERIAL)
      .value("Parallel", aligator::LQSolverChoice::PARALLEL)
      .value("StagedDense", aligator::LQSolverChoice::STAGEDENSE);

  nb::enum_<aligator::RolloutType>(m, "RolloutType", "Forward-pass rollout used during the solve.")
      .value("Linear", aligator::RolloutType::LINEAR)
      .value("NonLinear", aligator::RolloutType::NONLINEAR);

  nb::class_<TrajOptOptions>(m, "TrajOptOptions",
                             "Options controlling the ProxDDP trajectory optimizer.")
      .def(nb::init<>())
      .def(
          "__init__",
          [](TrajOptOptions* self, int max_iters, double tol, double mu_init,
             IntegratorType integrator, bool verbose, double control_reg, bool record_history,
             aligator::LQSolverChoice linear_solver_choice, int num_threads,
             aligator::RolloutType rollout_type) {
            new (self)
                TrajOptOptions{max_iters,   tol,         mu_init,        integrator,
                               verbose,     control_reg, record_history, linear_solver_choice,
                               num_threads, rollout_type};
          },
          "max_iters"_a = 100, "tol"_a = 1e-4, "mu_init"_a = 1e-2,
          "integrator"_a = IntegratorType::SemiImplicitEuler, "verbose"_a = false,
          "control_reg"_a = 1e-3, "record_history"_a = false,
          "linear_solver_choice"_a = aligator::LQSolverChoice::SERIAL, "num_threads"_a = 1,
          "rollout_type"_a = aligator::RolloutType::LINEAR)
      .def_rw("max_iters", &TrajOptOptions::max_iters, "Maximum ProxDDP outer iterations.")
      .def_rw("tol", &TrajOptOptions::tol, "Convergence tolerance.")
      .def_rw("mu_init", &TrajOptOptions::mu_init, "Augmented-Lagrangian penalty initialization.")
      .def_rw("integrator", &TrajOptOptions::integrator, "Dynamics integrator.")
      .def_rw("verbose", &TrajOptOptions::verbose,
              "Whether the solver prints per-iteration progress.")
      .def_rw("control_reg", &TrajOptOptions::control_reg,
              "Weight of the default quadratic control regularization (0 disables it).")
      .def_rw("record_history", &TrajOptOptions::record_history,
              "Record per-iteration diagnostics into TrajOptResult.history (0 overhead when "
              "false).")
      .def_rw("linear_solver_choice", &TrajOptOptions::linear_solver_choice,
              "Riccati backend for the LQ subproblem; consumed once, in build().")
      .def_rw("num_threads", &TrajOptOptions::num_threads,
              "Thread count for the Parallel backend; consumed once, in build().")
      .def_rw("rollout_type", &TrajOptOptions::rollout_type,
              "Forward-pass rollout; consumed once, in build().");

  nb::class_<TrajOptIterate>(m, "TrajOptIterate", "One recorded ProxDDP iteration.")
      .def(nb::init<>())
      .def_rw("iteration", &TrajOptIterate::iteration,
              "Iteration index within the solve (0-based).")
      .def_rw("cost", &TrajOptIterate::cost, "Total trajectory cost at this iteration.")
      .def_rw("prim_infeas", &TrajOptIterate::prim_infeas,
              "Primal infeasibility (constraint violation) at this iteration.")
      .def_rw("dual_infeas", &TrajOptIterate::dual_infeas,
              "Dual infeasibility (stationarity residual) at this iteration.");

  // --- Costs (soft) ---------------------------------------------------------------------------

  nb::class_<ConfigurationCost>(
      m, "ConfigurationCost",
      "Penalize deviation of the reduced-group configuration from a target.")
      .def(nb::init<>())
      .def_rw("q_target", &ConfigurationCost::q_target,
              "Target reduced-group configuration (size nq).")
      .def_rw("weights", &ConfigurationCost::weights, "Per-DoF tangent weights (size nv).");

  nb::class_<VelocityCost>(m, "VelocityCost",
                           "Penalize reduced-group velocity deviation from a target.")
      .def(nb::init<>())
      .def_rw("weights", &VelocityCost::weights, "Per-DoF velocity weights (size nv).")
      .def_rw("v_target", &VelocityCost::v_target, "Target velocity (size nv); empty means zero.");

  // --- Constraints (hard) ---------------------------------------------------------------------

  nb::class_<TorqueLimit>(
      m, "TorqueLimit",
      "Symmetric box limit on the control torque (defaults from the model effort).")
      .def(nb::init<>())
      .def_rw("tau_max", &TorqueLimit::tau_max,
              "Symmetric torque bound (size nv); empty means model effort limits.");

  // --- Seed / result --------------------------------------------------------------------------

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

  nb::class_<TrajOptResult>(m, "TrajOptResult", "Result of a trajectory optimization solve.")
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
      .def_rw("history", &TrajOptResult::history,
              "Per-iteration diagnostics from this solve; empty unless "
              "TrajOptOptions.record_history was set.")
      .def("toRoboplan", &TrajOptResult::toRoboplan, "scene"_a, "group_name"_a,
           "Convert the optimized trajectory to a full-model roboplan.JointTrajectory (positions + "
           "times; velocities/accelerations empty; torques dropped).");

  // --- The optimizer ---------------------------------------------------------------------------

  nb::class_<TrajectoryOptimizer>(
      m, "TrajectoryOptimizer",
      "Trajectory optimizer wrapping aligator's proximal-DDP solver over reduced-model free-space "
      "multibody dynamics.")
      .def(nb::init<std::shared_ptr<Scene>, std::string, int, double, TrajOptOptions>(), "scene"_a,
           "group_name"_a, "horizon"_a, "dt"_a, "options"_a = TrajOptOptions{})
      .def("horizon", &TrajectoryOptimizer::horizon, "Number of stages N.")
      .def("dt", &TrajectoryOptimizer::dt, "Time step dt, in seconds.")
      .def("integrator", &TrajectoryOptimizer::integrator, "The configured dynamics integrator.")
      .def("nq", &TrajectoryOptimizer::nq, "Reduced-model configuration size nq.")
      .def("nv", &TrajectoryOptimizer::nv, "Reduced-model tangent size nv.")
      .def("nx", &TrajectoryOptimizer::nx, "State dimension nx = nq + nv.")
      .def("setInitialState", &TrajectoryOptimizer::setInitialState, "q"_a,
           "Set the fixed initial configuration state x0 = [q; 0] (hot-path).")
      // addCost/addStageCost/addTerminalCost: each overload accepts a concrete cost type and
      // wraps it in CostSpec for the unified C++ method. Consumed once, inside build() -- there is
      // no post-build retargeting; call resetProblem() + re-add + build() to change a target.
      .def(
          "addCost",
          [](TrajectoryOptimizer& self, const ConfigurationCost& cost, double weight) {
            self.addCost(CostSpec(cost), weight);
          },
          "cost"_a, "weight"_a = 1.0, "Attach a cost to every stage.")
      .def(
          "addCost",
          [](TrajectoryOptimizer& self, const VelocityCost& cost, double weight) {
            self.addCost(CostSpec(cost), weight);
          },
          "cost"_a, "weight"_a = 1.0, "Attach a cost to every stage.")
      .def(
          "addStageCost",
          [](TrajectoryOptimizer& self, int stage, const ConfigurationCost& cost, double weight) {
            self.addStageCost(stage, CostSpec(cost), weight);
          },
          "stage"_a, "cost"_a, "weight"_a = 1.0, "Attach a cost to exactly stage `stage`.")
      .def(
          "addStageCost",
          [](TrajectoryOptimizer& self, int stage, const VelocityCost& cost, double weight) {
            self.addStageCost(stage, CostSpec(cost), weight);
          },
          "stage"_a, "cost"_a, "weight"_a = 1.0, "Attach a cost to exactly stage `stage`.")
      .def(
          "addTerminalCost",
          [](TrajectoryOptimizer& self, const ConfigurationCost& cost, double weight) {
            self.addTerminalCost(CostSpec(cost), weight);
          },
          "cost"_a, "weight"_a = 1.0, "Attach a cost to the terminal node.")
      .def(
          "addTerminalCost",
          [](TrajectoryOptimizer& self, const VelocityCost& cost, double weight) {
            self.addTerminalCost(CostSpec(cost), weight);
          },
          "cost"_a, "weight"_a = 1.0, "Attach a cost to the terminal node.")
      // addConstraint/addStageConstraint/addTerminalConstraint: same pattern for constraint specs.
      .def(
          "addConstraint",
          [](TrajectoryOptimizer& self, const TorqueLimit& c) {
            self.addConstraint(ConstraintSpec(c));
          },
          "constraint"_a, "Attach a constraint to every stage.")
      .def(
          "addStageConstraint",
          [](TrajectoryOptimizer& self, int stage, const TorqueLimit& c) {
            self.addStageConstraint(stage, ConstraintSpec(c));
          },
          "stage"_a, "constraint"_a, "Attach a constraint to exactly stage `stage`.")
      .def(
          "addTerminalConstraint",
          [](TrajectoryOptimizer& self, const TorqueLimit& c) {
            self.addTerminalConstraint(ConstraintSpec(c));
          },
          "constraint"_a, "Attach a constraint to the terminal node.")
      .def(
          "build", &TrajectoryOptimizer::build,
          "Finalize the problem (allocate the solver workspace and freeze the structure); required "
          "before solve.")
      .def("resetProblem", &TrajectoryOptimizer::resetProblem,
           "Rebuild the empty shell, re-enabling addCost/addConstraint (a fresh build() is "
           "required).")
      .def("interpolatePath", &TrajectoryOptimizer::interpolatePath, "waypoints"_a,
           "Straight-line warm-start seed through reduced-group waypoints onto the horizon grid.")
      .def(
          "solve",
          [](TrajectoryOptimizer& self, const TrajOptSeed& seed) {
            return handle_expected(self.solve(seed));
          },
          "seed"_a, "Run the ProxDDP solver from a warm-start seed (requires build()).");
}

}  // namespace roboplan
