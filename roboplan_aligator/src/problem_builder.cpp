#include "problem_builder.hpp"

#include <utility>
#include <vector>

#include <aligator/core/stage-model.hpp>                 // StageModelTpl
#include <aligator/modelling/costs/quad-state-cost.hpp>  // QuadraticControlCostTpl
#include <aligator/modelling/costs/sum-of-costs.hpp>     // CostStackTpl
#include <aligator/modelling/dynamics/integrator-rk2.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#include <aligator/modelling/dynamics/multibody-free-fwd.hpp>

namespace roboplan::aligator_detail {

namespace {

// aligator uses value-semantics polymorphism (xyz::polymorphic) for spaces/costs/dynamics; each
// is deep-copied where it is stored (§3.5), so building one template and reusing it is safe.
using ManifoldPoly = xyz::polymorphic<aligator::ManifoldAbstractTpl<double>>;
using CostPoly = xyz::polymorphic<aligator::CostAbstractTpl<double>>;
using ODEPoly = xyz::polymorphic<aligator::dynamics::ODEAbstractTpl<double>>;

using CostStack = aligator::CostStackTpl<double>;
using QuadraticControlCost = aligator::QuadraticControlCostTpl<double>;
using StageModel = aligator::StageModelTpl<double>;
using MultibodyFreeFwdDynamics = aligator::dynamics::MultibodyFreeFwdDynamicsTpl<double>;
using IntegratorSemiImplEuler = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
using IntegratorRK2 = aligator::dynamics::IntegratorRK2Tpl<double>;

}  // namespace

PhaseSpace makePhaseSpace(const pinocchio::Model& reduced_model) {
  // MultibodyPhaseSpace(const ModelType&) copies the model into its MultibodyConfiguration, so the
  // returned space owns its model and does not alias the caller's (API_NOTES.md,
  // multibody.hpp:120).
  return {reduced_model};
}

DiscreteDynamics makeDiscreteDynamics(const PhaseSpace& space, IntegratorType type, double dt) {
  // Continuous free-space ABA dynamics. The single-argument ctor sets actuation B = identity, so
  // the control is joint torque with nu = nv (fully-actuated group, §3.2).
  ODEPoly ode = MultibodyFreeFwdDynamics(space);

  switch (type) {
  case IntegratorType::RK2:
    return {IntegratorRK2(ode, dt)};
  case IntegratorType::SemiImplicitEuler:
    return {IntegratorSemiImplEuler(ode, dt)};
  }
  // Defensive default: IntegratorType is a closed enum, so this is unreachable, but a well-defined
  // fallback keeps the function total.
  return {IntegratorSemiImplEuler(ode, dt)};
}

std::unique_ptr<Problem> buildProblemShell(const PhaseSpace& space, const Eigen::VectorXd& x0,
                                           int horizon, double dt, const TrajOptOptions& options) {
  ManifoldPoly space_poly = space;     // erased state manifold, copied by value
  const int nu = space.getModel().nv;  // fully-actuated: nu = nv

  // Terminal cost placeholder: an empty cost sum on the state space (costs arrive in later
  // prompts).
  CostPoly term_cost = CostStack(space_poly, nu);

  // The x0 + nu + space + term_cost ctor auto-builds the initial-condition (StateError) equality
  // constraint into the problem's init_constraint_ (API_NOTES.md, traj-opt-problem.hpp:151-154).
  auto problem = std::make_unique<Problem>(x0, nu, space_poly, term_cost);

  // One representative discretized-dynamics model; addStage deep-copies whatever it is handed, so
  // every stage gets an independent copy.
  DiscreteDynamics dynamics = makeDiscreteDynamics(space, options.integrator, dt);

  // Default quadratic control regularization: cost = 1/2 * control_reg * ||u||^2 (target u = 0).
  // Building the empty problem-shell as a truly zero-cost problem is degenerate for ProxDDP (the
  // control Hessian would be singular), so this baseline term makes the shell a well-posed
  // minimum-effort problem (maintainer decision, Prompt 5). control_reg <= 0 disables it (§4.1).
  const bool add_control_reg = options.control_reg > 0.0;
  const Eigen::MatrixXd control_weights =
      add_control_reg ? Eigen::MatrixXd(options.control_reg * Eigen::MatrixXd::Identity(nu, nu))
                      : Eigen::MatrixXd();

  for (int k = 0; k < horizon; ++k) {
    CostStack stage_cost(space_poly, nu);  // empty cost sum
    if (add_control_reg) {
      CostPoly u_reg = QuadraticControlCost(space_poly, nu, control_weights);
      stage_cost.addCost(u_reg, 1.0);
    }
    StageModel stage(CostPoly(stage_cost), dynamics);
    problem->addStage(stage);
  }

  return problem;
}

}  // namespace roboplan::aligator_detail
