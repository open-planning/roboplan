#include <roboplan_aligator/problem_builder.hpp>

#include <aligator/modelling/costs/sum-of-costs.hpp>  // CostStackTpl
#include <aligator/modelling/dynamics/integrator-rk2.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#include <aligator/modelling/dynamics/multibody-free-fwd.hpp>

namespace roboplan::aligator_detail {

namespace {

// aligator uses value-semantics polymorphism (xyz::polymorphic) for spaces/costs/dynamics; each
// is deep-copied where it is stored, so building one template and reusing it is safe.
using ManifoldPoly = xyz::polymorphic<aligator::ManifoldAbstractTpl<double>>;
using CostPoly = xyz::polymorphic<aligator::CostAbstractTpl<double>>;
using ODEPoly = xyz::polymorphic<aligator::dynamics::ODEAbstractTpl<double>>;

using CostStack = aligator::CostStackTpl<double>;
using MultibodyFreeFwdDynamics = aligator::dynamics::MultibodyFreeFwdDynamicsTpl<double>;
using IntegratorSemiImplEuler = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
using IntegratorRK2 = aligator::dynamics::IntegratorRK2Tpl<double>;

}  // namespace

PhaseSpace makePhaseSpace(const pinocchio::Model& reduced_model) {
  // MultibodyPhaseSpace(const ModelType&) copies the model into its MultibodyConfiguration, so the
  // returned space owns its model and does not alias the caller's (multibody.hpp:120).
  return {reduced_model};
}

DiscreteDynamics makeDiscreteDynamics(const PhaseSpace& space, IntegratorType type, double dt) {
  // Continuous free-space ABA dynamics. The single-argument ctor sets actuation B = identity, so
  // the control is joint torque with nu = nv (fully-actuated group).
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

std::unique_ptr<Problem> buildEmptyProblem(const PhaseSpace& space, const Eigen::VectorXd& x0,
                                           int nu) {
  ManifoldPoly space_poly = space;  // erased state manifold, copied by value

  // Terminal cost placeholder: an empty cost sum on the state space. TrajectoryOptimizer::build()
  // replaces this wholesale once the terminal cost entries are known (aligator's own idiom for
  // updating term_cost_ -- a plain field assignment, e.g. external/aligator/tests/mpc-cycle.cpp).
  CostPoly term_cost = CostStack(space_poly, nu);

  // The x0 + nu + space + term_cost ctor auto-builds the initial-condition (StateError) equality
  // constraint into the problem's init_constraint_, and starts with an EMPTY stages_ list --
  // stages are added one at a time, fully formed, via addStage (traj-opt-problem.hpp:152-154,163).
  return std::make_unique<Problem>(x0, nu, space_poly, term_cost);
}

}  // namespace roboplan::aligator_detail
