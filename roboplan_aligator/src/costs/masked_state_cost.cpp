#include <roboplan_aligator/costs/masked_state_cost.hpp>

#include <aligator/modelling/costs/quad-state-cost.hpp>

namespace roboplan::aligator_detail {

namespace {

using ManifoldPoly = xyz::polymorphic<aligator::ManifoldAbstractTpl<double>>;
using CostPoly = xyz::polymorphic<aligator::CostAbstractTpl<double>>;

using QuadraticStateCost = aligator::QuadraticStateCostTpl<double>;

}  // namespace

void attachMaskedStateCost(CostStack& stack, const PhaseSpace& space, int nv,
                           const Eigen::VectorXd& target, const Eigen::VectorXd& block_weights,
                           bool mask_configuration_block, double cost_weight) {
  const int ndx = 2 * nv;
  Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(ndx, ndx);
  if (mask_configuration_block) {
    weights.diagonal().head(nv) = block_weights;
  } else {
    weights.diagonal().tail(nv) = block_weights;
  }

  QuadraticStateCost cost(ManifoldPoly(space), nv, target, weights);
  stack.addCost(CostPoly(cost), cost_weight);
}

}  // namespace roboplan::aligator_detail
