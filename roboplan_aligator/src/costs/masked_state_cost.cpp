#include <roboplan_aligator/costs/masked_state_cost.hpp>

#include <aligator/modelling/costs/quad-state-cost.hpp>

namespace roboplan::aligator_detail {

namespace {

using ManifoldPoly = xyz::polymorphic<aligator::ManifoldAbstractTpl<double>>;
using CostPoly = xyz::polymorphic<aligator::CostAbstractTpl<double>>;

using QuadraticStateCost = aligator::QuadraticStateCostTpl<double>;
using CostItem = CostStack::CostItem;

}  // namespace

std::function<void(const Eigen::VectorXd&)>
attachMaskedStateCost(CostStack& stack, const PhaseSpace& space, int nq, int nv,
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
  CostItem& item = stack.addCost(CostPoly(cost), cost_weight);
  auto* stored = dynamic_cast<QuadraticStateCost*>(&*item.first);

  const Eigen::VectorXd fixed_head = target.head(nq);
  const Eigen::VectorXd fixed_tail = target.tail(nv);
  return [stored, nq, nv, mask_configuration_block, fixed_head,
          fixed_tail](const Eigen::VectorXd& update) {
    Eigen::VectorXd new_target(nq + nv);
    if (mask_configuration_block) {
      new_target.head(nq) = update;
      new_target.tail(nv) = fixed_tail;
    } else {
      new_target.head(nq) = fixed_head;
      new_target.tail(nv) = update;
    }
    stored->setTarget(new_target);
  };
}

}  // namespace roboplan::aligator_detail
