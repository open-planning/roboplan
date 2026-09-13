#include <roboplan_aligator/cost_factory.hpp>

#include <stdexcept>
#include <string>

#include <roboplan_aligator/costs/masked_state_cost.hpp>
#include <roboplan_aligator/reduced_group_model.hpp>

namespace roboplan::aligator_detail {

void attachConfigurationCost(CostStack& stack, const PhaseSpace& space,
                             const ReducedGroupModel& rgm, const ConfigurationCost& spec,
                             double weight) {
  const int nq = rgm.nq();
  const int nv = rgm.nv();
  if (spec.q_target.size() != nq) {
    throw std::invalid_argument("ConfigurationCost: q_target size " +
                                std::to_string(spec.q_target.size()) + " != nq " +
                                std::to_string(nq) + ".");
  }
  if (spec.weights.size() != nv) {
    throw std::invalid_argument("ConfigurationCost: weights size " +
                                std::to_string(spec.weights.size()) + " != nv " +
                                std::to_string(nv) + ".");
  }

  // State target [q_target; 0]; the velocity block is masked out by the zero weight below.
  Eigen::VectorXd target = Eigen::VectorXd::Zero(nq + nv);
  target.head(nq) = spec.q_target;

  attachMaskedStateCost(stack, space, nv, target, spec.weights,
                        /*mask_configuration_block=*/true, weight);
}

}  // namespace roboplan::aligator_detail
