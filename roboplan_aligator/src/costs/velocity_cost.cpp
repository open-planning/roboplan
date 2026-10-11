#include <roboplan_aligator/cost_factory.hpp>

#include <stdexcept>
#include <string>

#include <roboplan_aligator/costs/masked_state_cost.hpp>
#include <roboplan_aligator/reduced_group_model.hpp>

namespace roboplan::aligator_detail {

void attachVelocityCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                        const VelocityCost& spec, double weight) {
  const int nq = rgm.nq();
  const int nv = rgm.nv();
  if (spec.weights.size() != nv) {
    throw std::invalid_argument("VelocityCost: weights size " +
                                std::to_string(spec.weights.size()) + " != nv " +
                                std::to_string(nv) + ".");
  }
  Eigen::VectorXd v_target = spec.v_target.size() == 0 ? Eigen::VectorXd::Zero(nv) : spec.v_target;
  if (v_target.size() != nv) {
    throw std::invalid_argument("VelocityCost: v_target size " + std::to_string(v_target.size()) +
                                " != nv " + std::to_string(nv) + ".");
  }

  // The configuration block is masked out, but the state-error residual still differences q against
  // the target's q part, so use the model's neutral configuration there (a valid config point).
  const Eigen::VectorXd q_neutral = space.neutral().head(nq);
  Eigen::VectorXd target(nq + nv);
  target << q_neutral, v_target;

  attachMaskedStateCost(stack, space, nv, target, spec.weights,
                        /*mask_configuration_block=*/false, weight);
}

}  // namespace roboplan::aligator_detail
