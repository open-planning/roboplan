#include <roboplan_aligator/cost_factory.hpp>

#include <stdexcept>
#include <string>
#include <utility>

#include <aligator/modelling/costs/quad-state-cost.hpp>

#include <roboplan_aligator/reduced_group_model.hpp>

namespace roboplan {

namespace aligator_detail {

namespace {

using ManifoldPoly = xyz::polymorphic<aligator::ManifoldAbstractTpl<double>>;
using CostPoly = xyz::polymorphic<aligator::CostAbstractTpl<double>>;

using QuadraticStateCost = aligator::QuadraticStateCostTpl<double>;
using CostItem = CostStack::CostItem;

}  // namespace

std::function<void(const Eigen::VectorXd&)>
attachConfigurationCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                        const ConfigurationCost& spec, double weight) {
  const int nq = rgm.nq();
  const int nv = rgm.nv();
  const int ndx = 2 * nv;
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
  Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(ndx, ndx);
  weights.diagonal().head(nv) = spec.weights;  // configuration-tangent block only

  QuadraticStateCost cost(ManifoldPoly(space), nv, target, weights);
  CostItem& item = stack.addCost(CostPoly(cost), weight);
  auto* stored = dynamic_cast<QuadraticStateCost*>(&*item.first);
  return [stored, nq, nv](const Eigen::VectorXd& q) {
    Eigen::VectorXd new_target = Eigen::VectorXd::Zero(nq + nv);
    new_target.head(nq) = q;
    stored->setTarget(new_target);
  };
}

std::function<void(const Eigen::VectorXd&)>
attachVelocityCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                   const VelocityCost& spec, double weight) {
  const int nq = rgm.nq();
  const int nv = rgm.nv();
  const int ndx = 2 * nv;
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
  Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(ndx, ndx);
  weights.diagonal().tail(nv) = spec.weights;  // velocity-tangent block only

  QuadraticStateCost cost(ManifoldPoly(space), nv, target, weights);
  CostItem& item = stack.addCost(CostPoly(cost), weight);
  auto* stored = dynamic_cast<QuadraticStateCost*>(&*item.first);
  return [stored, nq, nv, q_neutral](const Eigen::VectorXd& v) {
    Eigen::VectorXd new_target(nq + nv);
    new_target << q_neutral, v;
    stored->setTarget(new_target);
  };
}

}  // namespace aligator_detail

// --- CostHandle (public type; Impl defined in cost_factory.hpp) --------------------------------

CostHandle::CostHandle() = default;
CostHandle::CostHandle(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
CostHandle::~CostHandle() = default;
CostHandle::CostHandle(CostHandle&&) noexcept = default;
CostHandle& CostHandle::operator=(CostHandle&&) noexcept = default;

void CostHandle::setTarget(const Eigen::VectorXd& target) {
  if (!impl_) {
    throw std::logic_error(
        "CostHandle::setTarget: this handle was default-constructed (no attached cost); setTarget "
        "is meant for a handle returned by TrajectoryOptimizer::addCost.");
  }
  if (target.size() != impl_->expected_size) {
    throw std::invalid_argument("CostHandle::setTarget: target size " +
                                std::to_string(target.size()) + " does not match the cost's " +
                                std::to_string(impl_->expected_size) + ".");
  }
  for (auto& setter : impl_->vector_setters) {
    setter(target);
  }
}

}  // namespace roboplan
