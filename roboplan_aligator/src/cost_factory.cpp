#include <roboplan_aligator/cost_factory.hpp>

#include <stdexcept>
#include <string>
#include <utility>

namespace roboplan {

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
