#include <roboplan_aligator/cost_factory.hpp>

#include <stdexcept>
#include <string>
#include <utility>

#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/quad-state-cost.hpp>
#include <aligator/modelling/multibody/frame-placement.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "frame_axis_residual.hpp"
#include <roboplan_aligator/reduced_group_model.hpp>

namespace roboplan {

namespace aligator_detail {

namespace {

using ManifoldPoly = xyz::polymorphic<aligator::ManifoldAbstractTpl<double>>;
using CostPoly = xyz::polymorphic<aligator::CostAbstractTpl<double>>;
using StageFuncPoly = xyz::polymorphic<aligator::StageFunctionTpl<double>>;

using QuadraticResidualCost = aligator::QuadraticResidualCostTpl<double>;
using QuadraticStateCost = aligator::QuadraticStateCostTpl<double>;
using QuadraticControlCost = aligator::QuadraticControlCostTpl<double>;
using FramePlacementResidual = aligator::FramePlacementResidualTpl<double>;
using CostItem = CostStack::CostItem;

}  // namespace

std::function<void(const Eigen::Matrix4d&)>
attachFramePoseCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                    const FramePoseCost& spec, double weight) {
  const pinocchio::FrameIndex frame_id = roboplan::resolveFrame(rgm, spec.frame, "FramePoseCost");
  const int ndx = space.ndx();
  const int nu = rgm.nv();

  // Per-axis 6x6 weight: translation block (log6 order: linear first) then rotation block.
  Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(6, 6);
  weights.diagonal().head(3) = spec.position_cost;
  weights.diagonal().tail(3) = spec.orientation_cost;

  const pinocchio::SE3 target(spec.target);  // explicit SE3 ctor from a 4x4 homogeneous transform
  FramePlacementResidual residual(ndx, nu, rgm.reducedModel(), target, frame_id);
  QuadraticResidualCost cost(ManifoldPoly(space), StageFuncPoly(residual), weights);

  CostItem& item = stack.addCost(CostPoly(cost), weight);
  auto* stored = dynamic_cast<QuadraticResidualCost*>(&*item.first);
  auto* residual_ptr = stored->getResidual<FramePlacementResidual>();
  return [residual_ptr](const Eigen::Matrix4d& target_pose) {
    residual_ptr->setReference(pinocchio::SE3(target_pose));
  };
}

std::function<void(const Eigen::VectorXd&)>
attachFrameAxisCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                    const FrameAxisCost& spec, double weight) {
  const pinocchio::FrameIndex frame_id = roboplan::resolveFrame(rgm, spec.frame, "FrameAxisCost");
  const int ndx = space.ndx();
  const int nu = rgm.nv();

  const Eigen::MatrixXd weights = spec.weight * Eigen::MatrixXd::Identity(3, 3);
  FrameAxisResidual residual(ndx, nu, rgm.reducedModel(), frame_id, spec.axis_local,
                             spec.axis_world_target);
  QuadraticResidualCost cost(ManifoldPoly(space), StageFuncPoly(residual), weights);

  CostItem& item = stack.addCost(CostPoly(cost), weight);
  auto* stored = dynamic_cast<QuadraticResidualCost*>(&*item.first);
  auto* residual_ptr = stored->getResidual<FrameAxisResidual>();
  return [residual_ptr](const Eigen::VectorXd& axis_world) {
    residual_ptr->setAxisWorldTarget(Eigen::Vector3d(axis_world));
  };
}

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

std::function<void(const Eigen::VectorXd&)>
attachControlCost(CostStack& stack, const PhaseSpace& space, const ReducedGroupModel& rgm,
                  const ControlCost& spec, double weight) {
  const int nv = rgm.nv();
  if (spec.weights.size() != nv) {
    throw std::invalid_argument("ControlCost: weights size " + std::to_string(spec.weights.size()) +
                                " != nv " + std::to_string(nv) + ".");
  }
  Eigen::VectorXd u_target = spec.u_target.size() == 0 ? Eigen::VectorXd::Zero(nv) : spec.u_target;
  if (u_target.size() != nv) {
    throw std::invalid_argument("ControlCost: u_target size " + std::to_string(u_target.size()) +
                                " != nv " + std::to_string(nv) + ".");
  }

  Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(nv, nv);
  weights.diagonal() = spec.weights;

  QuadraticControlCost cost(ManifoldPoly(space), nv, weights);  // zero target
  CostItem& item = stack.addCost(CostPoly(cost), weight);
  auto* stored = dynamic_cast<QuadraticControlCost*>(&*item.first);
  stored->setTarget(u_target);  // set on the in-problem copy
  return [stored](const Eigen::VectorXd& u) { stored->setTarget(u); };
}

}  // namespace aligator_detail

// --- CostHandle (public type; Impl defined in cost_factory.hpp) --------------------------------

CostHandle::CostHandle() = default;
CostHandle::CostHandle(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
CostHandle::~CostHandle() = default;
CostHandle::CostHandle(CostHandle&&) noexcept = default;
CostHandle& CostHandle::operator=(CostHandle&&) noexcept = default;

void CostHandle::setTarget(const Eigen::Matrix4d& target_pose) {
  if (!impl_ || impl_->kind != Impl::Kind::Pose) {
    throw std::logic_error(
        "CostHandle::setTarget(Matrix4d): this handle is not a FramePoseCost handle.");
  }
  for (auto& setter : impl_->pose_setters) {
    setter(target_pose);
  }
}

void CostHandle::setTarget(const Eigen::VectorXd& target) {
  if (!impl_ || impl_->kind != Impl::Kind::Vector) {
    throw std::logic_error(
        "CostHandle::setTarget(VectorXd): this handle is a FramePoseCost handle; use the Matrix4d "
        "overload.");
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
