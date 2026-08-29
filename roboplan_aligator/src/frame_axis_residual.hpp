#pragma once

// INTERNAL helper — NOT public API (lives under src/, not installed, not bound). The custom
// aligator residual backing FrameAxisCost (design §4.3): aligator ships no frame-vector/axis
// residual, so we implement one, mirroring the FramePlacementResidual pattern
// (modelling/multibody/frame-placement.{hpp,hxx}) — a UnaryFunctionTpl (depends on q only) whose
// Data owns a pinocchio::Data for the frame kinematics. Verified symbols recorded in API_NOTES.md
// (§ "Cost factories + mutable target").
//
//   r(q) = R_wf(q) * axis_local - axis_world_target                              (nr = 3)
//   dr/dq = -skew(R_wf(q) * axis_local) * Jw,   Jw = angular rows of the frame
//                                               Jacobian in LOCAL_WORLD_ALIGNED.
// For unit axes, wrapping r in a QuadraticResidualCost yields (1 - cos angle) up to scale — the
// vector-difference scalarization chosen for FrameAxisCost (maintainer decision, Prompt 6).

#include <memory>

#include <Eigen/Core>

#include <aligator/core/unary-function.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

namespace roboplan::aligator_detail {

/// @brief The 3x3 skew-symmetric matrix of a 3-vector (so skew(a)*b == a.cross(b)).
inline Eigen::Matrix3d skew3(const Eigen::Vector3d& v) {
  Eigen::Matrix3d s;
  s << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return s;
}

struct FrameAxisResidualData;

/// @brief Residual aligning a body-fixed axis in `frame` with a world-target direction.
struct FrameAxisResidual : aligator::UnaryFunctionTpl<double> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ALIGATOR_DYNAMIC_TYPEDEFS(double);
  ALIGATOR_UNARY_FUNCTION_INTERFACE(double);
  using BaseData = aligator::StageFunctionDataTpl<double>;
  using Model = pinocchio::ModelTpl<double>;
  using Data = FrameAxisResidualData;

  Model pin_model_;
  pinocchio::FrameIndex frame_id_;
  Eigen::Vector3d axis_local_;
  Eigen::Vector3d axis_world_target_;

  FrameAxisResidual(const int ndx, const int nu, const Model& model,
                    const pinocchio::FrameIndex frame_id, const Eigen::Vector3d& axis_local,
                    const Eigen::Vector3d& axis_world_target)
      : Base(ndx, nu, 3), pin_model_(model), frame_id_(frame_id), axis_local_(axis_local),
        axis_world_target_(axis_world_target) {}

  /// @brief Hot-path target update: change the desired world direction (design §3.5).
  void setAxisWorldTarget(const Eigen::Vector3d& axis_world_target) {
    axis_world_target_ = axis_world_target;
  }
  const Eigen::Vector3d& getAxisWorldTarget() const { return axis_world_target_; }

  void evaluate(const ConstVectorRef& x, BaseData& data) const override;
  void computeJacobians(const ConstVectorRef& x, BaseData& data) const override;
  // Defined out-of-line below, after Data is complete: this is a concrete (non-template) class, so
  // the body is instantiated eagerly and cannot reference an incomplete Data.
  std::shared_ptr<BaseData> createData() const override;
};

struct FrameAxisResidualData : aligator::StageFunctionDataTpl<double> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Base = aligator::StageFunctionDataTpl<double>;
  using PinData = pinocchio::DataTpl<double>;

  PinData pin_data_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian_;  ///< 6 x nv, reused each call.
  Eigen::Matrix3d frame_rotation_;                           ///< R_wf cached from evaluate().

  explicit FrameAxisResidualData(const FrameAxisResidual& model)
      : Base(model.ndx1, model.nu, 3), pin_data_(model.pin_model_),
        frame_jacobian_(Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.pin_model_.nv)),
        frame_rotation_(Eigen::Matrix3d::Identity()) {}
};

inline std::shared_ptr<FrameAxisResidual::BaseData> FrameAxisResidual::createData() const {
  return std::make_shared<Data>(*this);
}

inline void FrameAxisResidual::evaluate(const ConstVectorRef& x, BaseData& data) const {
  Data& d = static_cast<Data&>(data);
  const Eigen::VectorXd q = x.head(pin_model_.nq);
  pinocchio::forwardKinematics(pin_model_, d.pin_data_, q);
  pinocchio::updateFramePlacement(pin_model_, d.pin_data_, frame_id_);
  d.frame_rotation_ = d.pin_data_.oMf[frame_id_].rotation();
  d.value_.noalias() = d.frame_rotation_ * axis_local_;
  d.value_ -= axis_world_target_;
}

inline void FrameAxisResidual::computeJacobians(const ConstVectorRef& /*x*/, BaseData& data) const {
  Data& d = static_cast<Data&>(data);
  // Reuses the forward kinematics computed in evaluate() (aligator always calls evaluate first),
  // mirroring FramePlacementResidual::computeJacobians (frame-placement.hxx).
  pinocchio::computeJointJacobians(pin_model_, d.pin_data_);
  pinocchio::getFrameJacobian(pin_model_, d.pin_data_, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED,
                              d.frame_jacobian_);
  const Eigen::Vector3d axis_world = d.frame_rotation_ * axis_local_;
  const int nv = pin_model_.nv;
  // r depends on q only: the velocity-tangent columns (right nv) are zero.
  d.Jx_.setZero();
  d.Jx_.leftCols(nv).noalias() = -skew3(axis_world) * d.frame_jacobian_.bottomRows(3);
}

}  // namespace roboplan::aligator_detail
