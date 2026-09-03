#pragma once

// INTERNAL helper — NOT public API (lives under src/, not installed, not bound). The custom
// aligator residual backing SelfCollisionConstraint / CollisionConstraint (design §5, reuse path).
//
// Why custom, not aligator's FrameCollisionResidualTpl: that residual builds its distance Jacobian
// as `coal_normal^T * (J_p2 - J_p1)` (frame-collision.hxx:63-65), but coal leaves
// `DistanceResult.normal` UNPOPULATED (zero) for mesh/BVH distance queries — intermittently, even
// for the same pair at different configurations. A zero normal ⇒ a zero (useless) collision
// gradient, so on mesh robots (every RoboPlan fixture) the solver gets no signal to steer away from
// contact.
//
// This residual is byte-for-byte aligator's FrameCollisionResidual EXCEPT it recovers the normal
// from the witness points that coal DOES populate: n = (p2 - p1) / ||p2 - p1||. By the envelope
// theorem the distance gradient is exactly `n^T (J_p2 - J_p1)` (witness sliding is second order),
// which this finite-difference-tests to ~1e-11 on the mesh fixtures. Everything else (coal
// distance, pinocchio frame Jacobians shifted to the witness points, the reduced-model path so
// columns are already in reduced-`v`) is unchanged. This is the "§5 build custom only for the gap"
// — the gap being coal's missing mesh normal. Verified symbols recorded in API_NOTES.md.
//
//   r(q) = signed distance of one collision pair                                    (nr = 1)
//   dr/dq = n^T (J_p2 - J_p1),  n = (p2 - p1)/||p2 - p1||,  J_pi = LOCAL_WORLD_ALIGNED
//           parent-frame Jacobian shifted to witness point pi (linear rows).

#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include <aligator/core/unary-function.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace roboplan::aligator_detail {

struct CollisionDistanceResidualData;

/// @brief Signed-distance residual for one collision pair, with a witness-point analytic Jacobian.
/// @details A UnaryFunctionTpl (depends on q only), nr = 1. Copies the model + geometry model by
/// value (like aligator's FrameCollisionResidual); pass the REDUCED model + reduced geometry so the
/// Jacobian columns land in reduced-`v` with no remap. `pair_id` indexes
/// `geom_model.collisionPairs`.
struct CollisionDistanceResidual : aligator::UnaryFunctionTpl<double> {
  ALIGATOR_DYNAMIC_TYPEDEFS(double);
  ALIGATOR_UNARY_FUNCTION_INTERFACE(double);
  using BaseData = aligator::StageFunctionDataTpl<double>;
  using Model = pinocchio::Model;
  using GeometryModel = pinocchio::GeometryModel;

  Model pin_model_;
  GeometryModel geom_model_;
  pinocchio::PairIndex pair_id_;
  pinocchio::FrameIndex frame_id1_;
  pinocchio::FrameIndex frame_id2_;

  CollisionDistanceResidual(const int ndx, const int nu, const Model& model,
                            const GeometryModel& geom_model, const pinocchio::PairIndex pair_id)
      : Base(ndx, nu, 1), pin_model_(model), geom_model_(geom_model), pair_id_(pair_id) {
    if (pair_id_ >= geom_model_.collisionPairs.size()) {
      throw std::out_of_range("CollisionDistanceResidual: collision pair index " +
                              std::to_string(pair_id_) + " out of range (" +
                              std::to_string(geom_model_.collisionPairs.size()) + " pairs).");
    }
    const auto& pair = geom_model_.collisionPairs[pair_id_];
    frame_id1_ = geom_model_.geometryObjects[pair.first].parentFrame;
    frame_id2_ = geom_model_.geometryObjects[pair.second].parentFrame;
  }

  void evaluate(const ConstVectorRef& x, BaseData& data) const;
  void computeJacobians(const ConstVectorRef& x, BaseData& data) const;
  std::shared_ptr<BaseData> createData() const;
};

/// @brief Data for CollisionDistanceResidual: owns a private pinocchio Data + GeometryData (one
/// kinematics/geometry pass per residual) and the two witness-point-shifted frame Jacobians.
struct CollisionDistanceResidualData : aligator::StageFunctionDataTpl<double> {
  using Base = aligator::StageFunctionDataTpl<double>;

  pinocchio::Data pin_data_;
  pinocchio::GeometryData geom_data_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian1_;  // 6 x nv, witness point 1
  Eigen::Matrix<double, 6, Eigen::Dynamic> frame_jacobian2_;  // 6 x nv, witness point 2

  explicit CollisionDistanceResidualData(const CollisionDistanceResidual& residual)
      : Base(residual.ndx1, residual.nu, 1), pin_data_(residual.pin_model_),
        geom_data_(residual.geom_model_), frame_jacobian1_(6, residual.pin_model_.nv),
        frame_jacobian2_(6, residual.pin_model_.nv) {
    frame_jacobian1_.setZero();
    frame_jacobian2_.setZero();
  }
};

inline void CollisionDistanceResidual::evaluate(const ConstVectorRef& x, BaseData& data) const {
  auto& d = static_cast<CollisionDistanceResidualData&>(data);
  const auto& q = x.head(pin_model_.nq);
  pinocchio::forwardKinematics(pin_model_, d.pin_data_, q);
  pinocchio::updateFramePlacements(pin_model_, d.pin_data_);
  pinocchio::updateGeometryPlacements(pin_model_, d.pin_data_, geom_model_, d.geom_data_, q);
  pinocchio::computeDistance(geom_model_, d.geom_data_, pair_id_);
  d.value_[0] = d.geom_data_.distanceResults[pair_id_].min_distance;
}

inline void CollisionDistanceResidual::computeJacobians(const ConstVectorRef& /*x*/,
                                                        BaseData& data) const {
  auto& d = static_cast<CollisionDistanceResidualData&>(data);
  const auto& result = d.geom_data_.distanceResults[pair_id_];
  const Eigen::Vector3d p1 = result.nearest_points[0];
  const Eigen::Vector3d p2 = result.nearest_points[1];

  // Frame Jacobians of the two witness points, in LOCAL_WORLD_ALIGNED, shifted from the parent
  // frame origin to the witness point (a pure translation). (Uses the frame placements cached by
  // evaluate, which aligator always calls first.)
  pinocchio::SE3 frame_to_witness1 = pinocchio::SE3::Identity();
  frame_to_witness1.translation(p1 - d.pin_data_.oMf[frame_id1_].translation());
  pinocchio::SE3 frame_to_witness2 = pinocchio::SE3::Identity();
  frame_to_witness2.translation(p2 - d.pin_data_.oMf[frame_id2_].translation());

  pinocchio::computeJointJacobians(pin_model_, d.pin_data_);
  pinocchio::getFrameJacobian(pin_model_, d.pin_data_, frame_id1_, pinocchio::LOCAL_WORLD_ALIGNED,
                              d.frame_jacobian1_);
  pinocchio::getFrameJacobian(pin_model_, d.pin_data_, frame_id2_, pinocchio::LOCAL_WORLD_ALIGNED,
                              d.frame_jacobian2_);
  d.frame_jacobian1_ = frame_to_witness1.toActionMatrixInverse() * d.frame_jacobian1_;
  d.frame_jacobian2_ = frame_to_witness2.toActionMatrixInverse() * d.frame_jacobian2_;

  // Normal from the witness points coal DOES populate (its reported normal is unreliable/zero for
  // mesh distance). At contact (||p2 - p1|| ~ 0) the direction is undefined, so leave a zero row.
  d.Jx_.setZero();
  const Eigen::Vector3d segment = p2 - p1;
  const double distance = segment.norm();
  if (distance > 1e-9) {
    const Eigen::Vector3d normal = segment / distance;
    d.Jx_.leftCols(pin_model_.nv).noalias() =
        normal.transpose() * (d.frame_jacobian2_.topRows<3>() - d.frame_jacobian1_.topRows<3>());
  }
}

// Out-of-line (non-template class, so an inline createData would instantiate CollisionDistance-
// ResidualData while still incomplete). Mirrors frame_axis_residual.hpp.
inline std::shared_ptr<CollisionDistanceResidual::BaseData>
CollisionDistanceResidual::createData() const {
  return std::make_shared<CollisionDistanceResidualData>(*this);
}

}  // namespace roboplan::aligator_detail
