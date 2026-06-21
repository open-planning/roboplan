#include <roboplan_oink/barriers/self_collision_barrier.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/collision/distance.hpp>

namespace roboplan {

SelfCollisionBarrier::SelfCollisionBarrier(const Oink& oink, const Scene& scene, double dt,
                                           const SelfCollisionBarrierOptions& options)
    : Barrier(options.gain, dt, options.safe_displacement_gain, options.safety_margin),
      n_collision_pairs(options.n_collision_pairs), d_min(options.d_min), d_max(options.d_max),
      v_indices(oink.v_indices) {
  if (d_min < 0.0) {
    throw std::invalid_argument("SelfCollisionBarrier: d_min must be non-negative (got " +
                                std::to_string(d_min) + ")");
  }
  if (n_collision_pairs <= 0) {
    throw std::invalid_argument("SelfCollisionBarrier: n_collision_pairs must be positive (got " +
                                std::to_string(n_collision_pairs) + ")");
  }

  // Clip the requested pair count to what the scene actually has.
  const auto total_pairs = static_cast<int>(scene.getCollisionModel().collisionPairs.size());
  n_collision_pairs = std::min(n_collision_pairs, total_pairs);

  initializeStorage(n_collision_pairs, oink.num_variables);
  closest_pair_indices.assign(n_collision_pairs, 0);
  all_distances = Eigen::VectorXd::Zero(total_pairs);

  const auto nv = scene.getModel().nv;
  joint_jacobian1 = Eigen::MatrixXd::Zero(6, nv);
  joint_jacobian2 = Eigen::MatrixXd::Zero(6, nv);
  full_row = Eigen::RowVectorXd::Zero(nv);

  // Share the Oink solver's collision scratch. Every distance and joint-Jacobian query below runs
  // on this context, so the barrier never touches scene state. The context is a snapshot of the
  // scene's collision geometry taken when the Oink was constructed.
  collision_context = &oink.getCollisionContext();
}

int SelfCollisionBarrier::getNumBarriers(const Scene& /*scene*/) const { return n_collision_pairs; }

tl::expected<void, std::string> SelfCollisionBarrier::computeBarrier(const Scene& scene) {
  const auto& geom_model = collision_context->getCollisionModel();
  const auto total_pairs = static_cast<int>(geom_model.collisionPairs.size());
  if (total_pairs < n_collision_pairs) {
    return tl::make_unexpected("SelfCollisionBarrier: scene has fewer collision pairs (" +
                               std::to_string(total_pairs) + ") than the barrier dimension (" +
                               std::to_string(n_collision_pairs) + ")");
  }

  // Update geometry placements and recompute pair distances on the barrier's own scratch. Pairs
  // whose bounding boxes are farther apart than d_max skip exact narrow-phase distance.
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();
  collision_context->computeDistances(q, d_max);

  const auto& geom_data = collision_context->getCollisionData();
  for (int k = 0; k < total_pairs; ++k) {
    all_distances[k] = geom_data.distanceResults[k].min_distance;
  }

  // Pick the n_collision_pairs smallest distances (most constraining barriers).
  std::vector<int> indices(total_pairs);
  std::iota(indices.begin(), indices.end(), 0);
  std::partial_sort(indices.begin(), indices.begin() + n_collision_pairs, indices.end(),
                    [&](int a, int b) { return all_distances[a] < all_distances[b]; });

  for (int i = 0; i < n_collision_pairs; ++i) {
    const int pair_idx = indices[i];
    closest_pair_indices[i] = static_cast<std::size_t>(pair_idx);
    barrier_values[i] = all_distances[pair_idx] - d_min;
  }

  return {};
}

tl::expected<void, std::string> SelfCollisionBarrier::computeJacobian(const Scene& scene) {
  const auto& model = collision_context->getModel();
  const auto& data = collision_context->getData();
  const auto& geom_model = collision_context->getCollisionModel();
  const auto& geom_data = collision_context->getCollisionData();

  // Joint Jacobians are needed for parent-joint Jacobian extraction below. They share the same
  // scratch Data as computeDistances(), so its forward kinematics and the witness points line up.
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();
  collision_context->computeJointJacobians(q);

  // NOTE: collision distances should have already been computed in computeBarrier(),
  // so we can reuse the witness points in geom_data without recomputing distances.
  jacobian_container.setZero();
  for (int i = 0; i < n_collision_pairs; ++i) {
    const std::size_t k = closest_pair_indices[i];
    const auto& coll_pair = geom_model.collisionPairs[k];
    const auto& dist_result = geom_data.distanceResults[k];

    const auto& geom_obj_1 = geom_model.geometryObjects[coll_pair.first];
    const auto& geom_obj_2 = geom_model.geometryObjects[coll_pair.second];

    const auto joint1_id = geom_obj_1.parentJoint;
    const auto joint2_id = geom_obj_2.parentJoint;

    const Eigen::Vector3d& witness_point_1 = dist_result.nearest_points[0];
    const Eigen::Vector3d& witness_point_2 = dist_result.nearest_points[1];

    // When the witness points coincide, the contact normal is undefined.
    // Skip this row (left at zero) to mirror Pink's behaviour.
    Eigen::Vector3d diff = witness_point_1 - witness_point_2;
    const double diff_norm = diff.norm();
    if (diff_norm < std::numeric_limits<double>::epsilon()) {
      continue;
    }
    diff = diff / diff_norm;

    // r_k: vector from joint k's origin to the witness point on body k (world frame).
    // Used as the lever arm coupling the joint's angular velocity into the witness point's
    // linear velocity via v = v_joint + ω × r.
    const Eigen::Vector3d r1 = witness_point_1 - data.oMi[joint1_id].translation();
    const Eigen::Vector3d r2 = witness_point_2 - data.oMi[joint2_id].translation();

    joint_jacobian1.setZero();
    joint_jacobian2.setZero();
    pinocchio::getJointJacobian(model, data, joint1_id,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint_jacobian1);
    pinocchio::getJointJacobian(model, data, joint2_id,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint_jacobian2);

    // J_i = n^T J^1_p + (r_1 × n)^T J^1_w - n^T J^2_p - (r_2 × n)^T J^2_w
    full_row.noalias() = diff.transpose() * joint_jacobian1.topRows<3>();
    full_row.noalias() += r1.cross(diff).transpose() * joint_jacobian1.bottomRows<3>();
    full_row.noalias() -= diff.transpose() * joint_jacobian2.topRows<3>();
    full_row.noalias() -= r2.cross(diff).transpose() * joint_jacobian2.bottomRows<3>();

    // Coal can produce NaN witness points / normals in pathological cases.
    // Mirror Pink's `np.nan_to_num`: replace NaN/inf with zero.
    for (int col = 0; col < full_row.size(); ++col) {
      if (!std::isfinite(full_row[col])) {
        full_row[col] = 0.0;
      }
    }

    jacobian_container.row(i) = full_row(v_indices);
  }

  return {};
}

tl::expected<double, std::string> SelfCollisionBarrier::evaluateAtConfiguration(
    const pinocchio::Model& /*model*/, pinocchio::Data& /*data*/, const Eigen::VectorXd& q) const {
  // Refresh geometry placements at q on the barrier's own scratch, then run narrow-phase distance
  // only on the pairs that computeBarrier() identified as closest. This skips the full pair sweep
  // that pinocchio::computeDistances() would otherwise do, and never touches scene state.
  const auto& model = collision_context->getModel();
  auto& data = collision_context->getData();
  const auto& geom_model = collision_context->getCollisionModel();
  auto& geom_data = collision_context->getCollisionData();
  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);

  double min_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n_collision_pairs; ++i) {
    const auto& result = pinocchio::computeDistance(geom_model, geom_data, closest_pair_indices[i]);
    min_distance = std::min(min_distance, result.min_distance);
  }
  return min_distance - d_min;
}

}  // namespace roboplan
