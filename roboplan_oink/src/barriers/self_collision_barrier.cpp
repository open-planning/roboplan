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

SelfCollisionBarrier::SelfCollisionBarrier(const Oink& oink, const Scene& scene,
                                           int n_collision_pairs_, double dt, double gain,
                                           double safe_displacement_gain, double d_min_,
                                           double safety_margin)
    : Barrier(gain, dt, safe_displacement_gain, safety_margin),
      n_collision_pairs(n_collision_pairs_), d_min(d_min_), v_indices(oink.v_indices),
      collision_model(&scene.getCollisionModel()) {
  if (d_min < 0.0) {
    throw std::invalid_argument("SelfCollisionBarrier: d_min must be non-negative (got " +
                                std::to_string(d_min) + ")");
  }
  if (n_collision_pairs <= 0) {
    throw std::invalid_argument("SelfCollisionBarrier: n_collision_pairs must be positive (got " +
                                std::to_string(n_collision_pairs) + ")");
  }
  const auto total_pairs = static_cast<int>(collision_model->collisionPairs.size());
  if (n_collision_pairs > total_pairs) {
    throw std::invalid_argument("SelfCollisionBarrier: requested " +
                                std::to_string(n_collision_pairs) +
                                " collision pairs but the scene only has " +
                                std::to_string(total_pairs) + " collision pairs.");
  }

  initializeStorage(n_collision_pairs, oink.num_variables);
  closest_pair_indices.assign(n_collision_pairs, 0);
  all_distances = Eigen::VectorXd::Zero(total_pairs);

  const auto nv = scene.getModel().nv;
  joint_jacobian1 = Eigen::MatrixXd::Zero(6, nv);
  joint_jacobian2 = Eigen::MatrixXd::Zero(6, nv);
  full_row = Eigen::RowVectorXd::Zero(nv);

  eval_geom_data = pinocchio::GeometryData(*collision_model);
}

int SelfCollisionBarrier::getNumBarriers(const Scene& /*scene*/) const { return n_collision_pairs; }

tl::expected<void, std::string> SelfCollisionBarrier::computeBarrier(const Scene& scene) {
  const auto& geom_model = scene.getCollisionModel();
  const auto total_pairs = static_cast<int>(geom_model.collisionPairs.size());
  if (total_pairs < n_collision_pairs) {
    return tl::make_unexpected("SelfCollisionBarrier: scene has fewer collision pairs (" +
                               std::to_string(total_pairs) + ") than the barrier dimension (" +
                               std::to_string(n_collision_pairs) + ")");
  }

  // Update geometry placements and recompute every pair distance.
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();
  scene.computeCollisionDistances(q);

  const auto& geom_data = scene.getCollisionData();
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
  const auto& model = scene.getModel();
  const auto& data = scene.getData();
  const auto& geom_model = scene.getCollisionModel();
  const auto& geom_data = scene.getCollisionData();

  // Joint Jacobians are needed for parent-joint Jacobian extraction below.
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();
  scene.computeJointJacobians(q);

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

tl::expected<double, std::string>
SelfCollisionBarrier::evaluateAtConfiguration(const pinocchio::Model& model, pinocchio::Data& data,
                                              const Eigen::VectorXd& q) const {
  if (collision_model == nullptr) {
    return tl::make_unexpected("SelfCollisionBarrier: collision_model pointer is null");
  }

  // Refresh geometry placements at q, then run narrow-phase distance only on the pairs
  // that computeBarrier() identified as closest. This skips the full pair sweep that
  // pinocchio::computeDistances() would otherwise do.
  pinocchio::updateGeometryPlacements(model, data, *collision_model, eval_geom_data, q);

  double min_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n_collision_pairs; ++i) {
    const auto& result =
        pinocchio::computeDistance(*collision_model, eval_geom_data, closest_pair_indices[i]);
    min_distance = std::min(min_distance, result.min_distance);
  }
  return min_distance - d_min;
}

}  // namespace roboplan
