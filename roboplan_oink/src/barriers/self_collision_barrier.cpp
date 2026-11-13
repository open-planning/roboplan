#include <roboplan_oink/barriers/self_collision_barrier.hpp>

#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

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

  // Clip the requested pair count to what the scene actually has. The request is kept so that
  // computeBarrier() can re-clip if the collision geometry changes later.
  requested_collision_pairs = n_collision_pairs;
  const auto total_pairs = static_cast<int>(scene.getCollisionModel().collisionPairs.size());
  n_collision_pairs = std::min(n_collision_pairs, total_pairs);

  initializeStorage(n_collision_pairs, oink.num_variables);
  closest_pair_indices.assign(n_collision_pairs, 0);
  all_distances = Eigen::VectorXd::Zero(total_pairs);

  full_row = Eigen::RowVectorXd::Zero(scene.getModel().nv);

  // Track the Oink solver rather than its context. Every distance and joint-Jacobian query below
  // runs on the solver's current context, so the barrier never touches scene state and never holds
  // a context the solver has since replaced.
  this->oink = &oink;
}

int SelfCollisionBarrier::getNumBarriers(const SceneContext& /*context*/) const {
  return n_collision_pairs;
}

tl::expected<void, std::string> SelfCollisionBarrier::computeBarrier(const SceneContext& context) {
  const auto& scene_context = this->context();
  const auto& geom_model = scene_context.getCollisionModel();
  const auto total_pairs = static_cast<int>(geom_model.collisionPairs.size());
  if (total_pairs <= 0) {
    return tl::make_unexpected("SelfCollisionBarrier: scene has no collision pairs to constrain");
  }

  // Re-clip the barrier dimension to the geometry the solver's context currently holds. Adding or
  // removing collision pairs changes total_pairs, and every buffer below is sized from it, so this
  // must run before they are written. Oink queries getNumBarriers() after computeBarrier(), so a
  // changed dimension is picked up when the QP is sized.
  const int active_pairs = std::min(requested_collision_pairs, total_pairs);
  if (active_pairs != n_collision_pairs) {
    n_collision_pairs = active_pairs;
    initializeStorage(n_collision_pairs, oink->num_variables);
    closest_pair_indices.assign(n_collision_pairs, 0);
  }
  if (all_distances.size() != total_pairs) {
    all_distances = Eigen::VectorXd::Zero(total_pairs);
  }

  // Update geometry placements and recompute pair distances on the solver's context. Pairs
  // whose bounding boxes are farther apart than d_max skip exact narrow-phase distance.
  const Eigen::VectorXd& q = context.getJointPositions();
  scene_context.computeDistances(q, d_max);

  const auto& geom_data = scene_context.getCollisionData();
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

tl::expected<void, std::string> SelfCollisionBarrier::computeJacobian(const SceneContext& context) {
  const auto& scene_context = this->context();

  // Joint Jacobians share the same scratch Data as computeDistances(), so its forward kinematics
  // and the witness points line up. Distances were already computed in computeBarrier(), so the
  // witness points and normals in the context's GeometryData are reused without recomputation.
  const Eigen::VectorXd& q = context.getJointPositions();
  scene_context.computeJointJacobians(q);

  jacobian_container.setZero();
  for (int i = 0; i < n_collision_pairs; ++i) {
    scene_context.computeDistanceJacobian(closest_pair_indices[i], full_row);
    jacobian_container.row(i) = full_row(v_indices);
  }

  return {};
}

tl::expected<double, std::string> SelfCollisionBarrier::evaluateAtConfiguration(
    const pinocchio::Model& /*model*/, pinocchio::Data& /*data*/, const Eigen::VectorXd& q) const {
  // Refresh geometry placements at q on the solver's context, then run narrow-phase distance
  // only on the pairs that computeBarrier() identified as closest. This skips the full pair sweep
  // that pinocchio::computeDistances() would otherwise do, and never touches scene state.
  const auto& scene_context = this->context();
  const auto& model = scene_context.getModel();
  auto& data = scene_context.getData();
  const auto& geom_model = scene_context.getCollisionModel();
  auto& geom_data = scene_context.getCollisionData();
  pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data, q);

  double min_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < n_collision_pairs; ++i) {
    const auto& result = pinocchio::computeDistance(geom_model, geom_data, closest_pair_indices[i]);
    min_distance = std::min(min_distance, result.min_distance);
  }
  return min_distance - d_min;
}

}  // namespace roboplan
