#include <roboplan/core/collision_context.hpp>

#include <limits>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/broadphase.hpp>
#include <pinocchio/collision/distance.hpp>

#include <roboplan/core/scene.hpp>

namespace roboplan {

CollisionContext::CollisionContext(const Scene& scene)
    : model_(scene.getModel()), collision_model_(scene.getCollisionModel()),
      data_(scene.getModel()), geom_data_(scene.getCollisionModel()) {
  // Bind a fresh broadphase manager to this context's own geometry data, then seed the geometry
  // world placements (at the neutral configuration) before the first AABB-tree build so coal does
  // not see degenerate bounding volumes. Mirrors Scene::rebuildBroadphaseManager().
  manager_.emplace(&model_, &collision_model_, &geom_data_);
  pinocchio::updateGeometryPlacements(model_, data_, collision_model_, geom_data_,
                                      pinocchio::neutral(model_));
  manager_->update(/*compute_local_aabb=*/true);

  // One coal collision object per geometry, used by computeDistances() to refresh world AABBs for
  // broadphase culling. The constructor computes each geometry's local AABB once. reserve() keeps
  // the objects address-stable during the fill (only relevant for re-entrancy, not correctness).
  aabb_objects_.reserve(collision_model_.geometryObjects.size());
  for (const auto& geom_obj : collision_model_.geometryObjects) {
    aabb_objects_.emplace_back(geom_obj.geometry, /*compute_local_aabb=*/true);
  }
}

bool CollisionContext::hasCollisions(const Eigen::VectorXd& q) const {
  return pinocchio::computeCollisions(model_, data_, *manager_, q, /*stopAtFirstCollision=*/true);
}

void CollisionContext::computeDistances(const Eigen::VectorXd& q,
                                        std::optional<double> broadphase_margin) const {
  // Without a margin there is nothing to cull: fall back to the exact all-pairs sweep.
  if (!broadphase_margin.has_value()) {
    pinocchio::computeDistances(model_, data_, collision_model_, geom_data_, q);
    return;
  }
  const double margin = *broadphase_margin;

  // Refresh joint + geometry placements on our own scratch, then update each geometry's world AABB.
  pinocchio::updateGeometryPlacements(model_, data_, collision_model_, geom_data_, q);
  for (std::size_t i = 0; i < aabb_objects_.size(); ++i) {
    const auto& world_T_geom = geom_data_.oMg[i];
    aabb_objects_[i].setTransform(world_T_geom.rotation(), world_T_geom.translation());
    aabb_objects_[i].computeAABB();
  }

  // For each active pair, use the AABB gap (a lower bound on the true distance) to decide whether
  // the exact narrow-phase distance is worth computing.
  const auto& pairs = collision_model_.collisionPairs;
  for (std::size_t k = 0; k < pairs.size(); ++k) {
    auto& result = geom_data_.distanceResults[k];
    if (!geom_data_.activeCollisionPairs[k]) {
      result.min_distance = std::numeric_limits<double>::infinity();
      continue;
    }

    const auto& pair = pairs[k];
    const double aabb_gap =
        aabb_objects_[pair.first].getAABB().distance(aabb_objects_[pair.second].getAABB());
    if (aabb_gap > margin) {
      // Provably farther than the margin: skip GJK/EPA. Record the (looser) lower bound as the
      // distance and collapse the witness points so any Jacobian built from them is a zero row.
      result.min_distance = aabb_gap;
      result.nearest_points[0].setZero();
      result.nearest_points[1].setZero();
    } else {
      pinocchio::computeDistance(collision_model_, geom_data_, k);
    }
  }
}

void CollisionContext::computeJointJacobians(const Eigen::VectorXd& q) const {
  pinocchio::computeJointJacobians(model_, data_, q);
}

}  // namespace roboplan
