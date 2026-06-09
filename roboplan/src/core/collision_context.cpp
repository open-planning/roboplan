#include <roboplan/core/collision_context.hpp>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/broadphase.hpp>

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
}

bool CollisionContext::hasCollisions(const Eigen::VectorXd& q) const {
  return pinocchio::computeCollisions(model_, data_, *manager_, q, /*stopAtFirstCollision=*/true);
}

}  // namespace roboplan
