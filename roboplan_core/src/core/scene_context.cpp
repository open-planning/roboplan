#include <roboplan/core/scene_context.hpp>

#include <limits>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/broadphase.hpp>
#include <pinocchio/collision/distance.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_utils.hpp>

namespace roboplan {

SceneContext::SceneContext(const Scene& scene)
    : scene_(scene), model_(scene.getModel()), collision_model_(scene.getCollisionModel()),
      data_(scene.getModel()), geom_data_(scene.getCollisionModel()),
      q_(scene.getCurrentJointPositions()), geometry_version_(scene.getGeometryVersion()) {
  // Seed pseudorandomly, matching Scene's own default. Callers that need reproducibility call
  // setRngSeed().
  std::random_device rd;
  rng_gen_ = std::mt19937(rd());

  // Bind a fresh broadphase manager to this context's own geometry data, then seed the geometry
  // world placements (at the neutral configuration) before the first AABB-tree build so coal does
  // not see degenerate bounding volumes. compute_local_aabb is false here on purpose.
  // Local AABBs live on the coal geometries, which are  shared with the Scene and other contexts.
  manager_.emplace(&model_, &collision_model_, &geom_data_);
  pinocchio::updateGeometryPlacements(model_, data_, collision_model_, geom_data_,
                                      pinocchio::neutral(model_));
  manager_->update(/*compute_local_aabb=*/false);
  aabb_objects_.reserve(collision_model_.geometryObjects.size());
  for (const auto& geom_obj : collision_model_.geometryObjects) {
    aabb_objects_.emplace_back(geom_obj.geometry, /*compute_local_aabb=*/false);
  }
}

bool SceneContext::isGeometryCurrent() const {
  return geometry_version_ == scene_.getGeometryVersion();
}

void SceneContext::checkGeometryCurrent(const char* what) const {
  if (isGeometryCurrent()) {
    return;
  }
  throw std::runtime_error(
      std::string("SceneContext::") + what +
      " was called after the scene's collision geometry changed. A context snapshots the geometry "
      "it was built from and sizes its own scratch to it, so it cannot answer queries against the "
      "new geometry. Rebuild the context (and any solver holding one) after adding or removing "
      "geometry or changing collision pairs.");
}

bool SceneContext::hasCollisions(const Eigen::VectorXd& q, const bool debug) const {
  checkGeometryCurrent("hasCollisions");
  if (!debug) {
    return pinocchio::computeCollisions(model_, data_, *manager_, q, /*stopAtFirstCollision=*/true);
  }
  return computeCollisionsVerbose(model_, data_, collision_model_, geom_data_, q);
}

void SceneContext::computeDistances(const Eigen::VectorXd& q,
                                    std::optional<double> broadphase_margin) const {
  checkGeometryCurrent("computeDistances");

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

void SceneContext::computeJointJacobians(const Eigen::VectorXd& q) const {
  scene_.computeJointJacobians(data_, q);
}

void SceneContext::updateFramePlacements(const Eigen::VectorXd& q) const {
  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
}

Eigen::Matrix4d SceneContext::forwardKinematics(const Eigen::VectorXd& q,
                                                const std::string& frame_name,
                                                const std::string& base_frame) const {
  return scene_.forwardKinematics(data_, q, frame_name, base_frame);
}

void SceneContext::computeFrameJacobian(const Eigen::VectorXd& q, pinocchio::FrameIndex frame_id,
                                        pinocchio::ReferenceFrame reference_frame,
                                        Eigen::Ref<Eigen::MatrixXd> jacobian) const {
  scene_.computeFrameJacobian(data_, q, frame_id, reference_frame, jacobian);
}

void SceneContext::computeRelativeFrameJacobian(const Eigen::VectorXd& q,
                                                pinocchio::FrameIndex frame_id,
                                                const std::string& base_frame,
                                                pinocchio::ReferenceFrame reference_frame,
                                                Eigen::Ref<Eigen::MatrixXd> jacobian) const {
  scene_.computeRelativeFrameJacobian(data_, q, frame_id, base_frame, reference_frame, jacobian);
}

void SceneContext::setRngSeed(unsigned int seed) { rng_gen_ = std::mt19937(seed); }

Eigen::VectorXd SceneContext::randomPositions() { return scene_.randomPositions(rng_gen_); }

void SceneContext::randomizeJointPositions(const std::vector<std::string>& joint_names,
                                           Eigen::VectorXd& q) {
  scene_.randomizeJointPositions(rng_gen_, joint_names, q);
}

std::optional<Eigen::VectorXd> SceneContext::randomCollisionFreePositions(size_t max_samples) {
  for (size_t idx = 0; idx < max_samples; ++idx) {
    const auto positions = randomPositions();
    if (!hasCollisions(positions)) {
      return positions;
    }
  }
  return std::nullopt;
}

void SceneContext::setJointPositions(const Eigen::VectorXd& q) {
  if (q.size() != model_.nq) {
    throw std::invalid_argument(
        "SceneContext::setJointPositions: expected " + std::to_string(model_.nq) +
        " configuration values (model.nq), got " + std::to_string(q.size()) + ".");
  }
  q_ = q;
}

Eigen::VectorXd SceneContext::toFullJointPositions(const std::string& group_name,
                                                   const Eigen::VectorXd& q) const {
  return scene_.toFullJointPositions(group_name, q, q_);
}

}  // namespace roboplan
