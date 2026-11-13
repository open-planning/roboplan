#include <roboplan/core/scene_context.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/broadphase.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_utils.hpp>

namespace roboplan {

SceneContext::SceneContext(const Scene& scene)
    : scene_(scene), model_(scene.getModel()), collision_model_(scene.getCollisionModel()),
      data_(scene.getModel()), geom_data_(scene.getCollisionModel()),
      q_(scene.getCurrentJointPositions()), geometry_version_(scene.getGeometryVersion()) {
  // Seed pseudorandomly like Scene; call setRngSeed() for reproducibility.
  std::random_device rd;
  rng_gen_ = std::mt19937(rd());

  // Bind a fresh broadphase manager to this context's own geometry data, then seed the geometry
  // world placements (at the neutral configuration) before the first AABB-tree build so coal does
  // not see degenerate bounding volumes. compute_local_aabb is false here on purpose.
  // Local AABBs live on the coal geometries, which are shared with the Scene and other contexts.
  manager_.emplace(&model_, &collision_model_, &geom_data_);
  pinocchio::updateGeometryPlacements(model_, data_, collision_model_, geom_data_,
                                      pinocchio::neutral(model_));
  manager_->update(/*compute_local_aabb=*/false);
  aabb_objects_.reserve(collision_model_.geometryObjects.size());
  for (const auto& geom_obj : collision_model_.geometryObjects) {
    aabb_objects_.emplace_back(geom_obj.geometry, /*compute_local_aabb=*/false);
  }
  joint_jacobian_1_ = Eigen::MatrixXd::Zero(6, model_.nv);
  joint_jacobian_2_ = Eigen::MatrixXd::Zero(6, model_.nv);
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
  const auto pairs = getCollidingPairs(q);
  for (const auto& pair : pairs) {
    std::cout << "Collision detected between " << pair << std::endl;
  }
  return !pairs.empty();
}

std::vector<std::string> SceneContext::getCollidingPairs(const Eigen::VectorXd& q) const {
  checkGeometryCurrent("getCollidingPairs");
  pinocchio::updateGeometryPlacements(model_, data_, collision_model_, geom_data_, q);
  pinocchio::computeCollisions(model_, data_, collision_model_, geom_data_, q,
                               /*stop_at_first_collision=*/false);

  // URDF geometries are named "<link>_<index>". Anything else is an object added to the scene.
  const auto body_name = [&](const pinocchio::GeomIndex index) -> const std::string& {
    const auto& geometry = collision_model_.geometryObjects.at(index);
    const auto& link = model_.frames.at(geometry.parentFrame).name;
    return geometry.name.rfind(link, 0) == 0 ? link : geometry.name;
  };

  std::set<std::string> pairs;
  for (size_t k = 0; k < collision_model_.collisionPairs.size(); ++k) {
    if (geom_data_.collisionResults.at(k).isCollision()) {
      const auto& cp = collision_model_.collisionPairs.at(k);
      pairs.insert(body_name(cp.first) + " <-> " + body_name(cp.second));
    }
  }
  return {pairs.begin(), pairs.end()};
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
      result.normal.setZero();
    } else {
      pinocchio::computeDistance(collision_model_, geom_data_, k);
    }
  }
}

void SceneContext::computeJointJacobians(const Eigen::VectorXd& q) const {
  scene_.computeJointJacobians(data_, q);
}

void SceneContext::computeDistanceJacobian(std::size_t pair_index,
                                           Eigen::Ref<Eigen::RowVectorXd> jacobian) const {
  checkGeometryCurrent("computeDistanceJacobian");
  const auto& pairs = collision_model_.collisionPairs;
  if (pair_index >= pairs.size()) {
    throw std::invalid_argument("SceneContext::computeDistanceJacobian: pair index " +
                                std::to_string(pair_index) + " is out of range (" +
                                std::to_string(pairs.size()) + " collision pairs).");
  }
  if (jacobian.size() != model_.nv) {
    throw std::invalid_argument("SceneContext::computeDistanceJacobian: expected a row of " +
                                std::to_string(model_.nv) + " entries (model.nv), got " +
                                std::to_string(jacobian.size()) + ".");
  }
  jacobian.setZero();
  if (!geom_data_.activeCollisionPairs[pair_index]) {
    return;
  }

  const auto& pair = pairs[pair_index];
  const auto& geom_obj_1 = collision_model_.geometryObjects[pair.first];
  const auto& geom_obj_2 = collision_model_.geometryObjects[pair.second];
  const auto& result = geom_data_.distanceResults[pair_index];
  const Eigen::Vector3d& witness_point_1 = result.nearest_points[0];
  const Eigen::Vector3d& witness_point_2 = result.nearest_points[1];

  // u: unit direction along which moving witness point 1 (relative to point 2) increases the
  // signed distance, i.e. minus coal's o1 -> o2 normal. Coal's normal stays valid under
  // penetration and when the witness points coincide, where the witness difference flips sign or
  // is undefined. Mesh-mesh distance leaves the normal unset, and culled pairs have it zeroed, so
  // those fall back to the witness difference, oriented by the sign of the distance.
  Eigen::Vector3d u;
  const bool mesh_mesh = geom_obj_1.geometry->getObjectType() == coal::OT_BVH &&
                         geom_obj_2.geometry->getObjectType() == coal::OT_BVH;
  if (!mesh_mesh && result.normal.allFinite() && std::abs(result.normal.norm() - 1.0) < 1.0e-6) {
    u = -result.normal;
  } else {
    u = witness_point_1 - witness_point_2;
    const double norm = u.norm();
    if (!(norm >= std::numeric_limits<double>::epsilon())) {
      return;  // Coincident (or culled, or non-finite) witness points: the gradient is undefined.
    }
    u /= norm;
    if (result.min_distance < 0.0) {
      u = -u;
    }
  }

  // r_k: lever arm from joint k's origin to its witness point (world frame), coupling the joint's
  // angular velocity into the witness point's linear velocity via v = v_joint + w x r.
  const auto joint_1 = geom_obj_1.parentJoint;
  const auto joint_2 = geom_obj_2.parentJoint;
  const Eigen::Vector3d r1 = witness_point_1 - data_.oMi[joint_1].translation();
  const Eigen::Vector3d r2 = witness_point_2 - data_.oMi[joint_2].translation();

  joint_jacobian_1_.setZero();
  joint_jacobian_2_.setZero();
  pinocchio::getJointJacobian(model_, data_, joint_1,
                              pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint_jacobian_1_);
  pinocchio::getJointJacobian(model_, data_, joint_2,
                              pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint_jacobian_2_);

  // J = u^T J^1_p + (r_1 x u)^T J^1_w - u^T J^2_p - (r_2 x u)^T J^2_w
  jacobian.noalias() = u.transpose() * joint_jacobian_1_.topRows<3>();
  jacobian.noalias() += r1.cross(u).transpose() * joint_jacobian_1_.bottomRows<3>();
  jacobian.noalias() -= u.transpose() * joint_jacobian_2_.topRows<3>();
  jacobian.noalias() -= r2.cross(u).transpose() * joint_jacobian_2_.bottomRows<3>();

  // Coal can produce non-finite witness points or normals in pathological cases.
  jacobian = jacobian.unaryExpr([](double x) { return std::isfinite(x) ? x : 0.0; });
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
  return randomCollisionFreePositions(scene_.getJointNamesWithMimics(),
                                      Eigen::VectorXd::Zero(model_.nq), max_samples);
}

std::optional<Eigen::VectorXd>
SceneContext::randomCollisionFreePositions(const std::vector<std::string>& joint_names,
                                           const Eigen::VectorXd& q_reference, size_t max_samples) {
  if (q_reference.size() != model_.nq) {
    throw std::invalid_argument(
        "SceneContext::randomCollisionFreePositions: expected " + std::to_string(model_.nq) +
        " configuration values (model.nq), got " + std::to_string(q_reference.size()) + ".");
  }
  Eigen::VectorXd positions = q_reference;
  for (size_t idx = 0; idx < max_samples; ++idx) {
    scene_.randomizeJointPositions(rng_gen_, joint_names, positions);
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
