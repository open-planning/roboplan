#include <stdexcept>
#include <string>

#include <pinocchio/algorithm/geometry.hpp>

#include <roboplan/core/robot_body_filter.hpp>

// Mirror the coal/hpp-fcl include guard used in geometry_wrappers.hpp.
#if defined(__has_include) && __has_include(<coal/fwd.hh>)
#include <coal/collision.h>
#include <coal/shape/geometric_shapes.h>
#else
#include <hpp/fcl/collision.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#endif

namespace roboplan {

RobotBodyFilter::RobotBodyFilter(const std::shared_ptr<Scene>& scene,
                                 const RobotBodyFilterOptions& options)
    : scene_{scene}, options_{options}, data_{scene->getModel()} {
  if (options_.padding < 0.0) {
    throw std::invalid_argument("RobotBodyFilter padding must be non-negative, got " +
                                std::to_string(options_.padding) + ".");
  }

  // Snapshot the robot's own geometries into a private geometry model. The Coal geometry
  // pointers are shared with the scene, but the model structure (and thus the scratch shaped
  // from it) stays valid even if objects are later added to or removed from the scene.
  const auto& collision_model = scene_->getCollisionModel();
  for (const auto robot_geom_id : scene_->getRobotCollisionGeometryIds()) {
    const auto& geom_obj = collision_model.geometryObjects[robot_geom_id];
    // Make sure the local AABB used by both the broadphase cull and the OBB test is available.
    geom_obj.geometry->computeLocalAABB();
    robot_geom_model_.addGeometryObject(geom_obj);
  }

  robot_geom_data_ = pinocchio::GeometryData(robot_geom_model_);
  geom_scratch_.resize(robot_geom_model_.ngeoms);
}

RobotBodyFilter::Mask
RobotBodyFilter::computeMask(const Eigen::VectorXd& q, const Eigen::Ref<const PointMatrix>& points,
                             const std::optional<Eigen::VectorXd>& extra_padding) {
  const auto num_points = points.rows();
  if (extra_padding && extra_padding->size() != num_points) {
    throw std::invalid_argument("RobotBodyFilter extra_padding has size " +
                                std::to_string(extra_padding->size()) + " but there are " +
                                std::to_string(num_points) + " points.");
  }

  // Place the robot geometry at the query configuration (runs forward kinematics).
  pinocchio::updateGeometryPlacements(scene_->getModel(), data_, robot_geom_model_,
                                      robot_geom_data_, q);

  // Per-geometry setup: world placements and padded world-frame AABBs. The AABB of a rotated
  // box with half extents h is |R| * h, expanded here by the scalar padding; any per-point
  // extra padding is applied at test time instead.
  const double padding = options_.padding;
  for (size_t g = 0; g < robot_geom_model_.ngeoms; ++g) {
    auto& scratch = geom_scratch_[g];
    const auto& oMg = robot_geom_data_.oMg[g];
    const auto& geom = *robot_geom_model_.geometryObjects[g].geometry;

    scratch.rotation = oMg.rotation();
    scratch.translation = oMg.translation();
    scratch.local_center = geom.aabb_local.center();
    scratch.local_half_extents = 0.5 * (geom.aabb_local.max_ - geom.aabb_local.min_);

    const Eigen::Vector3d world_center =
        scratch.rotation * scratch.local_center + scratch.translation;
    const Eigen::Vector3d world_half_extents =
        scratch.rotation.cwiseAbs() * scratch.local_half_extents +
        Eigen::Vector3d::Constant(padding);
    scratch.aabb_min = world_center - world_half_extents;
    scratch.aabb_max = world_center + world_half_extents;

    scratch.geometry = &geom;
    scratch.transform = CoalTransform(scratch.rotation, scratch.translation);
  }

  const bool use_narrowphase = (options_.method == RobotBodyFilterMethod::NARROWPHASE);
  const coal::Sphere point_geom(0.0);
  coal::CollisionRequest request;

  Mask mask = Mask::Constant(num_points, false);
  for (Eigen::Index i = 0; i < num_points; ++i) {
    const Eigen::Vector3d point = points.row(i);
    const double extra = extra_padding ? (*extra_padding)(i) : 0.0;

    for (const auto& scratch : geom_scratch_) {
      // Broadphase: skip geometries whose padded world AABB does not contain the point.
      if (((point - scratch.aabb_min).array() < -extra).any() ||
          ((point - scratch.aabb_max).array() > extra).any()) {
        continue;
      }

      bool near_body;
      if (use_narrowphase) {
        // Exact point-vs-geometry query: a zero-radius sphere collides when it is within the
        // security margin of the geometry surface, i.e. within the padding distance.
        request.security_margin = padding + extra;
        coal::CollisionResult result;
        coal::collide(&point_geom, CoalTransform(point), scratch.geometry, scratch.transform,
                      request, result);
        near_body = result.isCollision();
      } else {
        // Conservative point-in-padded-OBB test in the geometry's local frame.
        const Eigen::Vector3d local_point =
            scratch.rotation.transpose() * (point - scratch.translation);
        near_body = ((local_point - scratch.local_center).cwiseAbs().array() <=
                     scratch.local_half_extents.array() + (padding + extra))
                        .all();
      }

      if (near_body) {
        mask(i) = true;
        break;
      }
    }
  }
  return mask;
}

RobotBodyFilter::PointMatrix
RobotBodyFilter::filterPoints(const Eigen::VectorXd& q, const Eigen::Ref<const PointMatrix>& points,
                              const std::optional<Eigen::VectorXd>& extra_padding) {
  const Mask mask = computeMask(q, points, extra_padding);
  PointMatrix kept(points.rows() - mask.count(), 3);
  Eigen::Index kept_idx = 0;
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    if (!mask(i)) {
      kept.row(kept_idx++) = points.row(i);
    }
  }
  return kept;
}

}  // namespace roboplan
