#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <pinocchio/algorithm/geometry.hpp>

#include <roboplan/core/robot_body_filter.hpp>

// Mirror the coal/hpp-fcl include guard used in geometry_wrappers.hpp.
#if defined(__has_include) && __has_include(<coal/fwd.hh>)
#include <coal/collision.h>
#else
#include <hpp/fcl/collision.h>
#endif

namespace roboplan {

namespace {

/// @brief Number of points each worker thread claims at a time from the shared block counter.
constexpr Eigen::Index kBlockSize = 256;

/// @brief Minimum number of points per thread before an extra thread is worth spawning.
constexpr Eigen::Index kMinPointsPerThread = 8192;

}  // namespace

RobotBodyFilter::RobotBodyFilter(const std::shared_ptr<Scene>& scene,
                                 const RobotBodyFilterOptions& options)
    : scene_{scene}, options_{options}, data_{scene->getModel()} {
  if (options_.padding < 0.0) {
    throw std::invalid_argument("RobotBodyFilter padding must be non-negative, got " +
                                std::to_string(options_.padding) + ".");
  }
  max_threads_ = (options_.num_threads == 0) ? std::max(1u, std::thread::hardware_concurrency())
                                             : options_.num_threads;

  // Snapshot the robot's own geometries so later scene edits cannot invalidate the filter. The
  // collision objects compute each geometry's local AABB on construction.
  const auto& collision_model = scene_->getCollisionModel();
  for (const auto robot_geom_id : scene_->getRobotCollisionGeometryIds()) {
    const auto& geom_obj = collision_model.geometryObjects[robot_geom_id];
    robot_geom_model_.addGeometryObject(geom_obj);
    collision_objects_.emplace_back(geom_obj.geometry);
  }
  robot_geom_data_ = pinocchio::GeometryData(robot_geom_model_);
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

  // Place the collision objects at the query configuration and refresh their world-frame AABBs.
  for (size_t g = 0; g < collision_objects_.size(); ++g) {
    const auto& oMg = robot_geom_data_.oMg[g];
    collision_objects_[g].setTransform(oMg.rotation(), oMg.translation());
    collision_objects_[g].computeAABB();
  }

  const bool use_narrowphase = (options_.method == RobotBodyFilterMethod::Narrowphase);
  const coal::Sphere point_geom(0.0);
  Mask mask = Mask::Constant(num_points, false);

  // Classifies the points in [begin, end). The collision objects are read-only for the duration
  // of the query and each point writes only its own mask entry, so blocks can run concurrently.
  const auto classify_block = [&](const Eigen::Index begin, const Eigen::Index end) {
    coal::CollisionRequest request;
    for (Eigen::Index i = begin; i < end; ++i) {
      const Eigen::Vector3d point = points.row(i);
      const double margin = options_.padding + (extra_padding ? (*extra_padding)(i) : 0.0);

      for (const auto& object : collision_objects_) {
        // Broadphase: skip geometries whose world AABB, grown by the margin, misses the point.
        const auto& aabb = object.getAABB();
        if (((point - aabb.min_).array() < -margin).any() ||
            ((point - aabb.max_).array() > margin).any()) {
          continue;
        }

        bool near_body;
        if (use_narrowphase) {
          // Exact point-vs-geometry query: a zero-radius sphere collides when it is within the
          // security margin of the geometry surface.
          request.security_margin = margin;
          coal::CollisionResult result;
          coal::collide(&point_geom, CoalTransform(point), object.collisionGeometry().get(),
                        object.getTransform(), request, result);
          near_body = result.isCollision();
        } else {
          // Conservative test against the local AABB, grown by the margin, in the geometry frame.
          const auto& local_aabb = object.collisionGeometry()->aabb_local;
          const Eigen::Vector3d local_point = object.getTransform().inverseTransform(point);
          near_body = ((local_point - local_aabb.min_).array() >= -margin).all() &&
                      ((local_point - local_aabb.max_).array() <= margin).all();
        }

        if (near_body) {
          mask(i) = true;
          break;
        }
      }
    }
  };

  // Points are handed out in small fixed-size blocks from a shared counter, so threads that land
  // on stretches of cheap (culled) points simply take more blocks than those doing narrowphase
  // work, even when the points on the robot are clustered together in the cloud (as they are in
  // a sensor scan). The thread count is also capped to keep small clouds serial.
  const size_t num_threads =
      std::min<size_t>(max_threads_, std::max<Eigen::Index>(1, num_points / kMinPointsPerThread));
  if (num_threads <= 1) {
    classify_block(0, num_points);
    return mask;
  }

  std::atomic<Eigen::Index> next_begin{0};
  std::exception_ptr worker_error;
  std::mutex error_mutex;
  const auto worker = [&]() {
    try {
      for (auto begin = next_begin.fetch_add(kBlockSize); begin < num_points;
           begin = next_begin.fetch_add(kBlockSize)) {
        classify_block(begin, std::min(begin + kBlockSize, num_points));
      }
    } catch (...) {
      const std::lock_guard<std::mutex> lock(error_mutex);
      if (!worker_error) {
        worker_error = std::current_exception();
      }
      next_begin.store(num_points);  // Drain the remaining blocks so the other workers stop.
    }
  };

  std::vector<std::thread> workers;
  workers.reserve(num_threads - 1);
  for (size_t t = 1; t < num_threads; ++t) {
    workers.emplace_back(worker);
  }
  worker();  // The calling thread takes part as well.
  for (auto& thread : workers) {
    thread.join();
  }
  if (worker_error) {
    std::rethrow_exception(worker_error);
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
