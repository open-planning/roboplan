#include <algorithm>
#include <queue>
#include <utility>

#include <roboplan/core/collision_context.hpp>
#include <roboplan/core/path_utils.hpp>

namespace roboplan {

std::vector<Eigen::Matrix4d> computeFramePath(const Scene& scene, const Eigen::VectorXd& q_start,
                                              const Eigen::VectorXd& q_end,
                                              const std::string& frame_name,
                                              const double max_step_size) {

  const auto distance = scene.configurationDistance(q_start, q_end);
  const auto num_steps = static_cast<size_t>(std::ceil(distance / max_step_size)) + 1;

  std::vector<Eigen::Matrix4d> frame_path;
  frame_path.reserve(num_steps);

  for (size_t idx = 0; idx <= num_steps; ++idx) {
    const auto fraction = static_cast<double>(idx) / static_cast<double>(num_steps);
    const auto q_interp = scene.interpolate(q_start, q_end, fraction);
    frame_path.push_back(scene.forwardKinematics(q_interp, frame_name));
  }

  return frame_path;
}

std::vector<Eigen::Matrix4d> computeFramePath(const Scene& scene,
                                              const std::vector<Eigen::VectorXd>& q_vec,
                                              const std::string& frame_name) {
  std::vector<Eigen::Matrix4d> frame_path;
  frame_path.reserve(q_vec.size());
  for (const auto& q : q_vec) {
    frame_path.push_back(scene.forwardKinematics(q, frame_name));
  }
  return frame_path;
}

std::vector<Eigen::VectorXd> resampleUniform(const std::vector<Eigen::VectorXd>& positions,
                                             size_t count, const Scene& scene,
                                             const Eigen::VectorXi& q_indices) {
  // Slack guarding the divide-by-span when a segment has effectively zero length.
  constexpr double kEps = 1e-9;

  const size_t n = positions.size();
  if (n <= 2 || count < 2) {
    return positions;
  }

  // Scene::configurationDistance / interpolate operate on full model configurations, while the
  // input stores only the group coordinates. Reuse two full-configuration buffers, writing each
  // group waypoint into the group slice so the manifold-aware operations see the correct tangent
  // space. The non-group joints are arbitrary (they cancel in the distance and pass through
  // interpolation unchanged), so just seed both buffers from the scene's current state.
  Eigen::VectorXd q_lhs = scene.getCurrentJointPositions();
  Eigen::VectorXd q_rhs = q_lhs;

  // Cumulative configuration-space arc length along the dense path.
  std::vector<double> cumulative(n, 0.0);
  q_lhs(q_indices) = positions.front();
  for (size_t i = 1; i < n; ++i) {
    q_rhs(q_indices) = positions.at(i);
    cumulative.at(i) = cumulative.at(i - 1) + scene.configurationDistance(q_lhs, q_rhs);
    q_lhs(q_indices) = positions.at(i);
  }
  const double total_length = cumulative.back();
  if (total_length <= 0.0) {
    return {positions.front(), positions.back()};
  }

  const size_t num_points = std::min(count, n);
  std::vector<Eigen::VectorXd> result;
  result.reserve(num_points);
  size_t segment = 0;
  for (size_t k = 0; k < num_points; ++k) {
    const double target =
        total_length * static_cast<double>(k) / static_cast<double>(num_points - 1);
    while (segment + 1 < n && cumulative.at(segment + 1) < target) {
      ++segment;
    }
    if (segment + 1 >= n) {
      result.push_back(positions.back());
      continue;
    }
    const double span = cumulative.at(segment + 1) - cumulative.at(segment);
    const double fraction = span > kEps ? (target - cumulative.at(segment)) / span : 0.0;
    q_lhs(q_indices) = positions.at(segment);
    q_rhs(q_indices) = positions.at(segment + 1);
    result.push_back(scene.interpolate(q_lhs, q_rhs, fraction)(q_indices).eval());
  }
  // Pin the exact endpoints.
  result.front() = positions.front();
  result.back() = positions.back();
  return result;
}

namespace {

/// @brief Shared traversal for the hasCollisionsAlongPath overloads.
/// @details Visits the same minimal set of configurations along the path and answers each collision
///   check via the `has_collisions` callable, so the public overloads only differ in which scratch
///   (a caller-owned CollisionContext or the Scene's own) backs that check.
template <typename CollisionCheck>
bool hasCollisionsAlongPathImpl(const Scene& scene, const CollisionCheck& has_collisions,
                                const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                                const double max_step_size, const bool bisection,
                                const bool check_endpoints) {
  const auto distance = scene.configurationDistance(q_start, q_end);

  // Optionally check the endpoints. Callers that have already validated both endpoints can set
  // `check_endpoints` to false to skip these (expensive) collision checks entirely.
  const bool collision_at_endpoints =
      check_endpoints && (has_collisions(q_start) || has_collisions(q_end));

  // Special case for short paths (also handles division by zero in the next case).
  if (distance <= max_step_size) {
    return collision_at_endpoints;
  }

  // In the general case, check the first and last points, then all the intermediate ones.
  if (collision_at_endpoints) {
    return true;
  }

  const auto num_steps = static_cast<size_t>(std::ceil(distance / max_step_size));

  if (bisection) {
    // Visit the evenly-spaced interior grid points {1, ..., num_steps - 1} in a coarse-to-fine
    // bisection order by recursively subdividing intervals at their midpoints. This keeps the
    // early-termination benefit of bisection (collisions near the middle of an edge are found
    // first) while checking exactly the same minimal number of points as the linear scan.
    std::queue<std::pair<size_t, size_t>> intervals;
    intervals.emplace(0, num_steps);
    while (!intervals.empty()) {
      const auto [low, high] = intervals.front();
      intervals.pop();
      const size_t mid = low + (high - low) / 2;
      if (mid == low) {
        continue;  // No interior grid point in this interval.
      }
      const auto fraction = static_cast<double>(mid) / static_cast<double>(num_steps);
      if (has_collisions(scene.interpolate(q_start, q_end, fraction))) {
        return true;
      }
      intervals.emplace(low, mid);
      intervals.emplace(mid, high);
    }
    return false;
  }

  for (size_t idx = 1; idx < num_steps; ++idx) {
    const auto fraction = static_cast<double>(idx) / static_cast<double>(num_steps);
    if (has_collisions(scene.interpolate(q_start, q_end, fraction))) {
      return true;
    }
  }
  return false;
}

}  // namespace

bool hasCollisionsAlongPath(const Scene& scene, const CollisionContext& collision_context,
                            const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                            const double max_step_size, const bool bisection,
                            const bool check_endpoints) {
  return hasCollisionsAlongPathImpl(
      scene, [&](const Eigen::VectorXd& q) { return collision_context.hasCollisions(q); }, q_start,
      q_end, max_step_size, bisection, check_endpoints);
}

bool hasCollisionsAlongPath(const Scene& scene, const Eigen::VectorXd& q_start,
                            const Eigen::VectorXd& q_end, const double max_step_size,
                            const bool bisection, const bool check_endpoints) {
  return hasCollisionsAlongPathImpl(
      scene, [&](const Eigen::VectorXd& q) { return scene.hasCollisions(q); }, q_start, q_end,
      max_step_size, bisection, check_endpoints);
}

tl::expected<double, std::string>
computePathLength(const Scene& scene, const std::string& group_name, const JointPath& path) {
  if (path.positions.size() < 2) {
    return tl::make_unexpected("Path must contain 2 or more points!");
  }

  double length = 0.0;
  for (size_t idx = 0; idx + 1 < path.positions.size(); ++idx) {
    const auto q_start = scene.toFullJointPositions(group_name, path.positions[idx]);
    const auto q_end = scene.toFullJointPositions(group_name, path.positions[idx + 1]);
    length += scene.configurationDistance(q_start, q_end);
  }
  return length;
}

PathShortcutter::PathShortcutter(const std::shared_ptr<Scene> scene,
                                 const PathShortcuttingOptions& options)
    : scene_{scene}, options_{options} {

  // Validate the joint group.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(options_.group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize path shortcutter: " +
                             maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();

  q_full_ = scene_->getCurrentJointPositions();
}

JointPath PathShortcutter::shortcut(const JointPath& path) {

  const double max_step_size = options_.max_step_size;
  const unsigned int max_convergence_iters = options_.max_convergence_iters;

  // Make a copy of the provided path's configurations.
  JointPath shortened_path = path;
  auto& path_configs = shortened_path.positions;

  // We sample in the range (0, 1] to prevent modification of the starting configuration.
  std::random_device rd;
  std::mt19937 gen(options_.seed < 0 ? rd() : static_cast<unsigned int>(options_.seed));
  std::uniform_real_distribution<double> dis(std::numeric_limits<double>::epsilon(), 1.0);

  // Snapshot the scene geometry into a private collision context for this shortcutting pass, so all
  // connection checks below use their own scratch instead of the Scene's shared collision data.
  const CollisionContext collision_context(*scene_);

  q_full_ = scene_->getCurrentJointPositions();
  auto q_start = q_full_;
  auto q_end = q_full_;

  // Count of consecutive iterations that failed to apply a shortcut. Once this reaches
  // `max_convergence_iters` the path is considered converged and we stop early.
  unsigned int empty_iters = 0;

  const auto& q_indices = joint_group_info_.q_indices;
  for (unsigned int i = 0; i < options_.max_iters; ++i) {
    if (path_configs.size() < 3) {
      // The path is at maximum shortcutted-ness
      break;
    }

    // Periodically collapse redundant vertices left behind by corner-cutting shortcuts, so the
    // working path stays compact rather than accumulating unhelpful micro-segments.
    if (i > 0 && i % options_.redundant_removal_iters == 0) {
      removeRedundantVertices(path_configs, collision_context);
      if (path_configs.size() < 3) {
        break;
      }
    }

    // Recompute the path scalings every iteration. If we can't compute these we can
    // assume we are done (the path is at maximum shortness).
    const auto path_scalings_maybe = getNormalizedPathScaling(shortened_path);
    if (!path_scalings_maybe.has_value()) {
      break;
    }
    const auto path_scalings = path_scalings_maybe.value();

    // Randomly sample two points along the scaled path.
    double low = dis(gen);
    double high = dis(gen);
    if (low > high) {
      std::swap(low, high);
    }
    auto [q_low, idx_low] =
        getConfigurationFromNormalizedPathScaling(shortened_path, path_scalings, low);
    auto [q_high, idx_high] =
        getConfigurationFromNormalizedPathScaling(shortened_path, path_scalings, high);

    // Samples are on the same segment so shortening would have no effect.
    if (idx_high == idx_low) {
      if (max_convergence_iters > 0 && ++empty_iters >= max_convergence_iters) {
        break;
      }
      continue;
    }

    // We generally want the new path to be:
    //
    // q_start - > q_low -> q_high -> q_end
    //
    // Because q_low and q_high exist on valid segments, we do not need to check the preceding
    // and following connections. We ONLY need to ensure that q_low and q_high are directly
    // connectable!
    //
    // However, if  `q_start` and `q_low` or `q_high` and `q_end` are very close to each other,
    // it doesn't make sense to add new configurations. If this is the case, use the existing
    // configuration as the sample.
    q_start(q_indices) = path_configs[idx_low - 1];
    if (scene_->configurationDistance(q_start, q_low) < max_step_size) {
      q_low = q_start;
      idx_low--;  // Remove the existing configuration
    }

    q_end(q_indices) = path_configs[idx_high];
    if (scene_->configurationDistance(q_high, q_end) < max_step_size) {
      q_high = q_end;
      idx_high++;  // Remove the existing configuration
    }

    // Ensure the new connection is valid. If not, try again.
    if (hasCollisionsAlongPath(*scene_, collision_context, q_low, q_high, max_step_size)) {
      if (max_convergence_iters > 0 && ++empty_iters >= max_convergence_iters) {
        break;
      }
      continue;
    }

    // Erase elements from idx_low to idx_high (exclusive).
    path_configs.erase(path_configs.begin() + idx_low, path_configs.begin() + idx_high);
    path_configs.insert(path_configs.begin() + idx_low, q_high(q_indices).eval());
    path_configs.insert(path_configs.begin() + idx_low, q_low(q_indices).eval());

    // A shortcut was applied, so the path made progress; reset the convergence counter.
    empty_iters = 0;
  }

  // Final cleanup pass to collapse any redundant vertices remaining at convergence.
  removeRedundantVertices(path_configs, collision_context);

  return shortened_path;
}

size_t PathShortcutter::removeRedundantVertices(std::vector<Eigen::VectorXd>& path_configs,
                                                const CollisionContext& collision_context) {
  const auto& q_indices = joint_group_info_.q_indices;
  auto q_prev = q_full_;
  auto q_next = q_full_;

  size_t total_removed = 0;
  bool removed_any = true;
  while (removed_any) {
    removed_any = false;
    // Walk the interior vertices, deleting any whose neighbors connect directly. When a vertex is
    // removed we do not advance, so the new adjacency (i-1, i+1) is re-evaluated immediately.
    size_t i = 1;
    while (i + 1 < path_configs.size()) {
      q_prev(q_indices) = path_configs[i - 1];
      q_next(q_indices) = path_configs[i + 1];
      // The neighbors are existing collision-free path nodes, so skip the endpoint checks.
      if (!hasCollisionsAlongPath(*scene_, collision_context, q_prev, q_next,
                                  options_.max_step_size,
                                  /* bisection */ false, /* check_endpoints */ false)) {
        path_configs.erase(path_configs.begin() + i);
        ++total_removed;
        removed_any = true;
      } else {
        ++i;
      }
    }
  }

  return total_removed;
}

tl::expected<Eigen::VectorXd, std::string> PathShortcutter::getPathLengths(const JointPath& path) {
  if (path.positions.size() < 2) {
    return tl::make_unexpected("Path must contain 2 or more points!");
  }

  Eigen::VectorXd path_length_list;
  path_length_list.resize(path.positions.size());
  auto q_start = q_full_;
  auto q_end = q_full_;

  // Iteratively compute path lengths from start to finish
  double path_length = 0.0;
  path_length_list(0) = path_length;
  for (size_t idx = 0; idx < path.positions.size() - 1; ++idx) {
    q_start(joint_group_info_.q_indices) = path.positions[idx];
    q_end(joint_group_info_.q_indices) = path.positions[idx + 1];
    path_length += scene_->configurationDistance(q_start, q_end);
    path_length_list(idx + 1) = path_length;
  }

  return path_length_list;
}

tl::expected<Eigen::VectorXd, std::string>
PathShortcutter::getNormalizedPathScaling(const JointPath& path) {
  auto path_length_list_maybe = getPathLengths(path);
  if (!path_length_list_maybe.has_value()) {
    return path_length_list_maybe;
  }
  auto path_length_list = path_length_list_maybe.value();
  auto path_length = path_length_list(path_length_list.size() - 1);

  // Normalize and return
  if (path_length > 0.0) {
    path_length_list /= path_length;
  }

  return path_length_list;
}

std::pair<Eigen::VectorXd, size_t> PathShortcutter::getConfigurationFromNormalizedPathScaling(
    const JointPath& path, const Eigen::VectorXd& path_scalings, double value) {
  auto q_start = q_full_;
  auto q_end = q_full_;
  for (long idx = 1; idx < path_scalings.size() - 1; ++idx) {
    // Find the smallest index that is less than the provided value.
    if (value > path_scalings(idx)) {
      continue;
    }

    // Interpolate to the joint configuration
    const double delta_scale =
        (value - path_scalings(idx - 1)) / (path_scalings(idx) - path_scalings(idx - 1));
    q_start(joint_group_info_.q_indices) = path.positions[idx - 1];
    q_end(joint_group_info_.q_indices) = path.positions[idx];
    const Eigen::VectorXd q_interp = scene_->interpolate(q_start, q_end, delta_scale);

    return {q_interp, idx};
  }

  // If we get here then the index is the end of the list and we should just return the goal pose.
  q_end(joint_group_info_.q_indices) = path.positions.back();
  return {q_end, path.positions.size() - 1};
}

}  // namespace roboplan
