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

bool hasCollisionsAlongPath(const Scene& scene, const Eigen::VectorXd& q_start,
                            const Eigen::VectorXd& q_end, const double max_step_size) {

  const auto distance = scene.configurationDistance(q_start, q_end);

  // Special case for short paths (also handles division by zero in the next case).
  const bool collision_at_endpoints = scene.hasCollisions(q_start) || scene.hasCollisions(q_end);
  if (distance <= max_step_size) {
    return collision_at_endpoints;
  }

  // In the general case, check the first and last points, then all the intermediate ones.
  if (collision_at_endpoints) {
    return true;
  }
  const auto num_steps = static_cast<size_t>(std::ceil(distance / max_step_size)) + 1;
  for (size_t idx = 1; idx <= num_steps - 1; ++idx) {
    const auto fraction = static_cast<double>(idx) / static_cast<double>(num_steps);
    if (scene.hasCollisions(scene.interpolate(q_start, q_end, fraction))) {
      return true;
    }
  }
  return false;
}

PathShortcutter::PathShortcutter(const std::shared_ptr<Scene> scene, const std::string& group_name)
    : scene_{scene} {

  // Validate the joint group.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize path shortcutter: " +
                             maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();

  q_full_ = scene_->getCurrentJointPositions();
}

JointPath PathShortcutter::shortcut(const JointPath& path, double max_step_size,
                                    unsigned int max_iters, int seed) {

  // Make a copy of the provided path's configurations.
  JointPath shortened_path = path;
  auto& path_configs = shortened_path.positions;

  // We sample in the range (0, 1] to prevent modification of the starting configuration.
  std::random_device rd;
  std::mt19937 gen(seed < 0 ? seed : rd());
  std::uniform_real_distribution<double> dis(std::numeric_limits<double>::epsilon(), 1.0);

  q_full_ = scene_->getCurrentJointPositions();
  const auto& q_indices = joint_group_info_.q_indices;
  for (unsigned int i = 0; i < max_iters; ++i) {
    if (path_configs.size() < 3) {
      // The path is at maximum shortcutted-ness
      return shortened_path;
    }

    // Recompute the path scalings every iteration. If we can't compute these we can
    // assume we are done (the path is at maximum shortness).
    const auto path_scalings_maybe = getNormalizedPathScaling(shortened_path);
    if (!path_scalings_maybe.has_value()) {
      return shortened_path;
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
    auto q_start = q_full_;
    q_start(q_indices) = path_configs[idx_low - 1];
    if (scene_->configurationDistance(q_start, q_low) < max_step_size) {
      q_low = q_start;
      idx_low--;  // Remove the existing configuration
    }

    auto q_end = q_full_;
    q_end(q_indices) = path_configs[idx_high];
    if (scene_->configurationDistance(q_high, q_end) < max_step_size) {
      q_high = q_end;
      idx_high++;  // Remove the existing configuration
    }

    // Ensure the new connection is valid. If not, try again.
    if (hasCollisionsAlongPath(*scene_, q_low, q_high, max_step_size)) {
      continue;
    }

    // Erase elements from idx_low to idx_high (exclusive).
    path_configs.erase(path_configs.begin() + idx_low, path_configs.begin() + idx_high);
    path_configs.insert(path_configs.begin() + idx_low, q_high(q_indices));
    path_configs.insert(path_configs.begin() + idx_low, q_low(q_indices));
  }

  return shortened_path;
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
