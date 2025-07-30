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
  const auto num_steps = static_cast<size_t>(std::ceil(distance / max_step_size)) + 1;
  if (collision_at_endpoints) {
    return true;
  }
  for (size_t idx = 1; idx <= num_steps - 1; ++idx) {
    const auto fraction = static_cast<double>(idx) / static_cast<double>(num_steps);
    if (scene.hasCollisions(scene.interpolate(q_start, q_end, fraction))) {
      return true;
    }
  }
  return false;
}

JointPath shortcutPath(const Scene& scene, const JointPath& path, double max_step_size,
                       unsigned int max_iters, int seed) {

  // Make a copy of the provided path's configurations.
  JointPath shortened_path = path;
  auto& path_configs = shortened_path.positions;

  // We sample in the range (0, 1] to prevent modification of the starting configuration.
  std::random_device rd;
  std::mt19937 gen(seed < 0 ? seed : rd());
  std::uniform_real_distribution<double> dis(std::numeric_limits<double>::epsilon(), 1.0);

  for (unsigned int i = 0; i < max_iters; ++i) {
    // The path is at maximum shortcutted-ness
    if (path_configs.size() < 3) {
      return shortened_path;
    }

    // Recompute the path scalings every iteration. If we can't compute these we can
    // assume we are done (the path is at maximum shortness).
    const auto path_scalings_maybe = getNormalizedPathScaling(scene, shortened_path);
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
    const auto [q_low, idx_low] =
        getConfigurationFromNormalizedPathScaling(scene, shortened_path, path_scalings, low);
    const auto [q_high, idx_high] =
        getConfigurationFromNormalizedPathScaling(scene, shortened_path, path_scalings, high);
    if (idx_low == idx_high) {
      continue;
    }

    // Check if the sampled segment is collision free. If it is, shortcut the path by updating
    // the configs vector in place. We connect the low and high configurations directly, and
    // erase the intermediate nodes (if any).
    if (!hasCollisionsAlongPath(scene, q_low, q_high, max_step_size)) {
      path_configs[idx_low] = q_low;
      path_configs[idx_high] = q_high;

      if (idx_high > idx_low + 1) {
        path_configs.erase(path_configs.begin() + idx_low + 1, path_configs.begin() + idx_high);
      }
    }
  }

  return shortened_path;
}

std::optional<Eigen::VectorXd> getPathLengths(const Scene& scene, const JointPath& path) {
  if (path.positions.size() < 2) {
    return std::nullopt;
  }

  Eigen::VectorXd path_length_list;
  path_length_list.resize(path.positions.size());

  // Iteratively compute path lengths from start to finish
  double path_length = 0.0;
  path_length_list(0) = path_length;
  for (size_t idx = 0; idx < path.positions.size() - 1; ++idx) {
    path_length += scene.configurationDistance(path.positions[idx], path.positions[idx + 1]);
    path_length_list(idx + 1) = path_length;
  }

  return path_length_list;
}

std::optional<Eigen::VectorXd> getNormalizedPathScaling(const Scene& scene, const JointPath& path) {
  auto path_length_list_maybe = getPathLengths(scene, path);
  if (!path_length_list_maybe.has_value()) {
    return std::nullopt;
  }
  auto path_length_list = path_length_list_maybe.value();
  auto path_length = path_length_list(path_length_list.size() - 1);

  // Normalize and return
  if (path_length > 0.0) {
    path_length_list /= path_length;
  }

  return path_length_list;
}

std::pair<Eigen::VectorXd, size_t>
getConfigurationFromNormalizedPathScaling(const Scene& scene, const JointPath& path,
                                          const Eigen::VectorXd& path_scalings, double value) {

  for (long idx = 1; idx < path_scalings.size() - 1; ++idx) {
    // Find the smallest index that is less than the provided value.
    if (value > path_scalings(idx)) {
      continue;
    }

    // Interpolate to the joint configuration
    const double delta_scale =
        (value - path_scalings(idx - 1)) / (path_scalings(idx) - path_scalings(idx - 1));
    const Eigen::VectorXd q_interp =
        scene.interpolate(path.positions[idx - 1], path.positions[idx], delta_scale);

    return {q_interp, idx};
  }

  // If we get here then the index is the end of the list and we should just return the goal pose.
  return {path.positions.back(), path.positions.size() - 1};
}

}  // namespace roboplan
