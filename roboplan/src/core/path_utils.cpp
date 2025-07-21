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
                       unsigned int max_iters) {
  // Make a copy of the provided path.
  JointPath shortened_path = path;

  // Cannot shorten paths if they don't have enough points.
  if (path.positions.size() < 3) {
    return shortened_path;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis(0.0, 1.0);

  for (unsigned int i = 0; i < max_iters; ++i) {

    // The the path is at maximum shortcutted-ness
    if (shortened_path.positions.size() < 3) {
      return shortened_path;
    }

    // Randomly sample two points along the scaled path
    const auto path_scalings = getNormalizedPathScaling(scene, shortened_path);
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

    // Check if the sampled segment is collision free. If it is, shortcut the path.
    if (!hasCollisionsAlongPath(scene, q_low, q_high, max_step_size)) {
      std::vector<Eigen::VectorXd> new_positions;
      new_positions.reserve(idx_low + 2 + (shortened_path.positions.size() - idx_high));

      for (size_t i = 0; i < idx_low; ++i) {
        new_positions.push_back(shortened_path.positions[i]);
      }

      new_positions.push_back(q_low);
      new_positions.push_back(q_high);

      for (size_t i = idx_high; i < shortened_path.positions.size(); ++i) {
        new_positions.push_back(shortened_path.positions[i]);
      }

      shortened_path.positions = std::move(new_positions);
    }
  }

  return shortened_path;
}

std::vector<double> getNormalizedPathScaling(const Scene& scene, const JointPath& path) {
  if (path.positions.empty()) {
    return {};
  }
  if (path.positions.size() == 1) {
    return {1.0};
  }

  std::vector<double> path_length_list;
  path_length_list.reserve(path.positions.size());

  // Iteratively compute path lengths from start to finish
  double path_length = 0.0;
  path_length_list.push_back(path_length);
  for (size_t idx = 0; idx < path.positions.size() - 1; ++idx) {
    path_length += scene.configurationDistance(path.positions[idx], path.positions[idx + 1]);
    path_length_list.push_back(path_length);
  }

  // Normalize
  for (auto& length : path_length_list) {
    length /= path_length;
  }

  return path_length_list;
}

std::pair<Eigen::VectorXd, size_t>
getConfigurationFromNormalizedPathScaling(const Scene& scene, const JointPath& path,
                                          const std::vector<double>& path_scalings, double value) {

  for (size_t idx = 0; idx < path_scalings.size(); ++idx) {
    // Find the smallest index that is less than value
    if (value > path_scalings[idx]) {
      continue;
    }

    // Interpolate to the joint configuration
    const double delta_scale =
        (value - path_scalings[idx - 1]) / (path_scalings[idx] - path_scalings[idx - 1]);
    const Eigen::VectorXd q_interp =
        scene.interpolate(path.positions[idx - 1], path.positions[idx], delta_scale);

    return {q_interp, idx};
  }

  // This shouldn't be possible
  return {path.positions.back(), path.positions.size() - 1};
}

}  // namespace roboplan
