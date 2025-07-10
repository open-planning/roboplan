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

}  // namespace roboplan
