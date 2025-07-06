#include <chrono>
#include <stdexcept>

#include <roboplan_rrt/rrt.hpp>

namespace roboplan {

RRT::RRT(const std::shared_ptr<Scene> scene, const RRTOptions& options)
    : scene_{scene}, options_{options} {

  // Get the state space info and set bounds from the robot's joints.
  // TODO: Support other joint types besides prismatic and revolute.
  size_t concurrent_one_dof_joints = 0;
  Eigen::VectorXd lower_bounds = Eigen::VectorXd::Zero(scene_->getModel().nq);
  Eigen::VectorXd upper_bounds = Eigen::VectorXd::Zero(scene_->getModel().nq);
  for (const auto& joint_name : scene_->getJointNames()) {
    const auto& joint_info = scene_->getJointInfo(joint_name);
    switch (joint_info.type) {
    case JointType::FLOATING:
    case JointType::PLANAR:
      throw std::runtime_error("Multi-DOF joints not yet supported by RRT.");
    case JointType::CONTINUOUS:
      throw std::runtime_error("Continuous joints not yet supported by RRT.");
    default:  // Prismatic or revolute, which are single-DOF.
      lower_bounds(concurrent_one_dof_joints) = joint_info.limits.min_position[0];
      upper_bounds(concurrent_one_dof_joints) = joint_info.limits.max_position[0];
      ++concurrent_one_dof_joints;
    }
  }
  state_space_ = CombinedStateSpace({"Rn:" + std::to_string(concurrent_one_dof_joints)});
  state_space_.set_bounds(lower_bounds, upper_bounds);
};

std::optional<JointPath> RRT::plan(const JointConfiguration& start,
                                   const JointConfiguration& goal) {
  std::cout << "Planning...\n";

  const auto& q_start = start.positions;
  const auto& q_goal = goal.positions;

  // Ensure the start and goal poses are valid
  if (!scene_->isValidPose(q_start) || !scene_->isValidPose(q_goal)) {
    std::cout << "Invalid poses requested, cannot plan!\n";
    return std::nullopt;
  }

  // Check whether direct connection between the start and goal are possible.
  if ((scene_->configurationDistance(q_start, q_goal) <= options_.max_connection_distance) &&
      (!scene_->hasCollisionsAlongPath(q_start, q_goal, options_.collision_check_step_size))) {
    std::cout << "Can directly connect start and goal!\n";
    return JointPath{.joint_names = scene_->getJointNames(), .positions = {q_start, q_goal}};
  }

  // Initialize the trees for searching.
  kd_tree_.init_tree(state_space_.get_runtime_dim(), state_space_);
  kd_tree_.addPoint(q_start, 0);

  nodes_.clear();
  nodes_.reserve(options_.max_nodes);
  nodes_.emplace_back(q_start, -1);

  // Record the start for measuring timeouts
  const auto start_time = std::chrono::steady_clock::now();

  while (true) {
    // Check for timeout
    auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    if (options_.max_planning_time > 0 && options_.max_planning_time <= elapsed) {
      std::cout << "RRT timed out after " << options_.max_planning_time << " seconds.\n";
      break;
    }

    // Check loop termination criteria.
    if (nodes_.size() >= options_.max_nodes) {
      std::cout << "Added maximum number of nodes (" << options_.max_nodes << ").\n";
      break;
    }

    // Sample the next node with goal biasing.
    const auto q_sample = (uniform_dist_(rng_gen_) <= options_.goal_biasing_probability)
                              ? q_goal
                              : scene_->randomPositions();

    // Attempt to add the node to the tree, if a node is not added resample and try again.
    const auto q_extend_maybe = grow_tree(kd_tree_, nodes_, q_sample);
    if (!q_extend_maybe.has_value()) {
      continue;
    }
    const auto q_extend = q_extend_maybe.value();

    // Check if we can connect directly to the goal node.
    if ((scene_->configurationDistance(q_extend, q_goal) <= options_.max_connection_distance) &&
        (!scene_->hasCollisionsAlongPath(q_extend, q_goal, options_.collision_check_step_size))) {
      std::cout << "  Found goal with " << nodes_.size() << " sampled nodes!\n";
      nodes_.emplace_back(q_goal, nodes_.size() - 1);

      // TODO: Factor out backing out of path
      JointPath path;
      path.joint_names = scene_->getJointNames();
      auto cur_node = nodes_.back();
      path.positions.push_back(cur_node.config);
      auto cur_idx = static_cast<int>(nodes_.size()) - 1;
      while (true) {
        cur_idx = cur_node.parent_id;
        if (cur_idx < 0) {
          break;
        }
        cur_node = nodes_.at(cur_idx);
        path.positions.push_back(cur_node.config);
      }
      std::reverse(path.positions.begin(), path.positions.end());
      return path;
    }
  }

  std::cout << "Unable to find a plan!\n";
  return std::nullopt;
}

std::optional<Eigen::VectorXd> RRT::grow_tree(KdTree& kd_tree, std::vector<Node>& nodes,
                                              const Eigen::VectorXd& q_sample) {

  std::optional<Eigen::VectorXd> q_return = std::nullopt;

  // Extend from the nearest neighbor to max connection distance.
  // TODO: In the case of RRT-Connect, we will keep extending until we cannot any longer.
  // The collision checking will likely need to be inside this function in that case.
  const auto nn = kd_tree.search(q_sample);
  const auto& q_nearest = nodes.at(nn.id).config;

  int parent_id = nn.id;  // Track the correct parent ID
  auto q_current = q_nearest;

  while (true) {
    // Extend towards the sampled node
    auto q_extend = extend(q_current, q_sample, options_.max_connection_distance);

    // If the extended node cannot be connected to the tree then throw it away and return
    if (scene_->hasCollisionsAlongPath(q_current, q_extend, options_.collision_check_step_size)) {
      break;
    }

    q_return = q_extend;
    auto new_id = nodes.size();
    kd_tree.addPoint(q_extend, new_id);
    nodes.emplace_back(q_extend, parent_id);

    // Only one iteration if we are not using RRT-Connect.
    if (!options_.rrt_connect) {
      break;
    }

    // If we have reached the end point we're done.
    if (q_extend == q_sample) {
      break;
    }

    // Otherwise update the parent and continue extendeing.
    parent_id = new_id;
    q_current = q_extend;
  }

  return q_return;
}

Eigen::VectorXd RRT::extend(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal,
                            double max_connection_dist) {
  const auto distance = scene_->configurationDistance(q_start, q_goal);
  if (distance <= max_connection_dist) {
    return q_goal;
  }
  return pinocchio::interpolate(scene_->getModel(), q_start, q_goal,
                                max_connection_dist / distance);
}

void RRT::setRngSeed(unsigned int seed) {
  rng_gen_ = std::mt19937(seed);
  scene_->setRngSeed(seed);
}

}  // namespace roboplan
