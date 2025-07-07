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
  // TODO: We will need two trees, one from start and one from goal.
  KdTree start_tree, goal_tree;
  std::vector<Node> start_nodes, goal_nodes;
  initialize_tree(start_tree, start_nodes, q_start);
  initialize_tree(goal_tree, goal_nodes, q_goal);

  // Record the start for measuring timeouts.
  const auto start_time = std::chrono::steady_clock::now();

  while (true) {
    // Check for timeout.
    auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    if (options_.max_planning_time > 0 && options_.max_planning_time <= elapsed) {
      std::cout << "RRT timed out after " << options_.max_planning_time << " seconds.\n";
      break;
    }

    // Check loop termination criteria.
    if (start_nodes.size() >= options_.max_nodes) {
      std::cout << "Added maximum number of nodes (" << options_.max_nodes << ").\n";
      break;
    }

    // Check if the latest node can be connected directly to the goal.
    auto q_latest = start_nodes.back().config;
    if ((scene_->configurationDistance(q_latest, q_goal) <= options_.max_connection_distance) &&
        (!scene_->hasCollisionsAlongPath(q_latest, q_goal, options_.collision_check_step_size))) {

      // Always add the goal to the end of the nodes list
      start_nodes.emplace_back(q_goal, start_nodes.size() - 1);
      std::cout << "  Found goal with " << start_nodes.size() << " sampled nodes!\n";
      return get_path(start_nodes, start_nodes.size() - 1);
    }

    // Sample the next node with goal biasing.
    const auto q_sample = (uniform_dist_(rng_gen_) <= options_.goal_biasing_probability)
                              ? q_goal
                              : scene_->randomPositions();

    // Attempt to grow the tree towards the sampled node, if no nodes are added resample and try
    // again.
    if (!grow_tree(start_tree, start_nodes, q_sample)) {
      continue;
    }

    // If we have reached the goal then we are done.
    if (start_nodes.back().config == q_goal) {
      std::cout << "  Found goal with " << start_nodes.size() << " sampled nodes!\n";
      return get_path(start_nodes, start_nodes.size() - 1);
    }
  }

  std::cout << "Unable to find a plan!\n";
  return std::nullopt;
}

void RRT::initialize_tree(KdTree& tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_start) {
  tree = KdTree{};  // Resets the reference.
  tree.init_tree(state_space_.get_runtime_dim(), state_space_);
  tree.addPoint(q_start, 0);

  nodes.clear();
  nodes.reserve(options_.max_nodes);
  nodes.emplace_back(q_start, -1);
}

bool RRT::grow_tree(KdTree& kd_tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_sample) {
  bool grew_tree = false;

  // Extend from the nearest neighbor to max connection distance.
  const auto nn = kd_tree.search(q_sample);
  const auto& q_nearest = nodes.at(nn.id).config;

  int parent_id = nn.id;
  auto q_current = q_nearest;

  while (true) {
    // Extend towards the sampled node
    auto q_extend = extend(q_current, q_sample, options_.max_connection_distance);

    // If the extended node cannot be connected to the tree then throw it away and return
    if (scene_->hasCollisionsAlongPath(q_current, q_extend, options_.collision_check_step_size)) {
      break;
    }

    grew_tree = true;
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

  return grew_tree;
}

JointPath RRT::get_path(std::vector<Node>& nodes, int end_idx) {
  JointPath path;
  path.joint_names = scene_->getJointNames();
  auto cur_node = nodes[end_idx];
  path.positions.push_back(cur_node.config);
  auto cur_idx = end_idx;
  while (true) {
    cur_idx = cur_node.parent_id;
    if (cur_idx < 0) {
      break;
    }
    cur_node = nodes.at(cur_idx);
    path.positions.push_back(cur_node.config);
  }
  std::reverse(path.positions.begin(), path.positions.end());
  return path;
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
