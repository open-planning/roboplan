#include <chrono>
#include <sstream>
#include <stdexcept>

#include <roboplan/core/path_utils.hpp>
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

tl::expected<JointPath, std::string> RRT::plan(const JointConfiguration& start,
                                               const JointConfiguration& goal) {
  std::cout << "Planning...\n";

  const auto& q_start = start.positions;
  const auto& q_goal = goal.positions;

  // Ensure the start and goal poses are valid
  if (!scene_->isValidPose(q_start) || !scene_->isValidPose(q_goal)) {
    const auto msg = "Invalid poses requested, cannot plan!";
    std::cout << msg << "\n";
    return tl::make_unexpected(msg);
  }

  // Check whether direct connection between the start and goal are possible.
  if ((scene_->configurationDistance(q_start, q_goal) <= options_.max_connection_distance) &&
      (!hasCollisionsAlongPath(*scene_, q_start, q_goal, options_.collision_check_step_size))) {
    std::cout << "Can directly connect start and goal!\n";
    return JointPath{.joint_names = scene_->getJointNames(), .positions = {q_start, q_goal}};
  }

  // Initialize the trees for searching.
  // When using RRT-Connect we use two trees, one growing from the start, one growing from the goal.
  KdTree start_tree, goal_tree;
  initializeTree(start_tree, start_nodes_, q_start, options_.max_nodes);

  // The goal tree will only contain the goal pose if not using connect.
  size_t goal_tree_size = options_.rrt_connect ? options_.max_nodes : 1;
  initializeTree(goal_tree, goal_nodes_, q_goal, goal_tree_size);

  // For switching which tree we grow when using RRT-Connect.
  bool grow_start_tree = true;

  // Record the start for measuring timeouts.
  const auto start_time = std::chrono::steady_clock::now();

  while (true) {
    // Check for timeout.
    auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    if (options_.max_planning_time > 0 && options_.max_planning_time <= elapsed) {
      std::stringstream ss;
      ss << "RRT timed out after " << options_.max_planning_time << " seconds.";
      std::cout << ss.str() << "\n";
      return tl::make_unexpected(ss.str());
    }

    // Check loop termination criteria.
    if (start_nodes_.size() + goal_nodes_.size() >= options_.max_nodes) {
      std::stringstream ss;
      ss << "Added maximum number of nodes (" << options_.max_nodes << ").";
      std::cout << ss.str() << "\n";
      return tl::make_unexpected(ss.str());
    }

    // Set grow and target tree for this loop iteration.
    KdTree& tree = grow_start_tree ? start_tree : goal_tree;
    KdTree& target_tree = grow_start_tree ? goal_tree : start_tree;
    std::vector<Node>& nodes = grow_start_tree ? start_nodes_ : goal_nodes_;
    std::vector<Node>& target_nodes = grow_start_tree ? goal_nodes_ : start_nodes_;

    // Sample the next node with goal biasing, using the goal node for the starting tree,
    // the start node for the goal tree.
    const auto& q_target = grow_start_tree ? q_goal : q_start;
    const auto q_sample = (uniform_dist_(rng_gen_) <= options_.goal_biasing_probability)
                              ? q_target
                              : scene_->randomPositions();

    // Attempt to grow the tree towards the sampled node.
    // If no nodes are added, we resample and try again.
    if (!growTree(tree, nodes, q_sample)) {
      continue;
    }

    // Check if the trees can be connected from the latest added node. If so we are done.
    auto maybe_path = joinTrees(nodes, target_tree, target_nodes, grow_start_tree);
    if (maybe_path.has_value()) {
      std::cout << " Found goal with " << start_nodes_.size() + goal_nodes_.size()
                << " sampled nodes!\n";
      return maybe_path.value();
    }

    // Switch the grow and target trees for the next iteration, if required.
    if (options_.rrt_connect) {
      grow_start_tree = !grow_start_tree;
    }
  }

  return tl::make_unexpected("Unable to find a path!");
}

void RRT::initializeTree(KdTree& tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_init,
                         size_t max_size) {
  tree = KdTree{};  // Resets the reference.
  tree.init_tree(state_space_.get_runtime_dim(), state_space_);
  tree.addPoint(q_init, 0);

  nodes.clear();
  nodes.reserve(max_size);
  nodes.emplace_back(q_init, -1);
}

bool RRT::growTree(KdTree& kd_tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_sample) {
  bool grew_tree = false;

  // Extend from the nearest neighbor to max connection distance.
  const auto& nn = kd_tree.search(q_sample);
  const auto& q_nearest = nodes.at(nn.id).config;

  int parent_id = nn.id;
  auto q_current = q_nearest;

  while (true) {
    // Extend towards the sampled node
    auto q_extend = extend(q_current, q_sample, options_.max_connection_distance);

    // If the extended node cannot be connected to the tree then throw it away and return
    if (hasCollisionsAlongPath(*scene_, q_current, q_extend, options_.collision_check_step_size)) {
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

    // Otherwise update the parent and continue extending.
    parent_id = new_id;
    q_current = q_extend;
  }

  return grew_tree;
}

std::optional<JointPath> RRT::joinTrees(const std::vector<Node>& nodes, const KdTree& target_tree,
                                        const std::vector<Node>& target_nodes,
                                        bool grow_start_tree) {
  // The most recently added node is the last appended node in the nodes list.
  const auto& last_added_node = nodes.back();
  const auto& q_last_added = last_added_node.config;

  // Find the nearest node in the target tree.
  const auto& nn = target_tree.search(q_last_added);
  const auto& nearest_node = target_nodes.at(nn.id);
  const auto& q_nearest = nearest_node.config;

  // If the nearest and latest nodes are equal we only need one of them, so start from the parent.
  const auto& latest_node =
      q_last_added == q_nearest ? nodes.at(last_added_node.parent_id) : last_added_node;
  const auto& q_latest = latest_node.config;

  // If the latest sampled node in one tree can be connected to the nearest node in the target tree,
  // then a path exists and we should return it.
  if ((scene_->configurationDistance(q_latest, q_nearest) <= options_.max_connection_distance) &&
      (!hasCollisionsAlongPath(*scene_, q_latest, q_nearest, options_.collision_check_step_size))) {

    // If (grow_start_tree), nodes is start_tree, target_nodes is goal_tree. Otherwise it is
    // reversed.
    JointPath start_path =
        grow_start_tree ? getPath(nodes, latest_node) : getPath(target_nodes, nearest_node);
    JointPath goal_path =
        grow_start_tree ? getPath(target_nodes, nearest_node) : getPath(nodes, latest_node);

    // We always set start_path as connection -> start_node and goal_path is connection ->
    // goal_node.
    std::reverse(start_path.positions.begin(), start_path.positions.end());
    start_path.positions.insert(start_path.positions.end(), goal_path.positions.begin(),
                                goal_path.positions.end());

    return start_path;
  }

  return std::nullopt;
}

JointPath RRT::getPath(const std::vector<Node>& nodes, const Node& end_node) {
  JointPath path;
  path.joint_names = scene_->getJointNames();
  auto cur_node = &end_node;
  path.positions.push_back(cur_node->config);
  while (true) {
    auto cur_idx = cur_node->parent_id;
    if (cur_idx < 0) {
      break;
    }
    cur_node = &nodes.at(cur_idx);
    path.positions.push_back(cur_node->config);
  }
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
