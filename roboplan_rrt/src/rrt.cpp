#include <chrono>
#include <stdexcept>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene_utils.hpp>
#include <roboplan_rrt/rrt.hpp>

namespace roboplan {

RRT::RRT(const std::shared_ptr<Scene> scene, const RRTOptions& options)
    : scene_{scene}, options_{options} {

  // Validate the joint group.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(options.group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize RRT planner: " + maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();

  // Get the state space info and set bounds from the group's joints.
  const auto maybe_collapsed_pos = collapseContinuousJointPositions(
      *scene_, options_.group_name, Eigen::VectorXd::Zero(joint_group_info_.q_indices.size()));
  if (!maybe_collapsed_pos) {
    throw std::runtime_error("Failed to instantiate RRT planner: " + maybe_collapsed_pos.error());
  }

  std::vector<std::string> state_space_names;
  state_space_names.reserve(joint_group_info_.joint_names.size());
  for (const auto& joint_name : joint_group_info_.joint_names) {
    const auto maybe_joint_info = scene_->getJointInfo(joint_name);
    if (!maybe_joint_info) {
      throw std::runtime_error("Failed to instantiate RRT planner: " + maybe_joint_info.error());
    }
    const auto& joint_info = maybe_joint_info.value();
    if (joint_info.mimic_info) {
      // Mimic joints have nq=0; they are not separate entries in the collapsed state space.
      continue;
    }
    switch (joint_info.type) {
    case JointType::FLOATING:
      throw std::runtime_error("Floating joints not yet supported by RRT.");
    case JointType::PLANAR:
      state_space_names.push_back("Rn:2");
      state_space_names.push_back("SO2");
      break;
    case JointType::CONTINUOUS:
      // The solution squashes the continuous position vectors to be used as an SO(2).
      state_space_names.push_back("SO2");
      break;
    default:  // Prismatic or revolute, which are single-DOF.
      state_space_names.push_back("Rn:1");
    }
  }

  const auto maybe_joint_position_limits =
      scene_->getPositionLimitVectors(options_.group_name, /*collapsed*/ true);
  if (!maybe_joint_position_limits) {
    throw std::runtime_error("Failed to instantiate RRT planner: " +
                             maybe_joint_position_limits.error());
  }

  state_space_ = CombinedStateSpace(state_space_names);
  state_space_.set_bounds(maybe_joint_position_limits->first, maybe_joint_position_limits->second);

  if (state_space_.get_runtime_dim() != static_cast<int>(maybe_collapsed_pos->size())) {
    throw std::runtime_error("Failed to instantiate RRT planner: State space dimension (" +
                             std::to_string(state_space_.get_runtime_dim()) +
                             ") does not match collapsed configuration dimension (" +
                             std::to_string(maybe_collapsed_pos->size()) + ") for group '" +
                             options_.group_name + "'.");
  }
};

tl::expected<JointPath, std::string> RRT::plan(const JointConfiguration& start,
                                               const JointConfiguration& goal) {
  // Record the start for measuring timeouts.
  const auto start_time = std::chrono::steady_clock::now();

  const auto& q_indices = joint_group_info_.q_indices;
  auto q_start = scene_->toFullJointPositions(options_.group_name, start.positions);
  auto q_goal = scene_->toFullJointPositions(options_.group_name, goal.positions);
  auto q_sample = q_start;

  // Snapshot the scene's collision geometry into this plan's private context. All collision checks
  // below route through it, so this plan() call never contends on the Scene's shared collision
  // scratch (it is safe to run concurrently with collision queries elsewhere).
  const CollisionContext collision_context(*scene_);

  // Ensure the start and goal configurations are valid and collision-free.
  if (!scene_->isValidConfiguration(q_start)) {
    return tl::make_unexpected("Invalid start configuration requested, cannot plan!");
  }
  if (!scene_->isValidConfiguration(q_goal)) {
    return tl::make_unexpected("Invalid goal configuration requested, cannot plan!");
  }
  if (collision_context.hasCollisions(q_start)) {
    return tl::make_unexpected("Start configuration is in collision, cannot plan!");
  }
  if (collision_context.hasCollisions(q_goal)) {
    return tl::make_unexpected("Goal configuration is in collision, cannot plan!");
  }

  // Check whether direct connection between the start and goal is possible.
  // Both endpoints were validated as collision-free above, so we only check the interior.
  if ((scene_->configurationDistance(q_start, q_goal) <= options_.max_connection_distance) &&
      (!hasCollisionsAlongPath(*scene_, collision_context, q_start, q_goal,
                               options_.collision_check_step_size,
                               options_.collision_check_use_bisection,
                               /*check_endpoints*/ false))) {
    return JointPath{.joint_names = joint_group_info_.joint_names,
                     .positions = {q_start(q_indices), q_goal(q_indices)}};
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

  while (true) {
    // Check for timeout.
    auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    if (options_.max_planning_time > 0 && options_.max_planning_time <= elapsed) {
      return tl::make_unexpected("RRT timed out after " +
                                 std::to_string(options_.max_planning_time) + " seconds.");
    }

    // Check loop termination criteria.
    if (start_nodes_.size() + goal_nodes_.size() >= options_.max_nodes) {
      return tl::make_unexpected("Added maximum number of nodes (" +
                                 std::to_string(options_.max_nodes) + ").");
    }

    // Set grow and target tree for this loop iteration.
    KdTree& tree = grow_start_tree ? start_tree : goal_tree;
    KdTree& target_tree = grow_start_tree ? goal_tree : start_tree;
    std::vector<Node>& nodes = grow_start_tree ? start_nodes_ : goal_nodes_;
    std::vector<Node>& target_nodes = grow_start_tree ? goal_nodes_ : start_nodes_;

    // Sample the next configuration to grow toward.
    // Goal biasing applies only to single-tree RRT, where it pulls the lone (start) tree toward
    // the goal. In RRT-Connect the bidirectional CONNECT step below already pulls each tree toward
    // the other, so we sample uniformly at random and let the trees reach for one another rather
    // than repeatedly aiming at the fixed opposite endpoint.
    if (!options_.rrt_connect && uniform_dist_(rng_gen_) <= options_.goal_biasing_probability) {
      q_sample = q_goal;
    } else {
      // Randomize only the planning group's DOFs in-place; non-group entries keep their values.
      scene_->randomizeJointPositions(joint_group_info_.joint_names, q_sample);
    }

    // Extend the growing tree a single step toward the sample (EXTEND).
    // If nothing was added, resample and try again.
    if (!growTree(tree, nodes, q_sample, collision_context, /*greedy*/ false)) {
      continue;
    }

    // In RRT-Connect, greedily grow the target tree toward the growing tree's new frontier node
    // (the CONNECT step), so the two trees actively reach for each other. The connection itself is
    // verified and turned into a path by joinTrees below.
    if (options_.rrt_connect) {
      growTree(target_tree, target_nodes, nodes.back().config, collision_context, /*greedy*/ true);
    }

    // Check if the trees can be connected from the latest added node. If so we are done.
    auto maybe_path =
        joinTrees(nodes, target_tree, target_nodes, grow_start_tree, collision_context);
    if (maybe_path.has_value()) {
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
  const auto& q_indices = joint_group_info_.q_indices;
  tree.addPoint(collapse(q_init(q_indices)), 0);

  nodes.clear();
  nodes.reserve(max_size);
  nodes.emplace_back(q_init, -1);
}

bool RRT::growTree(KdTree& kd_tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_sample,
                   const CollisionContext& collision_context, bool greedy) {
  bool grew_tree = false;
  const auto& q_indices = joint_group_info_.q_indices;

  // Extend from the nearest neighbor to max connection distance.
  const auto& nn = kd_tree.search(collapse(q_sample(q_indices)));
  const auto& q_nearest = nodes.at(nn.id).config;

  int parent_id = nn.id;
  auto q_current = q_nearest;

  while (true) {
    // Extend towards the sampled node
    auto q_extend = extend(q_current, q_sample, options_.max_connection_distance);

    // If the extended node cannot be connected to the tree then throw it away and return. The new
    // endpoint `q_extend` must be validated; `q_current` is always an existing (known collision-
    // free) tree node, so checking the endpoints only re-checks that known-free configuration.
    if (hasCollisionsAlongPath(*scene_, collision_context, q_current, q_extend,
                               options_.collision_check_step_size,
                               options_.collision_check_use_bisection,
                               /*check_endpoints*/ true)) {
      break;
    }

    grew_tree = true;
    auto new_id = nodes.size();
    kd_tree.addPoint(collapse(q_extend(q_indices)), new_id);
    nodes.emplace_back(q_extend, parent_id);

    // A plain EXTEND adds a single node; only the greedy CONNECT step keeps extending.
    if (!greedy) {
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
                                        const std::vector<Node>& target_nodes, bool grow_start_tree,
                                        const CollisionContext& collision_context) {
  // The most recently added node is the last appended node in the nodes list.
  const auto& last_added_node = nodes.back();
  const auto& q_last_added = last_added_node.config;

  // Find the nearest node in the target tree (search uses collapsed coordinates).
  const auto& q_indices = joint_group_info_.q_indices;
  const auto& nn = target_tree.search(collapse(q_last_added(q_indices)));
  if (nn.id < 0 || static_cast<size_t>(nn.id) >= target_nodes.size()) {
    throw std::runtime_error("K-D tree search returned invalid node id in joinTrees.");
  }
  const auto& nearest_node = target_nodes.at(nn.id);
  const auto& q_nearest = nearest_node.config;

  // If the nearest and latest nodes are equal we only need one of them, so start from the parent.
  const auto& latest_node =
      q_last_added == q_nearest ? nodes.at(last_added_node.parent_id) : last_added_node;
  const auto& q_latest = latest_node.config;

  // If the latest sampled node in one tree can be connected to the nearest node in the target tree,
  // then a path exists and we should return it. Both endpoints are existing tree nodes and are
  // therefore already known collision-free, so we skip re-checking them.
  if ((scene_->configurationDistance(q_latest, q_nearest) <= options_.max_connection_distance) &&
      (!hasCollisionsAlongPath(*scene_, collision_context, q_latest, q_nearest,
                               options_.collision_check_step_size,
                               options_.collision_check_use_bisection,
                               /*check_endpoints*/ false))) {

    // If (grow_start_tree), nodes is start_tree, target_nodes is goal_tree.
    // Otherwise it is reversed.
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
  path.joint_names = joint_group_info_.joint_names;
  const auto& q_indices = joint_group_info_.q_indices;
  auto cur_node = &end_node;
  path.positions.push_back(cur_node->config(q_indices));
  while (true) {
    auto cur_idx = cur_node->parent_id;
    if (cur_idx < 0) {
      break;
    }
    cur_node = &nodes.at(cur_idx);
    path.positions.push_back(cur_node->config(q_indices));
  }
  return path;
}

Eigen::VectorXd RRT::collapse(const Eigen::VectorXd& q_group) const {
  // Fast path: a group with no continuous/planar DOFs collapses to itself, so skip the work in
  // collapseContinuousJointPositions (a group-info map lookup that copies the whole JointGroupInfo,
  // plus a per-joint getJointInfo lookup). This runs on every k-d tree insert and nearest-neighbor
  // query, so it is firmly on the RRT hot path.
  if (!joint_group_info_.has_continuous_dofs) {
    return q_group;
  }

  const auto maybe_collapsed =
      collapseContinuousJointPositions(*scene_, options_.group_name, q_group);
  if (!maybe_collapsed) {
    throw std::runtime_error("Failed to collapse joint positions: " + maybe_collapsed.error());
  }
  return maybe_collapsed.value();
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
