#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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
  state_dim_ = state_space_.get_runtime_dim();

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

  // When fast_return is disabled, planning does not stop at the first solution; it keeps growing
  // (and, for RRT*, rewiring) until the budget runs out and returns the lowest-cost path found so
  // far. When fast_return is enabled these are unused, as the first solution is returned directly.
  std::optional<JointPath> best_path;
  double best_cost = std::numeric_limits<double>::infinity();

  while (true) {
    // Check for timeout.
    auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    if (options_.max_planning_time > 0 && options_.max_planning_time <= elapsed) {
      // Without fast_return, the budget running out is the normal stopping condition: return the
      // best path found so far, if any.
      if (best_path.has_value()) {
        return best_path.value();
      }
      return tl::make_unexpected("RRT timed out after " +
                                 std::to_string(options_.max_planning_time) + " seconds.");
    }

    // Check loop termination criteria.
    if (start_nodes_.size() + goal_nodes_.size() >= options_.max_nodes) {
      if (best_path.has_value()) {
        return best_path.value();
      }
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

    // Check if the trees can be connected from the latest added node.
    auto maybe_path =
        joinTrees(nodes, target_tree, target_nodes, grow_start_tree, collision_context);
    if (maybe_path.has_value()) {
      auto& [path, path_cost] = maybe_path.value();
      // With fast_return, return the first path found. Otherwise keep the cheapest path seen and
      // keep growing (and, for RRT*, rewiring) until the budget is exhausted.
      if (options_.fast_return) {
        return std::move(path);
      }
      if (path_cost < best_cost) {
        best_cost = path_cost;
        best_path = std::move(path);
      }
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
    int new_id;
    if (options_.rrt_star) {
      new_id = rewire(kd_tree, nodes, q_extend, parent_id, collision_context);
    } else {
      new_id = static_cast<int>(nodes.size());
      kd_tree.addPoint(collapse(q_extend(q_indices)), new_id);
      // Track cost-to-come when the planner will run to budget, so the cheapest path can be picked.
      // With fast_return the first path is returned, so the cost is unused and left at zero.
      const double cost =
          options_.fast_return
              ? 0.0
              : nodes.at(parent_id).cost + scene_->configurationDistance(q_current, q_extend);
      nodes.emplace_back(q_extend, parent_id, cost);
    }

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

std::optional<std::pair<JointPath, double>>
RRT::joinTrees(const std::vector<Node>& nodes, const KdTree& target_tree,
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

  // Helper function to build the connected path
  const auto build_path = [&](const Node& from_node, const Node& to_node,
                              double cost) -> std::pair<JointPath, double> {
    JointPath start_path =
        grow_start_tree ? getPath(nodes, from_node) : getPath(target_nodes, to_node);
    JointPath goal_path =
        grow_start_tree ? getPath(target_nodes, to_node) : getPath(nodes, from_node);

    std::reverse(start_path.positions.begin(), start_path.positions.end());
    start_path.positions.insert(start_path.positions.end(), goal_path.positions.begin(),
                                goal_path.positions.end());
    return {std::move(start_path), cost};
  };

  // If the trees meet at the same point, stitch directly through the shared node with no
  // connecting edge. This avoids the need to fall back to a parent that may be farther
  // than max_connection_distance away (which can happen say, after RRT* rewiring).
  if (q_last_added == q_nearest) {
    // Since they are the same, the total cost-to-come of the joint path is just the last added
    // nodes costs. Note that the node costs are only meaningful when the planner is tracking
    // them (RRT*, or any mode with fast_return disabled); callers returning the first path
    // ignore this value.
    const auto path_cost = last_added_node.cost + nearest_node.cost;
    return build_path(last_added_node, nearest_node, path_cost);
  }

  // Otherwise attempt a normal connection edge between the two nearest nodes.
  const auto connection_distance = scene_->configurationDistance(q_last_added, q_nearest);
  if ((connection_distance <= options_.max_connection_distance) &&
      (!hasCollisionsAlongPath(*scene_, collision_context, q_last_added, q_nearest,
                               options_.collision_check_step_size,
                               options_.collision_check_use_bisection,
                               /*check_endpoints*/ false))) {

    // If the nodes are not the same the total cost-to-come of the joined path is the two
    // connected nodes' costs plus the connecting edge length.
    const auto path_cost = last_added_node.cost + nearest_node.cost + connection_distance;
    return build_path(last_added_node, nearest_node, path_cost);
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

int RRT::rewire(KdTree& kd_tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_new,
                int default_parent_id, const CollisionContext& collision_context) {
  const auto& q_indices = joint_group_info_.q_indices;
  const int new_id = static_cast<int>(nodes.size());

  // Gather the new node's neighbors before it is inserted, so it is not its own neighbor.
  const auto near_ids = findNearNodes(kd_tree, nodes, q_new);

  // Choose the parent that yields the lowest cost-to-come, defaulting to the node we extended from.
  // Each candidate edge runs between two known collision-free tree configurations (or the
  // just-validated q_new), so the endpoints themselves are not re-checked.
  int best_parent = default_parent_id;
  double best_cost = nodes.at(default_parent_id).cost +
                     scene_->configurationDistance(nodes.at(default_parent_id).config, q_new);
  for (const int near_id : near_ids) {
    const auto& q_near = nodes.at(near_id).config;
    const double candidate_cost =
        nodes.at(near_id).cost + scene_->configurationDistance(q_near, q_new);
    if (candidate_cost < best_cost &&
        !hasCollisionsAlongPath(*scene_, collision_context, q_near, q_new,
                                options_.collision_check_step_size,
                                options_.collision_check_use_bisection,
                                /*check_endpoints*/ false)) {
      best_cost = candidate_cost;
      best_parent = near_id;
    }
  }

  kd_tree.addPoint(collapse(q_new(q_indices)), new_id);
  nodes.emplace_back(q_new, best_parent, best_cost);
  nodes.at(best_parent).children.push_back(new_id);

  // Rewire: route a neighbor through the new node whenever that lowers its cost-to-come.
  for (const int near_id : near_ids) {
    if (near_id == best_parent) {
      continue;
    }
    Node& near_node = nodes.at(near_id);
    const double rewired_cost = best_cost + scene_->configurationDistance(q_new, near_node.config);
    if (rewired_cost < near_node.cost &&
        !hasCollisionsAlongPath(*scene_, collision_context, q_new, near_node.config,
                                options_.collision_check_step_size,
                                options_.collision_check_use_bisection,
                                /*check_endpoints*/ false)) {
      // Detach from the old parent, attach to the new node, then refresh the subtree's costs.
      auto& old_siblings = nodes.at(near_node.parent_id).children;
      old_siblings.erase(std::remove(old_siblings.begin(), old_siblings.end(), near_id),
                         old_siblings.end());
      near_node.parent_id = new_id;
      near_node.cost = rewired_cost;
      nodes.at(new_id).children.push_back(near_id);
      propagateCost(nodes, near_id);
    }
  }

  return new_id;
}

std::vector<int> RRT::findNearNodes(const KdTree& tree, const std::vector<Node>& nodes,
                                    const Eigen::VectorXd& q) const {
  // The k-d tree metric sums the per-joint distances (an L1-style norm), whereas the planner's
  // costs and `max_connection_distance` use the scene's configuration distance (an L2-style norm).
  // To keep `rewire_distance` in the same (configuration-space) units as the other options, we
  // search a ball large enough to be a superset of the L2 ball of that radius -- since the L1 norm
  // is at most sqrt(dim) times the L2 norm -- and then filter the candidates by the actual
  // configuration distance.
  const auto& q_indices = joint_group_info_.q_indices;
  const double search_radius =
      options_.rewire_distance * std::sqrt(static_cast<double>(state_dim_));
  const auto neighbors = tree.searchBall(collapse(q(q_indices)), search_radius);

  std::vector<int> ids;
  ids.reserve(neighbors.size());
  for (const auto& neighbor : neighbors) {
    if (scene_->configurationDistance(nodes.at(neighbor.id).config, q) <=
        options_.rewire_distance) {
      ids.push_back(neighbor.id);
    }
  }
  return ids;
}

void RRT::propagateCost(std::vector<Node>& nodes, int root_id) {
  // Iterative DFS over the subtree. The root's cost was already updated by the caller; each
  // descendant's cost-to-come is its (now up-to-date) parent's cost plus the edge length to it.
  std::vector<int> stack(nodes.at(root_id).children);
  while (!stack.empty()) {
    const int id = stack.back();
    stack.pop_back();
    Node& node = nodes.at(id);
    const Node& parent = nodes.at(node.parent_id);
    node.cost = parent.cost + scene_->configurationDistance(parent.config, node.config);
    stack.insert(stack.end(), node.children.begin(), node.children.end());
  }
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
