#include <roboplan_rrt/rrt.hpp>

namespace roboplan {

RRT::RRT(const std::shared_ptr<Scene> scene, const RRTOptions& options)
    : scene_{scene}, options_{options} {

  // TODO: Get the state space info and set bounds from the robot's joints.
  state_space_ = CombinedStateSpace({"Rn:6"});
  state_space_.set_bounds(-3.14 * Eigen::VectorXd::Ones(6), 3.14 * Eigen::VectorXd::Ones(6));
};

std::optional<JointPath> RRT::plan(const JointConfiguration& start,
                                   const JointConfiguration& goal) {
  std::cout << "Planning...\n";
  tree_.init_tree(state_space_.get_runtime_dim(), state_space_);
  nodes_.clear();
  nodes_.reserve(options_.max_nodes);

  const auto& q_start = start.positions;
  const auto& q_goal = goal.positions;

  tree_.addPoint(q_start, 0);
  nodes_.emplace_back(q_start, -1);

  if ((scene_->configurationDistance(q_start, q_goal) <= options_.max_connection_distance) &&
      (!scene_->hasCollisionsAlongPath(q_start, q_goal, options_.collision_check_step_size))) {
    std::cout << "Can directly connect start and goal!\n";
    return JointPath{.joint_names = {}, .positions = {q_start, q_goal}};
  }

  while (true) {
    // Sample the next node.
    const auto q_sample = (uniform_dist_(rng_gen_) <= options_.goal_biasing_probability)
                              ? q_goal
                              : scene_->randomPositions();
    const auto nn = tree_.search(q_sample);

    // Extend to max connection distance.
    // TODO: In the case of RRT-Connect, we will keep extending until we cannot any longer.
    // The collision checking will likely need to be inside this function in that case.
    const auto q_extend =
        extend(nodes_.at(nn.id).config, q_sample, options_.max_connection_distance);

    // Check that the extended node can be added to the tree, and if so whether it can directly
    // connect to the goal node.
    if (!scene_->hasCollisionsAlongPath(q_start, q_extend, options_.collision_check_step_size)) {
      tree_.addPoint(q_extend, nodes_.size());
      nodes_.emplace_back(q_extend, nn.id);

      if (!scene_->hasCollisionsAlongPath(q_extend, q_goal, options_.collision_check_step_size)) {
        std::cout << "  Found goal with " << nodes_.size() << " sampled nodes!\n";
        nodes_.emplace_back(q_goal, nodes_.size() - 1);

        // TODO: Factor out backing out of path
        JointPath path;
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

    if (nodes_.size() >= options_.max_nodes) {
      std::cout << "Added maximum number of nodes (" << options_.max_nodes << ").\n";
      break;
    }
  }

  std::cout << "Unable to find a plan!\n";
  return std::nullopt;
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
