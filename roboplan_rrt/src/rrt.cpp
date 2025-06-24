#include <roboplan_rrt/rrt.hpp>

namespace roboplan {

RRT::RRT(const Scene& scene, const RRTOptions& options) : scene_{scene}, options_{options} {

  // TODO: Get the state space info and set bounds from the robot's joints.
  state_space_ = CombinedStateSpace({"Rn:6"});
  state_space_.set_bounds(-3.14 * Eigen::VectorXd::Ones(6), 3.14 * Eigen::VectorXd::Ones(6));

  tree_.init_tree(state_space_.get_runtime_dim(), state_space_);
};

void RRT::plan(const JointConfiguration& start, const JointConfiguration& goal) {
  std::cout << "Planning...\n";
  const auto& q_start = start.positions;
  const auto& q_goal = goal.positions;

  int num_nodes = 0;
  tree_.addPoint(q_start, num_nodes);
  std::cout << "Added start node\n";

  if (!scene_.hasCollisionsAlongPath(q_start, q_goal, options_.collision_check_step_size)) {
    std::cout << "Can directly connect start and goal!\n";
    return;
  }

  for (size_t idx = 0; idx < 10000; ++idx) {
    std::cout << "Node " << idx << ":\n";
    const auto q_rand = scene_.randomPositions();
    const auto nn = tree_.search(q_rand);
    std::cout << "  Nearest neighbor: " << nn.id << "\n";

    if (!scene_.hasCollisionsAlongPath(q_start, q_rand, options_.collision_check_step_size)) {
      std::cout << "  Edge valid -- adding node\n";
      num_nodes++;
      tree_.addPoint(q_rand, num_nodes);

      if (!scene_.hasCollisionsAlongPath(q_rand, q_goal, options_.collision_check_step_size)) {
        std::cout << "  Found goal!\n";
        return;
      }
    } else {
      std::cout << "  Edge in collision\n";
    }
    std::cout << "\n";
  }
}

}  // namespace roboplan
