#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ompl/base/Planner.h>
#include <tl/expected.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_ompl/scene_space.hpp>

namespace roboplan {

/// @brief Options struct for the OMPL planner wrapper.
struct OMPLPlannerOptions {
  /// @brief The joint group name to be used by the planner.
  std::string group_name = "";

  /// @brief The OMPL planner to use. See OMPLPlanner::plannerNames() for the supported names.
  std::string planner_name = "RRTConnect";

  /// @brief The planning time budget, in seconds. Anytime planners such as AORRTC use all of it.
  double max_planning_time = 5.0;

  /// @brief The configuration-space step size for collision checking along edges.
  double collision_check_step_size = 0.05;

  /// @brief If true, uses bisection instead of linear search for collision checking along edges.
  bool collision_check_use_bisection = true;

  /// @brief If true, runs OMPL's path simplifier on the solution.
  bool simplify_path = false;

  /// @brief The time budget for path simplification, in seconds. If <= 0, simplifies until the
  /// path stops improving.
  double max_simplification_time = 0.0;

  /// @brief Planner-specific parameters, forwarded by name to the OMPL planner (e.g. "range").
  std::map<std::string, std::string> planner_params;
};

/// @brief Timing and bookkeeping from the most recent plan.
struct OMPLPlanStats {
  /// @brief The time the planner spent searching, in seconds.
  double planning_time = 0.0;

  /// @brief The time spent simplifying the path, in seconds.
  double simplification_time = 0.0;

  /// @brief The number of single-state validity checks.
  size_t num_state_checks = 0;

  /// @brief The number of edge (motion) validity checks.
  size_t num_motion_checks = 0;
};

/// @brief Motion planner that runs OMPL's sampling-based planners against a roboplan Scene.
class OMPLPlanner {
public:
  OMPLPlanner(std::shared_ptr<Scene> scene, const OMPLPlannerOptions& options);

  /// @brief Sets or updates the planner options. Rebuilds the state space if the group changed.
  void setOptions(const OMPLPlannerOptions& options);

  /// @brief Plans a path from start to goal.
  /// @return A joint-space path if planning succeeds, otherwise an error message.
  tl::expected<JointPath, std::string> plan(const JointConfiguration& start,
                                            const JointConfiguration& goal);

  /// @brief Seeds OMPL's random number generation.
  /// @details This is global to all OMPL planners in the process and only takes effect if called
  /// before any of them has run.
  static void setRngSeed(unsigned int seed);

  const OMPLPlanStats& getLastStats() const { return stats_; }

  /// @brief The vertices of the search graph from the last plan, as expanded group positions with
  /// OMPL's vertex tag. RRT-Connect tags its start tree 1 and its goal tree 2.
  const std::vector<std::pair<Eigen::VectorXd, int>>& getNodes() const { return nodes_; }

  /// @brief The edges of the search graph from the last plan, as index pairs into getNodes().
  const std::vector<std::pair<unsigned int, unsigned int>>& getEdges() const { return edges_; }

  /// @brief The OMPL planner names accepted by `OMPLPlannerOptions::planner_name`.
  static std::vector<std::string> plannerNames();

private:
  std::shared_ptr<Scene> scene_;
  OMPLPlannerOptions options_;
  std::shared_ptr<SceneStateSpace> space_;
  OMPLPlanStats stats_;
  std::vector<std::pair<Eigen::VectorXd, int>> nodes_;
  std::vector<std::pair<unsigned int, unsigned int>> edges_;
};

/// @brief Allocates an OMPL geometric planner by name, or nullptr if the name is unknown.
ompl::base::PlannerPtr makePlanner(const std::string& name,
                                   const ompl::base::SpaceInformationPtr& si);

}  // namespace roboplan
