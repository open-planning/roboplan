#include <exception>

#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/informed/RejectionInfSampler.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/EITstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/AORRTC.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

#include <roboplan/core/path_utils.hpp>
#include <roboplan_ompl/ompl_planner.hpp>

namespace roboplan {

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {

template <typename Planner> ob::PlannerPtr allocate(const ob::SpaceInformationPtr& si) {
  return std::make_shared<Planner>(si);
}

// Projection-based planners (KPIECE, SBL, STRIDE, ...) are left out: the compound state space has
// no default projection.
const std::map<std::string, ob::PlannerPtr (*)(const ob::SpaceInformationPtr&)> kPlanners = {
    {"RRT", &allocate<og::RRT>},
    {"RRTConnect", &allocate<og::RRTConnect>},
    {"LazyRRT", &allocate<og::LazyRRT>},
    {"RRTstar", &allocate<og::RRTstar>},
    {"InformedRRTstar", &allocate<og::InformedRRTstar>},
    {"RRTsharp", &allocate<og::RRTsharp>},
    {"RRTXstatic", &allocate<og::RRTXstatic>},
    {"LBTRRT", &allocate<og::LBTRRT>},
    {"BiTRRT", &allocate<og::BiTRRT>},
    {"AORRTC", &allocate<og::AORRTC>},
    {"BITstar", &allocate<og::BITstar>},
    {"ABITstar", &allocate<og::ABITstar>},
    {"AITstar", &allocate<og::AITstar>},
    {"EITstar", &allocate<og::EITstar>},
    {"FMT", &allocate<og::FMT>},
    {"BFMT", &allocate<og::BFMT>},
    {"PRM", &allocate<og::PRM>},
    {"PRMstar", &allocate<og::PRMstar>},
    {"LazyPRM", &allocate<og::LazyPRM>},
    {"LazyPRMstar", &allocate<og::LazyPRMstar>},
    {"BiEST", &allocate<og::BiEST>},
};

/// @brief Path length objective whose informed sampler works on any state space.
/// @details OMPL's direct informed sampler only handles real vector (and SE(2)/SE(3)) spaces and
/// throws otherwise, which would break AORRTC, BIT*, and Informed RRT* for groups with continuous
/// or planar joints. Those fall back to rejection sampling, which is correct but slower.
class PathLengthObjective : public ob::PathLengthOptimizationObjective {
public:
  using ob::PathLengthOptimizationObjective::PathLengthOptimizationObjective;

  ob::InformedSamplerPtr allocInformedStateSampler(const ob::ProblemDefinitionPtr& pdef,
                                                   unsigned int max_calls) const override {
    try {
      return ob::PathLengthOptimizationObjective::allocInformedStateSampler(pdef, max_calls);
    } catch (const ompl::Exception&) {
      return std::make_shared<ob::RejectionInfSampler>(pdef, max_calls);
    }
  }
};

}  // namespace

ob::PlannerPtr makePlanner(const std::string& name, const ob::SpaceInformationPtr& si) {
  const auto it = kPlanners.find(name);
  return it == kPlanners.end() ? nullptr : it->second(si);
}

std::vector<std::string> OMPLPlanner::plannerNames() {
  std::vector<std::string> names;
  for (const auto& [name, allocator] : kPlanners) {
    names.push_back(name);
  }
  return names;
}

OMPLPlanner::OMPLPlanner(std::shared_ptr<Scene> scene, const OMPLPlannerOptions& options)
    : scene_{std::move(scene)}, options_{options},
      space_{std::make_shared<SceneStateSpace>(*scene_, options.group_name)} {
  // OMPL logs every setup and solve at INFO level, which is global state; keep it quiet.
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
}

void OMPLPlanner::setOptions(const OMPLPlannerOptions& options) {
  if (options.group_name != options_.group_name) {
    space_ = std::make_shared<SceneStateSpace>(*scene_, options.group_name);
  }
  options_ = options;
}

void OMPLPlanner::setRngSeed(unsigned int seed) { ompl::RNG::setSeed(seed); }

tl::expected<JointPath, std::string> OMPLPlanner::plan(const JointConfiguration& start,
                                                       const JointConfiguration& goal) {
  stats_ = {};
  nodes_.clear();
  edges_.clear();

  // Snapshot the scene into this plan's private context, as the RRT planner does.
  auto context = std::make_shared<SceneContext>(*scene_);
  const auto q_start = context->toFullJointPositions(options_.group_name, start.positions);
  const auto q_goal = context->toFullJointPositions(options_.group_name, goal.positions);
  if (!scene_->isValidConfiguration(q_start)) {
    return tl::make_unexpected("Invalid start configuration requested, cannot plan!");
  }
  if (!scene_->isValidConfiguration(q_goal)) {
    return tl::make_unexpected("Invalid goal configuration requested, cannot plan!");
  }
  if (context->hasCollisions(q_start)) {
    return tl::make_unexpected("Start configuration is in collision, cannot plan!");
  }
  if (context->hasCollisions(q_goal)) {
    return tl::make_unexpected("Goal configuration is in collision, cannot plan!");
  }

  JointPath path;
  path.joint_names = space_->getJointGroupInfo().joint_names;

  // A collision-free straight line is the optimal path, so skip the planner, as the RRT planner
  // does. This also sidesteps a bug in OMPL 2.0.2's AORRTC, which returns a goal-to-goal path when
  // its first attempt connects the goal straight to the start.
  if (!hasCollisionsAlongPath(*scene_, *context, q_start, q_goal,
                              options_.collision_check_step_size,
                              options_.collision_check_use_bisection,
                              /*check_endpoints=*/false)) {
    path.positions = {start.positions, goal.positions};
    return path;
  }

  const auto si = makeSpaceInformation(space_, context, options_.collision_check_step_size,
                                       options_.collision_check_use_bisection);
  auto planner = makePlanner(options_.planner_name, si);
  if (!planner) {
    return tl::make_unexpected("Unknown OMPL planner '" + options_.planner_name + "'.");
  }
  og::SimpleSetup setup(si);
  setup.setPlanner(planner);
  // Optimizing planners minimize path length and, with no cost threshold, use the whole budget.
  setup.setOptimizationObjective(std::make_shared<PathLengthObjective>(si));
  ob::ScopedState<> start_state(space_);
  ob::ScopedState<> goal_state(space_);
  space_->fromGroupPositions(start.positions, start_state.get());
  space_->fromGroupPositions(goal.positions, goal_state.get());
  setup.setStartAndGoalStates(start_state, goal_state);
  // Set up before touching parameters: some OMPL planners (AORRTC) crash in their destructor if
  // they are discarded before setup().
  setup.setup();
  for (const auto& [key, value] : options_.planner_params) {
    if (!planner->params().setParam(key, value)) {
      return tl::make_unexpected("Unknown parameter '" + key + "' for OMPL planner '" +
                                 options_.planner_name + "'.");
    }
  }

  try {
    const auto status = setup.solve(options_.max_planning_time);
    stats_.planning_time = setup.getLastPlanComputationTime();
    if (!setup.haveExactSolutionPath()) {
      return tl::make_unexpected("OMPL planner '" + options_.planner_name +
                                 "' failed: " + status.asString());
    }
    if (options_.simplify_path) {
      setup.simplifySolution(options_.max_simplification_time);
      stats_.simplification_time = setup.getLastSimplificationTime();
    }
  } catch (const std::exception& ex) {
    return tl::make_unexpected("OMPL planner '" + options_.planner_name + "' failed: " + ex.what());
  }
  stats_.num_state_checks =
      std::dynamic_pointer_cast<SceneStateValidityChecker>(si->getStateValidityChecker())
          ->getNumChecks();
  stats_.num_motion_checks = si->getMotionValidator()->getCheckedMotionCount();

  ob::PlannerData planner_data(si);
  setup.getPlannerData(planner_data);
  std::vector<unsigned int> neighbors;
  for (unsigned int v = 0; v < planner_data.numVertices(); ++v) {
    const auto& vertex = planner_data.getVertex(v);
    nodes_.emplace_back(space_->toGroupPositions(vertex.getState()), vertex.getTag());
    neighbors.clear();
    planner_data.getEdges(v, neighbors);
    for (const auto n : neighbors) {
      edges_.emplace_back(v, n);
    }
  }

  for (const auto* state : setup.getSolutionPath().getStates()) {
    path.positions.push_back(space_->toGroupPositions(state));
  }
  return path;
}

}  // namespace roboplan
