#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_ompl/ompl_planner.hpp>

#include <roboplan_bindings/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void initOmpl(nanobind::module_& m) {
  nanobind::class_<OMPLPlannerOptions>(m, "OMPLPlannerOptions",
                                       "Options struct for the OMPL planner wrapper.")
      .def(nanobind::init<const std::string&, const std::string&, double, double, bool, bool,
                          double, const std::map<std::string, std::string>&>(),
           "group_name"_a = "", "planner_name"_a = "RRTConnect", "max_planning_time"_a = 5.0,
           "collision_check_step_size"_a = 0.05, "collision_check_use_bisection"_a = true,
           "simplify_path"_a = false, "max_simplification_time"_a = 0.0,
           "planner_params"_a = std::map<std::string, std::string>{})
      .def_rw("group_name", &OMPLPlannerOptions::group_name,
              "The joint group name to be used by the planner.")
      .def_rw("planner_name", &OMPLPlannerOptions::planner_name,
              "The OMPL planner to use. See OMPLPlanner.plannerNames() for the supported names.")
      .def_rw("max_planning_time", &OMPLPlannerOptions::max_planning_time,
              "The planning time budget, in seconds. Anytime planners such as AORRTC use all of "
              "it.")
      .def_rw("collision_check_step_size", &OMPLPlannerOptions::collision_check_step_size,
              "The configuration-space step size for collision checking along edges.")
      .def_rw(
          "collision_check_use_bisection", &OMPLPlannerOptions::collision_check_use_bisection,
          "If true, uses bisection instead of linear search for collision checking along edges.")
      .def_rw("simplify_path", &OMPLPlannerOptions::simplify_path,
              "If true, runs OMPL's path simplifier on the solution.")
      .def_rw("max_simplification_time", &OMPLPlannerOptions::max_simplification_time,
              "The time budget for path simplification, in seconds. If <= 0, simplifies until the "
              "path stops improving.")
      .def_rw("planner_params", &OMPLPlannerOptions::planner_params,
              "Planner-specific parameters, forwarded by name to the OMPL planner (e.g. "
              "'range').");

  nanobind::class_<OMPLPlanStats>(m, "OMPLPlanStats",
                                  "Timing and bookkeeping from the most recent plan.")
      .def_ro("planning_time", &OMPLPlanStats::planning_time,
              "The time the planner spent searching, in seconds.")
      .def_ro("simplification_time", &OMPLPlanStats::simplification_time,
              "The time spent simplifying the path, in seconds.")
      .def_ro("num_state_checks", &OMPLPlanStats::num_state_checks,
              "The number of single-state validity checks.")
      .def_ro("num_motion_checks", &OMPLPlanStats::num_motion_checks,
              "The number of edge (motion) validity checks.");

  nanobind::class_<OMPLPlanner>(
      m, "OMPLPlanner",
      "Motion planner that runs OMPL's sampling-based planners against a roboplan Scene.")
      .def(nanobind::init<std::shared_ptr<Scene>, const OMPLPlannerOptions&>(), "scene"_a,
           "options"_a)
      .def("setOptions", &OMPLPlanner::setOptions,
           "Sets or updates the planner options. Rebuilds the state space if the group changed.",
           "options"_a)
      .def("plan", unwrap_expected(&OMPLPlanner::plan),
           nanobind::call_guard<nanobind::gil_scoped_release>(), "Plans a path from start to goal.",
           "start"_a, "goal"_a)
      .def_static("setRngSeed", &OMPLPlanner::setRngSeed,
                  "Seeds OMPL's random number generation. This is global to all OMPL planners in "
                  "the process and only takes effect if called before any of them has run.",
                  "seed"_a)
      .def("getLastStats", &OMPLPlanner::getLastStats,
           "Timing and bookkeeping from the most recent plan.")
      .def("getNodes", &OMPLPlanner::getNodes,
           "The vertices of the search graph from the last plan, as (expanded group positions, "
           "OMPL vertex tag) pairs. RRT-Connect tags its start tree 1 and its goal tree 2.")
      .def("getEdges", &OMPLPlanner::getEdges,
           "The edges of the search graph from the last plan, as index pairs into getNodes().")
      .def_static("plannerNames", &OMPLPlanner::plannerNames,
                  "The OMPL planner names accepted by OMPLPlannerOptions.planner_name.");
}

}  // namespace roboplan
