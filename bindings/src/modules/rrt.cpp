#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_rrt/rrt.hpp>

#include <modules/simple_ik.hpp>
#include <utils/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_rrt(nanobind::module_& m) {
  nanobind::class_<Node>(m, "Node", "Defines a graph node for search-based planners.")
      .def(nanobind::init<const Eigen::VectorXd&, int>(), "config"_a, "parent_id"_a)
      .def_ro("config", &Node::config, "The configuration (e.g., joint positions) of this node.")
      .def_ro("parent_id", &Node::parent_id, "The parent node ID.");

  nanobind::class_<RRTOptions>(m, "RRTOptions", "Options struct for RRT planner.")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("group_name", &RRTOptions::group_name,
              "The joint group name to be used by the planner.")
      .def_rw("max_nodes", &RRTOptions::max_nodes, "The maximum number of nodes to sample.")
      .def_rw("max_connection_distance", &RRTOptions::max_connection_distance,
              "The maximum configuration distance between two nodes.")
      .def_rw("collision_check_step_size", &RRTOptions::collision_check_step_size,
              "The configuration-space step size for collision checking along edges.")
      .def_rw("goal_biasing_probability", &RRTOptions::goal_biasing_probability,
              "The probability of sampling the goal node instead of a random node.")
      .def_rw("max_planning_time", &RRTOptions::max_planning_time,
              "The maximum amount of time to allow for planning, in seconds.")
      .def_rw("rrt_connect", &RRTOptions::rrt_connect,
              "If true, use the RRT-Connect algorithm to grow the search trees.");

  nanobind::class_<RRT>(
      m, "RRT", "Motion planner based on the Rapidly-exploring Random Tree (RRT) algorithm.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const RRTOptions&>(), "scene"_a,
           "options"_a)
      .def("plan", unwrap_expected(&RRT::plan), "Plan a path from start to goal.", "start"_a,
           "goal"_a)
      .def("setRngSeed", &RRT::setRngSeed, "Sets the seed for the random number generator (RNG).",
           "seed"_a)
      .def("getNodes", &RRT::getNodes,
           "Returns the start and goal trees' node vectors, for visualization purposes.");
}

}  // namespace roboplan
