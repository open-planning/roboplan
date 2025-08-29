#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_rrt/rrt.hpp>

#include <modules/simple_ik.hpp>
#include <utils/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_rrt(nanobind::module_& m) {
  nanobind::class_<Node>(m, "Node")
      .def(nanobind::init<const Eigen::VectorXd&, int>())
      .def_ro("config", &Node::config)
      .def_ro("parent_id", &Node::parent_id);

  nanobind::class_<RRTOptions>(m, "RRTOptions")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("max_nodes", &RRTOptions::max_nodes)
      .def_rw("max_connection_distance", &RRTOptions::max_connection_distance)
      .def_rw("collision_check_step_size", &RRTOptions::collision_check_step_size)
      .def_rw("goal_biasing_probability", &RRTOptions::goal_biasing_probability)
      .def_rw("max_planning_time", &RRTOptions::max_planning_time)
      .def_rw("rrt_connect", &RRTOptions::rrt_connect);

  nanobind::class_<RRT>(m, "RRT")
      .def(nanobind::init<const std::shared_ptr<Scene>, const RRTOptions&>())
      .def("plan", unwrap_expected(&RRT::plan))
      .def("setRngSeed", &RRT::setRngSeed)
      .def("getNodes", &RRT::getNodes);
}

}  // namespace roboplan
