#include <limits>
#include <optional>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_rrt/constraints.hpp>
#include <roboplan_rrt/rrt.hpp>

#include <roboplan_bindings/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void initConstraints(nanobind::module_& m) {
  nanobind::class_<Constraint>(
      m, "Constraint", "Base class for kinematic constraints that planners enforce by projection.")
      .def("dimension", &Constraint::dimension,
           "The number of residual coordinates this constraint contributes.")
      .def("getGroupName", &Constraint::getGroupName,
           "The joint group whose degrees of freedom a projection is allowed to move.")
      .def("computeError", &Constraint::computeError,
           "Computes the constraint residual at a configuration, which is zero where the "
           "constraint is satisfied and otherwise the signed overshoot past the nearest bound.",
           "q"_a)
      .def("satisfies", &Constraint::satisfies,
           "Checks whether a configuration satisfies this constraint.", "q"_a);

  nanobind::class_<PoseConstraint, Constraint>(
      m, "PoseConstraint",
      "Constrains the pose of a robot frame to a box of allowed task space displacements.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const std::string&, const std::string&,
                          const TaskSpaceVector&, const TaskSpaceVector&, const Eigen::Matrix4d&,
                          const std::string&, double, double>(),
           "scene"_a, "group_name"_a, "frame_name"_a,
           "lower_bounds"_a = TaskSpaceVector::Constant(-std::numeric_limits<double>::infinity()),
           "upper_bounds"_a = TaskSpaceVector::Constant(std::numeric_limits<double>::infinity()),
           "tform"_a = Eigen::Matrix4d::Identity(), "reference_frame"_a = "",
           "position_tolerance"_a = 1.0e-3, "orientation_tolerance"_a = 5.0e-3)
      .def("computeDisplacement", &PoseConstraint::computeDisplacement,
           "Computes the constrained frame's [x, y, z, roll, pitch, yaw] displacement from the "
           "region frame.",
           "q"_a)
      .def_prop_ro("frame_name", &PoseConstraint::getFrameName,
                   "The name of the constrained frame.")
      .def_prop_ro("reference_frame", &PoseConstraint::getReferenceFrame,
                   "The name of the frame the region transform is expressed in.")
      .def_prop_ro("tform", &PoseConstraint::getTransform,
                   "The pose of the region frame relative to the reference frame.")
      .def_prop_ro("lower_bounds", &PoseConstraint::getLowerBounds,
                   "The lower bounds on the frame's task space displacement.")
      .def_prop_ro("upper_bounds", &PoseConstraint::getUpperBounds,
                   "The upper bounds on the frame's task space displacement.")
      .def_prop_ro("position_tolerance", &PoseConstraint::getPositionTolerance,
                   "The position residual norm, in meters, below which the constraint is "
                   "satisfied.")
      .def_prop_ro("orientation_tolerance", &PoseConstraint::getOrientationTolerance,
                   "The orientation residual norm, in radians, below which the constraint is "
                   "satisfied.");

  nanobind::class_<ConstraintProjectorOptions>(m, "ConstraintProjectorOptions",
                                               "Options struct for constraint projection.")
      .def(nanobind::init<double, size_t, double, double, double>(), "path_step_size"_a = 0.1,
           "max_iters"_a = 50, "correction_step_size"_a = 1.0, "damping"_a = 1.0e-6,
           "convergence_ratio"_a = 0.1)
      .def_rw("path_step_size", &ConstraintProjectorOptions::path_step_size,
              "The configuration-space step size taken between projections.")
      .def_rw("max_iters", &ConstraintProjectorOptions::max_iters,
              "The maximum number of projection iterations before giving up.")
      .def_rw("correction_step_size", &ConstraintProjectorOptions::correction_step_size,
              "The fraction of each computed correction to apply per projection iteration.")
      .def_rw("damping", &ConstraintProjectorOptions::damping,
              "Damping value for the Jacobian pseudoinverse.")
      .def_rw("convergence_ratio", &ConstraintProjectorOptions::convergence_ratio,
              "The fraction of each constraint's tolerance the projection converges to, leaving "
              "headroom for the interpolation between projected configurations.");

  nanobind::class_<ConstraintProjector>(
      m, "ConstraintProjector",
      "Projects joint configurations onto the intersection of a set of constraints.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const std::string&,
                          const std::vector<std::shared_ptr<Constraint>>&,
                          const ConstraintProjectorOptions&>(),
           "scene"_a, "group_name"_a, "constraints"_a, "options"_a = ConstraintProjectorOptions())
      .def("satisfies", &ConstraintProjector::satisfies,
           "Checks whether a configuration satisfies every constraint.", "q"_a)
      .def(
          "project",
          [](ConstraintProjector& self,
             const Eigen::VectorXd& q) -> std::optional<Eigen::VectorXd> {
            Eigen::VectorXd q_projected = q;
            if (!self.project(q_projected)) {
              return std::nullopt;
            }
            return q_projected;
          },
          "Projects a configuration onto the constraints, returning None if the projection did "
          "not converge.",
          "q"_a)
      .def("satisfiesAlongPath", &ConstraintProjector::satisfiesAlongPath,
           "Checks whether a straight-line configuration-space path stays inside the constraints.",
           "q_start"_a, "q_end"_a, "max_step_size"_a, "check_endpoints"_a = true)
      .def("empty", &ConstraintProjector::empty,
           "Whether the projector holds no constraints, and is therefore a no-op.");
}

void initRrt(nanobind::module_& m) {
  initConstraints(m);

  nanobind::class_<Node>(m, "Node", "Defines a graph node for search-based planners.")
      .def(nanobind::init<const Eigen::VectorXd&, int>(), "config"_a, "parent_id"_a)
      .def_ro("config", &Node::config, "The configuration (e.g., joint positions) of this node.")
      .def_ro("parent_id", &Node::parent_id, "The parent node ID.")
      .def_ro("cost", &Node::cost, "The cost-to-come from the tree root to this node (RRT* only).");

  nanobind::class_<RRTOptions>(m, "RRTOptions", "Options struct for RRT planner.")
      .def(nanobind::init<const std::string&, size_t, double, double, bool, double, double, bool,
                          bool, double, bool, const ConstraintProjectorOptions&>(),
           "group_name"_a = "", "max_nodes"_a = 1000, "max_connection_distance"_a = 3.0,
           "collision_check_step_size"_a = 0.05, "collision_check_use_bisection"_a = true,
           "goal_biasing_probability"_a = 0.15, "max_planning_time"_a = 0.0,
           "rrt_connect"_a = false, "rrt_star"_a = false, "rewire_distance"_a = 5.0,
           "fast_return"_a = true, "constraint_projection"_a = ConstraintProjectorOptions())
      .def_rw("group_name", &RRTOptions::group_name,
              "The joint group name to be used by the planner.")
      .def_rw("max_nodes", &RRTOptions::max_nodes, "The maximum number of nodes to sample.")
      .def_rw("max_connection_distance", &RRTOptions::max_connection_distance,
              "The maximum configuration distance between two nodes.")
      .def_rw("collision_check_step_size", &RRTOptions::collision_check_step_size,
              "The configuration-space step size for collision checking along edges.")
      .def_rw(
          "collision_check_use_bisection", &RRTOptions::collision_check_use_bisection,
          "If true, uses bisection instead of linear search for collision checking along edges.")
      .def_rw("goal_biasing_probability", &RRTOptions::goal_biasing_probability,
              "The probability of sampling the goal node instead of a random node.")
      .def_rw("max_planning_time", &RRTOptions::max_planning_time,
              "The maximum amount of time to allow for planning, in seconds.")
      .def_rw("rrt_connect", &RRTOptions::rrt_connect,
              "If true, use the RRT-Connect algorithm to grow the search trees.")
      .def_rw("rrt_star", &RRTOptions::rrt_star,
              "If true, use the RRT* algorithm to grow asymptotically optimal trees.")
      .def_rw("rewire_distance", &RRTOptions::rewire_distance,
              "The configuration-space radius used to find neighbors for RRT* rewiring.")
      .def_rw("fast_return", &RRTOptions::fast_return,
              "If true, return on the first path found; if false, plan until the budget is "
              "exhausted and return the lowest-cost path.")
      .def_rw("constraint_projection", &RRTOptions::constraint_projection,
              "Options for the projection that pulls sampled configurations onto the constraints.");

  nanobind::class_<RRT>(
      m, "RRT", "Motion planner based on the Rapidly-exploring Random Tree (RRT) algorithm.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const RRTOptions&>(), "scene"_a,
           "options"_a)
      .def("setOptions", &RRT::setOptions, "Sets or updates the options for the RRT planner.",
           "options"_a)
      .def("plan", unwrap_expected(&RRT::plan),
           "Plan a path from start to goal, optionally subject to constraints that every "
           "configuration on the path must satisfy.",
           "start"_a, "goal"_a, "constraints"_a = std::vector<std::shared_ptr<Constraint>>{})
      .def("setRngSeed", &RRT::setRngSeed, "Sets the seed for the random number generator (RNG).",
           "seed"_a)
      .def("getNodes", &RRT::getNodes,
           "Returns the start and goal trees' node vectors, for visualization purposes.");
}

}  // namespace roboplan
