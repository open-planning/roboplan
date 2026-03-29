#include <limits>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_oink/barriers/position_barrier.hpp>
#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/configuration.hpp>
#include <roboplan_oink/tasks/frame.hpp>

#include <modules/optimal_ik.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_optimal_ik(nanobind::module_& m) {

  nanobind::class_<Task>(m, "Task", "Abstract base class for IK tasks.")
      .def_ro("gain", &Task::gain, "Task gain for low-pass filtering.")
      .def_ro("weight", &Task::weight, "Weight matrix for cost normalization.")
      .def_ro("lm_damping", &Task::lm_damping, "Levenberg-Marquardt damping.")
      .def_ro("num_variables", &Task::num_variables, "Number of optimization variables.");

  // Bind FrameTaskOptions configuration struct
  nanobind::class_<FrameTaskOptions>(m, "FrameTaskOptions", "Parameters for FrameTask.")
      .def(nanobind::init<double, double, double, double, double, double>(),
           "position_cost"_a = 1.0, "orientation_cost"_a = 1.0, "task_gain"_a = 1.0,
           "lm_damping"_a = 0.0, "max_position_error"_a = std::numeric_limits<double>::infinity(),
           "max_rotation_error"_a = std::numeric_limits<double>::infinity(),
           "Constructor with custom parameters.")
      .def_rw("position_cost", &FrameTaskOptions::position_cost, "Position cost weight.")
      .def_rw("orientation_cost", &FrameTaskOptions::orientation_cost, "Orientation cost weight.")
      .def_rw("task_gain", &FrameTaskOptions::task_gain, "Task gain for low-pass filtering.")
      .def_rw("lm_damping", &FrameTaskOptions::lm_damping, "Levenberg-Marquardt damping.")
      .def_rw("max_position_error", &FrameTaskOptions::max_position_error,
              "Maximum position error magnitude (meters). Infinite = no limit.")
      .def_rw("max_rotation_error", &FrameTaskOptions::max_rotation_error,
              "Maximum rotation error magnitude (radians). Infinite = no limit.");

  // Bind FrameTask inheriting from Task
  nanobind::class_<FrameTask, Task>(m, "FrameTask",
                                    "Task to reach a target pose for a specified frame.")
      .def(nanobind::init<const CartesianConfiguration&, int, const FrameTaskOptions&>(),
           "target_pose"_a, "num_variables"_a, "options"_a = FrameTaskOptions{})
      .def_ro("frame_name", &FrameTask::frame_name, "Name of the frame to control.")
      .def_ro("frame_id", &FrameTask::frame_id,
              "Index of the frame in the scene's Pinocchio model.")
      .def_ro("target_pose", &FrameTask::target_pose, "Target pose for the frame.")
      .def_ro("max_position_error", &FrameTask::max_position_error,
              "Maximum position error magnitude (meters).")
      .def_ro("max_rotation_error", &FrameTask::max_rotation_error,
              "Maximum rotation error magnitude (radians).")
      .def("setTargetFrameTransform", &FrameTask::setTargetFrameTransform, "tform"_a,
           "Sets the target transform for this frame task.");

  // Bind ConfigurationTaskOptions configuration struct
  nanobind::class_<ConfigurationTaskOptions>(m, "ConfigurationTaskOptions",
                                             "Parameters for ConfigurationTask.")
      .def(nanobind::init<double, double>(), "task_gain"_a = 1.0, "lm_damping"_a = 0.0)
      .def_rw("task_gain", &ConfigurationTaskOptions::task_gain,
              "Task gain for low-pass filtering.")
      .def_rw("lm_damping", &ConfigurationTaskOptions::lm_damping, "Levenberg-Marquardt damping.");

  // Bind ConfigurationTask inheriting from Task
  nanobind::class_<ConfigurationTask, Task>(m, "ConfigurationTask",
                                            "Task to reach a target joint configuration.")
      .def(nanobind::init<const Eigen::VectorXd&, const Eigen::VectorXd&,
                          const ConfigurationTaskOptions&>(),
           "target_q"_a, "joint_weights"_a, "options"_a = ConfigurationTaskOptions{})
      .def_rw("target_q", &ConfigurationTask::target_q, "Target joint configuration.")
      .def_rw("joint_weights", &ConfigurationTask::joint_weights,
              "Weights for each joint in the configuration task.");

  // Bind the abstract Constraints base class
  nanobind::class_<Constraints>(m, "Constraints", "Abstract base class for IK constraints.");

  // Bind PositionLimit constraint
  nanobind::class_<PositionLimit, Constraints>(m, "PositionLimit",
                                               "Constraint to enforce joint position limits.")
      .def(nanobind::init<int, double>(), "num_variables"_a, "gain"_a = 1.0)
      .def_rw("config_limit_gain", &PositionLimit::config_limit_gain,
              "Gain for position limit enforcement.");

  // Bind VelocityLimit constraint
  nanobind::class_<VelocityLimit, Constraints>(m, "VelocityLimit",
                                               "Constraint to enforce joint velocity limits.")
      .def(nanobind::init<int, double, const Eigen::VectorXd&>(), "num_variables"_a, "dt"_a,
           "v_max"_a)
      .def_rw("dt", &VelocityLimit::dt, "Time step for velocity calculation.")
      .def_rw("v_max", &VelocityLimit::v_max, "Maximum joint velocities.");

  // Bind the abstract Barrier base class
  nanobind::class_<Barrier>(m, "Barrier", "Abstract base class for Control Barrier Functions.")
      .def("get_num_barriers", &Barrier::getNumBarriers, "scene"_a,
           "Get the number of barrier constraints.")
      .def_ro("gain", &Barrier::gain, "Barrier gain (gamma).")
      .def_ro("dt", &Barrier::dt, "Timestep.")
      .def_ro("safe_displacement_gain", &Barrier::safe_displacement_gain,
              "Gain for safe displacement regularization.")
      .def_ro("safety_margin", &Barrier::safety_margin,
              "Conservative margin for hard constraints.");

  // Bind ConstraintAxisSelection configuration struct
  nanobind::class_<ConstraintAxisSelection>(m, "ConstraintAxisSelection",
                                            "Axis selection for position barrier constraints.")
      .def(nanobind::init<bool, bool, bool>(), "x"_a = true, "y"_a = true, "z"_a = true,
           "Constructor with axis enable flags.")
      .def_rw("x", &ConstraintAxisSelection::x, "Constrain X axis.")
      .def_rw("y", &ConstraintAxisSelection::y, "Constrain Y axis.")
      .def_rw("z", &ConstraintAxisSelection::z, "Constrain Z axis.");

  // Bind PositionBarrier
  nanobind::class_<PositionBarrier, Barrier>(
      m, "PositionBarrier",
      "Position barrier constraint that keeps a frame within an axis-aligned bounding box.")
      .def(nanobind::init<const std::string&, const Eigen::Vector3d&, const Eigen::Vector3d&, int,
                          double, const ConstraintAxisSelection&, double, double, double>(),
           "frame_name"_a, "p_min"_a, "p_max"_a, "num_variables"_a, "dt"_a,
           "axis_selection"_a = ConstraintAxisSelection(), "gain"_a = 1.0,
           "safe_displacement_gain"_a = 1.0, "safety_margin"_a = 0.0,
           "Create a position barrier with optional axis selection.")
      .def("get_frame_position", &PositionBarrier::getFramePosition, "scene"_a,
           "Get the current frame position in world coordinates.")
      .def_ro("frame_name", &PositionBarrier::frame_name, "Name of the constrained frame.")
      .def_ro("axis_selection", &PositionBarrier::axis_selection, "Axis selection for constraints.")
      .def_ro("p_min", &PositionBarrier::p_min, "Minimum position bounds.")
      .def_ro("p_max", &PositionBarrier::p_max, "Maximum position bounds.");

  // Bind Oink solver
  nanobind::class_<Oink>(m, "Oink", "Optimal Inverse Kinematics solver.")
      .def(nanobind::init<int>(), "num_variables"_a,
           "Constructor with number of optimization variables.")
      .def_ro("num_variables", &Oink::num_variables, "Number of optimization variables.")
      .def(
          "solveIk",
          [](Oink& self, const std::vector<std::shared_ptr<Task>>& tasks,
             const std::vector<std::shared_ptr<Constraints>>& constraints,
             const std::vector<std::shared_ptr<Barrier>>& barriers,
             const std::shared_ptr<Scene>& scene, nanobind::DRef<Eigen::VectorXd> delta_q,
             double regularization) {
            auto result =
                self.solveIk(tasks, constraints, barriers, *scene, delta_q, regularization);
            if (!result.has_value()) {
              throw std::runtime_error("IK solve failed: " + result.error());
            }
          },
          "tasks"_a, "constraints"_a, "barriers"_a, "scene"_a, "delta_q"_a,
          "regularization"_a = 1e-12,
          "Solve inverse kinematics for given tasks, constraints, and optional barriers.\n\n"
          "Solves a QP optimization problem to compute the joint velocity that minimizes\n"
          "weighted task errors while satisfying all constraints and barrier functions.\n"
          "The result is written directly into the provided delta_q buffer.\n\n"
          "Args:\n"
          "    tasks: List of weighted tasks to optimize for.\n"
          "    constraints: List of constraints to satisfy.\n"
          "    barriers: List of barrier functions for safety constraints (default: []).\n"
          "    scene: Scene containing robot model and state.\n"
          "    delta_q: Pre-allocated numpy array for output (size = num_variables).\n"
          "             Must be a contiguous float64 array. Modified in-place.\n"
          "    regularization: Tikhonov regularization weight for the QP Hessian\n"
          "                    (default: 1e-12). Higher values improve numerical stability\n"
          "                    but may reduce task tracking accuracy.\n\n"
          "Raises:\n"
          "    RuntimeError: If the QP solver fails to find a solution.\n\n"
          "Examples:\n"
          "    # Without barriers:\n"
          "    delta_q = np.zeros(oink.num_variables)\n"
          "    oink.solveIk(tasks, constraints, [], scene, delta_q)\n\n"
          "    # With barriers:\n"
          "    oink.solveIk(tasks, constraints, barriers, scene, delta_q)\n\n"
          "    # With custom regularization:\n"
          "    oink.solveIk(tasks, constraints, barriers, scene, delta_q, 1e-6)")
      .def(
          "solveIk",
          [](Oink& self, const std::vector<std::shared_ptr<Task>>& tasks,
             const std::shared_ptr<Scene>& scene, nanobind::DRef<Eigen::VectorXd> delta_q,
             double regularization) {
            auto result = self.solveIk(tasks, *scene, delta_q, regularization);
            if (!result.has_value()) {
              throw std::runtime_error("IK solve failed: " + result.error());
            }
          },
          "tasks"_a, "scene"_a, "delta_q"_a, "regularization"_a = 1e-12,
          "Solve inverse kinematics for tasks only (no constraints or barriers).\n\n"
          "Args:\n"
          "    tasks: List of weighted tasks to optimize for.\n"
          "    scene: Scene containing robot model and state.\n"
          "    delta_q: Pre-allocated numpy array for output (size = num_variables).\n"
          "    regularization: Tikhonov regularization weight (default: 1e-12).\n\n"
          "Example:\n"
          "    delta_q = np.zeros(oink.num_variables)\n"
          "    oink.solveIk(tasks, scene, delta_q)")
      .def(
          "solveIk",
          [](Oink& self, const std::vector<std::shared_ptr<Task>>& tasks,
             const std::vector<std::shared_ptr<Constraints>>& constraints,
             const std::shared_ptr<Scene>& scene, nanobind::DRef<Eigen::VectorXd> delta_q,
             double regularization) {
            auto result = self.solveIk(tasks, constraints, *scene, delta_q, regularization);
            if (!result.has_value()) {
              throw std::runtime_error("IK solve failed: " + result.error());
            }
          },
          "tasks"_a, "constraints"_a, "scene"_a, "delta_q"_a, "regularization"_a = 1e-12,
          "Solve inverse kinematics for tasks with constraints (no barriers).\n\n"
          "Args:\n"
          "    tasks: List of weighted tasks to optimize for.\n"
          "    constraints: List of constraints to satisfy.\n"
          "    scene: Scene containing robot model and state.\n"
          "    delta_q: Pre-allocated numpy array for output (size = num_variables).\n"
          "    regularization: Tikhonov regularization weight (default: 1e-12).\n\n"
          "Example:\n"
          "    delta_q = np.zeros(oink.num_variables)\n"
          "    oink.solveIk(tasks, constraints, scene, delta_q)")
      .def(
          "solveIk",
          [](Oink& self, const std::vector<std::shared_ptr<Task>>& tasks,
             const std::vector<std::shared_ptr<Barrier>>& barriers,
             const std::shared_ptr<Scene>& scene, nanobind::DRef<Eigen::VectorXd> delta_q,
             double regularization) {
            auto result = self.solveIk(tasks, barriers, *scene, delta_q, regularization);
            if (!result.has_value()) {
              throw std::runtime_error("IK solve failed: " + result.error());
            }
          },
          "tasks"_a, "barriers"_a, "scene"_a, "delta_q"_a, "regularization"_a = 1e-12,
          "Solve inverse kinematics for tasks with barriers (no constraints).\n\n"
          "Args:\n"
          "    tasks: List of weighted tasks to optimize for.\n"
          "    barriers: List of barrier functions for safety constraints.\n"
          "    scene: Scene containing robot model and state.\n"
          "    delta_q: Pre-allocated numpy array for output (size = num_variables).\n"
          "    regularization: Tikhonov regularization weight (default: 1e-12).\n\n"
          "Example:\n"
          "    delta_q = np.zeros(oink.num_variables)\n"
          "    oink.solveIk(tasks, barriers, scene, delta_q)")
      .def(
          "enforceBarriers",
          [](Oink& self, const std::vector<std::shared_ptr<Barrier>>& barriers,
             std::shared_ptr<Scene>& scene, nanobind::DRef<Eigen::VectorXd> delta_q,
             double tolerance) {
            auto result = self.enforceBarriers(barriers, *scene, delta_q, tolerance);
            if (!result.has_value()) {
              throw std::runtime_error("Barrier enforcement failed: " + result.error());
            }
          },
          "barriers"_a, "scene"_a, "delta_q"_a, "tolerance"_a = 0.0,
          "Validate delta_q against barriers using forward kinematics.\n\n"
          "This method provides a post-solve safety check by evaluating the actual barrier\n"
          "values at the candidate configuration (q + delta_q). If any barrier would be\n"
          "violated, delta_q is set to zero to prevent unsafe motion.\n\n"
          "This is a backup safety mechanism for cases where the linearized CBF constraint\n"
          "in the QP has significant error (e.g., large jumps, near-boundary configurations).\n\n"
          "Args:\n"
          "    barriers: List of barrier functions to check.\n"
          "    scene: Scene containing robot model and state (current configuration q).\n"
          "    delta_q: Configuration displacement to validate. Modified in place: set to\n"
          "             zero if barrier violation is detected.\n"
          "    tolerance: Tolerance for barrier violation detection. A barrier is considered\n"
          "               violated if h(q + delta_q) < -tolerance. Default is 0.0.\n\n"
          "Raises:\n"
          "    RuntimeError: If barrier evaluation fails (e.g., frame not found).\n\n"
          "Example:\n"
          "    delta_q = np.zeros(oink.num_variables)\n"
          "    oink.solveIk(tasks, constraints, barriers, scene, delta_q)\n"
          "    oink.enforceBarriers(barriers, scene, delta_q)");
}

}  // namespace roboplan
