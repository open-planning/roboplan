#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <stdexcept>

#include <roboplan/core/scene.hpp>
#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/configuration.hpp>
#include <roboplan_oink/tasks/frame.hpp>
#include <roboplan_oink/teleop/command_filter.hpp>
#include <roboplan_oink/teleop/teleop_target.hpp>

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
      .def(nanobind::init<double, double, double, double>(), "position_cost"_a = 1.0,
           "orientation_cost"_a = 1.0, "task_gain"_a = 1.0, "lm_damping"_a = 0.0)
      .def_rw("position_cost", &FrameTaskOptions::position_cost, "Position cost weight.")
      .def_rw("orientation_cost", &FrameTaskOptions::orientation_cost, "Orientation cost weight.")
      .def_rw("task_gain", &FrameTaskOptions::task_gain, "Task gain for low-pass filtering.")
      .def_rw("lm_damping", &FrameTaskOptions::lm_damping, "Levenberg-Marquardt damping.");

  // Bind FrameTask inheriting from Task
  nanobind::class_<FrameTask, Task>(m, "FrameTask",
                                    "Task to reach a target pose for a specified frame.")
      .def(nanobind::init<const CartesianConfiguration&, int, const FrameTaskOptions&>(),
           "target_pose"_a, "num_variables"_a, "options"_a = FrameTaskOptions{})
      .def_ro("frame_name", &FrameTask::frame_name, "Name of the frame to control.")
      .def_ro("frame_id", &FrameTask::frame_id,
              "Index of the frame in the scene's Pinocchio model.")
      .def_ro("target_pose", &FrameTask::target_pose, "Target pose for the frame.")
      .def("setTargetFrameTransform", &FrameTask::setTargetFrameTransform,
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

  // Bind Oink solver
  nanobind::class_<CommandFilterConfig>(m, "CommandFilterConfig",
                                        "Parameters for teleop command filtering.")
      .def(nanobind::init<>())
      .def_rw("linear_deadzone", &CommandFilterConfig::linear_deadzone,
              "Linear deadzone threshold (unitless, [0, 1]).")
      .def_rw("angular_deadzone", &CommandFilterConfig::angular_deadzone,
              "Angular deadzone threshold (unitless, [0, 1]).")
      .def_rw("linear_sensitivity", &CommandFilterConfig::linear_sensitivity,
              "Linear sensitivity (m/s at unit input).")
      .def_rw("angular_sensitivity", &CommandFilterConfig::angular_sensitivity,
              "Angular sensitivity (rad/s at unit input).")
      .def_rw("max_linear_speed", &CommandFilterConfig::max_linear_speed,
              "Maximum translational speed (m/s).")
      .def_rw("max_angular_speed", &CommandFilterConfig::max_angular_speed,
              "Maximum angular speed (rad/s).")
      .def_rw("smoothing_alpha", &CommandFilterConfig::smoothing_alpha,
              "EMA smoothing coefficient (unitless, [0, 1]).");

  nanobind::class_<CommandFilter>(m, "CommandFilter",
                                  "Applies deadzone, scaling, clamping, and smoothing.")
      .def(nanobind::init<const CommandFilterConfig&>(), "config"_a = CommandFilterConfig{})
      .def(
          "filter",
          [](CommandFilter& self, nanobind::DRef<Eigen::VectorXd> raw_twist) {
            if (raw_twist.size() != 6) {
              throw std::invalid_argument("raw_twist must have size 6.");
            }
            const Eigen::Matrix<double, 6, 1> input = raw_twist;
            Eigen::VectorXd output = self.filter(input);
            return output;
          },
          "raw_twist"_a,
          "Filter raw normalized twist [vx, vy, vz, wx, wy, wz] into physical units.")
      .def("reset", &CommandFilter::reset, "Reset internal smoothing state.")
      .def("setConfig", &CommandFilter::setConfig, "Set filter configuration.")
      .def(
          "getConfig", [](const CommandFilter& self) { return self.getConfig(); },
          "Get filter configuration.");

  nanobind::enum_<TeleopFrame>(m, "TeleopFrame", "Frame used for teleop pose integration.")
      .value("EndEffector", TeleopFrame::EndEffector)
      .value("World", TeleopFrame::World);

  nanobind::class_<WorkspaceBounds>(m, "WorkspaceBounds", "Axis-aligned workspace bounds.")
      .def(nanobind::init<>())
      .def_rw("min_xyz", &WorkspaceBounds::min_xyz, "Lower XYZ bound in meters.")
      .def_rw("max_xyz", &WorkspaceBounds::max_xyz, "Upper XYZ bound in meters.");

  nanobind::class_<TeleopTarget>(m, "TeleopTarget",
                                 "Maintains and updates a target pose from twist commands.")
      .def(nanobind::init<const Eigen::Matrix4d&, const WorkspaceBounds&, TeleopFrame>(),
           "initial_pose"_a, "bounds"_a = WorkspaceBounds{}, "frame"_a = TeleopFrame::EndEffector)
      .def(
          "update",
          [](TeleopTarget& self, nanobind::DRef<Eigen::VectorXd> twist, double dt) {
            if (twist.size() != 6) {
              throw std::invalid_argument("twist must have size 6.");
            }
            const Eigen::Matrix<double, 6, 1> input = twist;
            self.update(input, dt);
          },
          "twist"_a, "dt"_a, "Integrate twist into target pose using default frame.")
      .def(
          "update",
          [](TeleopTarget& self, nanobind::DRef<Eigen::VectorXd> twist, double dt,
             TeleopFrame frame) {
            if (twist.size() != 6) {
              throw std::invalid_argument("twist must have size 6.");
            }
            const Eigen::Matrix<double, 6, 1> input = twist;
            self.update(input, dt, frame);
          },
          "twist"_a, "dt"_a, "frame"_a, "Integrate twist into target pose using explicit frame.")
      .def("resetTo", &TeleopTarget::resetTo, "Reset target pose to a specified transform.")
      .def("getTarget", &TeleopTarget::getTarget, "Get current target pose as a 4x4 transform.")
      .def("setFrame", &TeleopTarget::setFrame, "Set default integration frame.")
      .def("getFrame", &TeleopTarget::getFrame, "Get default integration frame.")
      .def("setBounds", &TeleopTarget::setBounds, "Set workspace bounds.")
      .def(
          "getBounds", [](const TeleopTarget& self) { return self.getBounds(); },
          "Get workspace bounds.");

  // Bind Oink solver
  nanobind::class_<Oink>(m, "Oink", "Optimal Inverse Kinematics solver.")
      .def(nanobind::init<int>(), "num_variables"_a)
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
          "Solve inverse kinematics for given tasks and constraints.\n\n"
          "Solves a QP optimization problem to compute the joint velocity that minimizes\n"
          "weighted task errors while satisfying all constraints. The result is written\n"
          "directly into the provided delta_q buffer.\n\n"
          "Args:\n"
          "    tasks: List of weighted tasks to optimize for.\n"
          "    constraints: List of constraints to satisfy.\n"
          "    scene: Scene containing robot model and state.\n"
          "    delta_q: Pre-allocated numpy array for output (size = num_variables).\n"
          "             Must be a contiguous float64 array. Modified in-place.\n"
          "    regularization: Tikhonov regularization weight for the QP Hessian.\n"
          "                    Higher values improve numerical stability but may reduce\n"
          "                    task tracking accuracy. Default: 1e-12.\n\n"
          "Raises:\n"
          "    RuntimeError: If the QP solver fails to find a solution.\n\n"
          "Example:\n"
          "    delta_q = np.zeros(oink.num_variables)\n"
          "    oink.solveIk(tasks, constraints, scene, delta_q)");
}

}  // namespace roboplan
