#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
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

  // Bind FrameTaskParams configuration struct
  nanobind::class_<FrameTaskParams>(m, "FrameTaskParams", "Parameters for FrameTask.")
      .def(nanobind::init<>())
      .def(
          "__init__",
          [](FrameTaskParams* self, double position_cost, double orientation_cost, double task_gain,
             double lm_damping) {
            new (self) FrameTaskParams{position_cost, orientation_cost, task_gain, lm_damping};
          },
          "position_cost"_a = 1.0, "orientation_cost"_a = 1.0, "task_gain"_a = 1.0,
          "lm_damping"_a = 0.0, "Constructor with custom parameters.")
      .def_rw("position_cost", &FrameTaskParams::position_cost, "Position cost weight.")
      .def_rw("orientation_cost", &FrameTaskParams::orientation_cost, "Orientation cost weight.")
      .def_rw("task_gain", &FrameTaskParams::task_gain, "Task gain for low-pass filtering.")
      .def_rw("lm_damping", &FrameTaskParams::lm_damping, "Levenberg-Marquardt damping.");

  // Bind FrameTask inheriting from Task
  nanobind::class_<FrameTask, Task>(m, "FrameTask",
                                    "Task to reach a target pose for a specified frame.")
      .def(nanobind::init<const std::string&, const CartesianConfiguration&, int,
                          const FrameTaskParams&>(),
           "frame_name"_a, "target_pose"_a, "num_variables"_a, "params"_a = FrameTaskParams{})
      .def_rw("frame_name", &FrameTask::frame_name, "Name of the frame to control.")
      .def_rw("target_pose", &FrameTask::target_pose, "Target pose for the frame.");

  // Bind ConfigurationTaskParams configuration struct
  nanobind::class_<ConfigurationTaskParams>(m, "ConfigurationTaskParams",
                                            "Parameters for ConfigurationTask.")
      .def(nanobind::init<>())
      .def(
          "__init__",
          [](ConfigurationTaskParams* self, double task_gain, double lm_damping) {
            new (self) ConfigurationTaskParams{task_gain, lm_damping};
          },
          "task_gain"_a = 1.0, "lm_damping"_a = 0.0)
      .def_rw("task_gain", &ConfigurationTaskParams::task_gain, "Task gain for low-pass filtering.")
      .def_rw("lm_damping", &ConfigurationTaskParams::lm_damping, "Levenberg-Marquardt damping.");

  // Bind ConfigurationTask inheriting from Task
  nanobind::class_<ConfigurationTask, Task>(m, "ConfigurationTask",
                                            "Task to reach a target joint configuration.")
      .def(nanobind::init<const Eigen::VectorXd&, const Eigen::VectorXd&,
                          const ConfigurationTaskParams&>(),
           "target_q"_a, "joint_weights"_a, "params"_a = ConfigurationTaskParams{})
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
  nanobind::class_<Oink>(m, "Oink", "Optimal Inverse Kinematics solver.")
      .def(nanobind::init<int>(), "num_variables"_a)
      .def(
          "solveIk",
          [](Oink& self, const std::vector<std::shared_ptr<Task>>& tasks,
             const std::vector<std::shared_ptr<Constraints>>& constraints,
             const std::shared_ptr<Scene>& scene) -> Eigen::VectorXd {
            Eigen::VectorXd delta_q;
            auto result = self.solveIk(tasks, constraints, *scene, delta_q);
            if (!result.has_value()) {
              throw std::runtime_error("IK solve failed: " + result.error());
            }
            return delta_q;
          },
          "tasks"_a, "constraints"_a, "scene"_a,
          "Solve inverse kinematics with constraints and return delta_q. Raises RuntimeError on "
          "failure.");
}

}  // namespace roboplan
