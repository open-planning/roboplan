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

  // Bind the abstract Task base class with shared_ptr holder
  nanobind::class_<Task>(m, "Task")
      .def_ro("gain", &Task::gain)
      .def_ro("weight", &Task::weight)
      .def_ro("lm_damping", &Task::lm_damping)
      .def_ro("num_variables", &Task::num_variables)
      .def_rw("jacobian_container", &Task::jacobian_container)
      .def_rw("error_container", &Task::error_container)
      .def_rw("H_dense", &Task::H_dense);

  // Bind FrameTaskParams configuration struct
  nanobind::class_<FrameTaskParams>(m, "FrameTaskParams")
      .def(nanobind::init<>())
      .def(
          "__init__",
          [](FrameTaskParams* self, double position_cost, double orientation_cost, double task_gain,
             double lm_damping) {
            new (self) FrameTaskParams{position_cost, orientation_cost, task_gain, lm_damping};
          },
          "position_cost"_a = 1.0, "orientation_cost"_a = 1.0, "task_gain"_a = 1.0,
          "lm_damping"_a = 0.0)
      .def_rw("position_cost", &FrameTaskParams::position_cost)
      .def_rw("orientation_cost", &FrameTaskParams::orientation_cost)
      .def_rw("task_gain", &FrameTaskParams::task_gain)
      .def_rw("lm_damping", &FrameTaskParams::lm_damping);

  // Bind FrameTask inheriting from Task
  nanobind::class_<FrameTask, Task>(m, "FrameTask")
      .def(nanobind::init<const std::string&, const CartesianConfiguration&, int,
                          const FrameTaskParams&>(),
           "frame_name"_a, "target_pose"_a, "num_variables"_a, "params"_a = FrameTaskParams{})
      .def_rw("frame_name", &FrameTask::frame_name)
      .def_rw("target_pose", &FrameTask::target_pose)
      .def(
          "computeError",
          [](FrameTask& self, const Scene& scene) {
            auto result = self.computeError(scene);
            if (!result.has_value()) {
              throw std::runtime_error("computeError failed: " + result.error());
            }
          },
          "scene"_a)
      .def(
          "computeJacobian",
          [](FrameTask& self, const Scene& scene) {
            auto result = self.computeJacobian(scene);
            if (!result.has_value()) {
              throw std::runtime_error("computeJacobian failed: " + result.error());
            }
          },
          "scene"_a);

  // Bind ConfigurationTaskParams configuration struct
  nanobind::class_<ConfigurationTaskParams>(m, "ConfigurationTaskParams")
      .def(nanobind::init<>())
      .def(
          "__init__",
          [](ConfigurationTaskParams* self, double task_gain, double lm_damping) {
            new (self) ConfigurationTaskParams{task_gain, lm_damping};
          },
          "task_gain"_a = 1.0, "lm_damping"_a = 0.0)
      .def_rw("task_gain", &ConfigurationTaskParams::task_gain)
      .def_rw("lm_damping", &ConfigurationTaskParams::lm_damping);

  // Bind ConfigurationTask inheriting from Task
  nanobind::class_<ConfigurationTask, Task>(m, "ConfigurationTask")
      .def(nanobind::init<const Eigen::VectorXd&, const Eigen::VectorXd&,
                          const ConfigurationTaskParams&>(),
           "target_q"_a, "joint_weights"_a, "params"_a = ConfigurationTaskParams{})
      .def_rw("target_q", &ConfigurationTask::target_q)
      .def_rw("joint_weights", &ConfigurationTask::joint_weights)
      .def(
          "computeError",
          [](ConfigurationTask& self, const Scene& scene) {
            auto result = self.computeError(scene);
            if (!result.has_value()) {
              throw std::runtime_error("computeError failed: " + result.error());
            }
          },
          "scene"_a)
      .def(
          "computeJacobian",
          [](ConfigurationTask& self, const Scene& scene) {
            auto result = self.computeJacobian(scene);
            if (!result.has_value()) {
              throw std::runtime_error("computeJacobian failed: " + result.error());
            }
          },
          "scene"_a);

  // Bind the abstract Constraints base class
  nanobind::class_<Constraints>(m, "Constraints");

  // Bind PositionLimit constraint
  nanobind::class_<PositionLimit, Constraints>(m, "PositionLimit")
      .def(nanobind::init<int, double>(), "num_variables"_a, "gain"_a = 1.0)
      .def_rw("config_limit_gain", &PositionLimit::config_limit_gain)
      .def("computeQpConstraints", &PositionLimit::computeQpConstraints);

  // Bind VelocityLimit constraint
  nanobind::class_<VelocityLimit, Constraints>(m, "VelocityLimit")
      .def(nanobind::init<int, double, const Eigen::VectorXd&>(), "num_variables"_a, "dt"_a,
           "v_max"_a)
      .def_rw("dt", &VelocityLimit::dt)
      .def_rw("v_max", &VelocityLimit::v_max)
      .def("computeQpConstraints", &VelocityLimit::computeQpConstraints);

  // Bind Oink solver
  nanobind::class_<Oink>(m, "Oink")
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
