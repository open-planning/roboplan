#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

#include <modules/core.hpp>
#include <utils/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_core_types(nanobind::module_& m) {

  nanobind::class_<JointConfiguration>(m, "JointConfiguration")
      .def(nanobind::init<>())  // Default constructor
      .def(nanobind::init<const std::vector<std::string>&, const Eigen::VectorXd&>())
      .def_rw("joint_names", &JointConfiguration::joint_names)
      .def_rw("positions", &JointConfiguration::positions)
      .def_rw("velocities", &JointConfiguration::velocities)
      .def_rw("accelerations", &JointConfiguration::accelerations);

  nanobind::class_<CartesianConfiguration>(m, "CartesianConfiguration")
      .def(nanobind::init<>())  // Default constructor
      .def(nanobind::init<const std::string&, const std::string&, const Eigen::Matrix4d&>())
      .def_rw("base_frame", &CartesianConfiguration::base_frame)
      .def_rw("tip_frame", &CartesianConfiguration::tip_frame)
      .def_rw("tform", &CartesianConfiguration::tform);

  nanobind::enum_<JointType>(m, "JointType")
      .value("PRISMATIC", JointType::PRISMATIC)
      .value("REVOLUTE", JointType::REVOLUTE)
      .value("CONTINUOUS", JointType::CONTINUOUS)
      .value("PLANAR", JointType::PLANAR)
      .value("FLOATING", JointType::FLOATING);

  nanobind::class_<JointLimits>(m, "JointLimits")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("min_position", &JointLimits::min_position)
      .def_rw("max_position", &JointLimits::max_position)
      .def_rw("max_velocity", &JointLimits::max_velocity)
      .def_rw("max_acceleration", &JointLimits::max_acceleration)
      .def_rw("max_jerk", &JointLimits::max_jerk);

  nanobind::class_<JointInfo>(m, "JointInfo")
      .def(nanobind::init<const JointType>())
      .def_ro("type", &JointInfo::type)
      .def_ro("num_position_dofs", &JointInfo::num_position_dofs)
      .def_ro("num_velocity_dofs", &JointInfo::num_velocity_dofs)
      .def_ro("limits", &JointInfo::limits);

  nanobind::class_<JointPath>(m, "JointPath")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("joint_names", &JointPath::joint_names)
      .def_rw("positions", &JointPath::positions)
      .def("__repr__", [](const JointPath& path) {
        std::stringstream ss;
        ss << path;
        return ss.str();
      });

  nanobind::class_<JointTrajectory>(m, "JointTrajectory")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("joint_names", &JointTrajectory::joint_names)
      .def_rw("times", &JointTrajectory::times)
      .def_rw("positions", &JointTrajectory::positions)
      .def_rw("velocities", &JointTrajectory::velocities)
      .def_rw("accelerations", &JointTrajectory::accelerations)
      .def("__repr__", [](const JointTrajectory& traj) {
        std::stringstream ss;
        ss << traj;
        return ss.str();
      });
}

void init_core_scene(nanobind::module_& m) {
  nanobind::class_<Scene>(m, "Scene")
      .def(nanobind::init<const std::string&, const std::filesystem::path&,
                          const std::filesystem::path&, const std::vector<std::filesystem::path>&,
                          const std::filesystem::path&>(),
           "name"_a, "urdf_path"_a, "srdf_path"_a,
           "package_paths"_a = std::vector<std::filesystem::path>(),
           "yaml_config_path"_a = std::filesystem::path())
      .def("getName", &Scene::getName)
      .def("getJointNames", &Scene::getJointNames)
      .def("getJointInfo", &Scene::getJointInfo)
      .def("configurationDistance", &Scene::configurationDistance)
      .def("setRngSeed", &Scene::setRngSeed)
      .def("randomPositions", &Scene::randomPositions)
      .def("randomCollisionFreePositions", &Scene::randomCollisionFreePositions,
           "max_samples"_a = 1000)
      .def("hasCollisions", &Scene::hasCollisions)
      .def("isValidPose", &Scene::isValidPose)
      .def("interpolate", &Scene::interpolate)
      .def("forwardKinematics", &Scene::forwardKinematics)
      .def("getFrameId", unwrap_expected(&Scene::getFrameId))
      .def("__repr__", [](const Scene& scene) {
        std::stringstream ss;
        ss << scene;
        return ss.str();
      });
}

void init_core_path_utils(nanobind::module_& m) {
  m.def("computeFramePath", &computeFramePath);
  m.def("hasCollisionsAlongPath", &hasCollisionsAlongPath);
  m.def("shortcutPath", &shortcutPath, "scene"_a, "path"_a, "max_step_size"_a, "max_iters"_a = 100,
        "seed"_a = 0);
  m.def("getPathLengths", unwrap_expected(&getPathLengths));
  m.def("getNormalizedPathScaling", unwrap_expected(&getNormalizedPathScaling));
  m.def("getConfigurationFromNormalizedPathScaling", &getConfigurationFromNormalizedPathScaling);
}

}  // namespace roboplan
