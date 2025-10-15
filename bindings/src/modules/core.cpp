#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_utils.hpp>
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
      .value("UNKNOWN", JointType::UNKNOWN)
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

  nanobind::class_<JointMimicInfo>(m, "JointMimicInfo")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("mimicked_joint_name", &JointMimicInfo::mimicked_joint_name)
      .def_rw("scaling", &JointMimicInfo::scaling)
      .def_rw("offset", &JointMimicInfo::offset);

  nanobind::class_<JointInfo>(m, "JointInfo")
      .def(nanobind::init<const JointType>())
      .def_ro("type", &JointInfo::type)
      .def_ro("num_position_dofs", &JointInfo::num_position_dofs)
      .def_ro("num_velocity_dofs", &JointInfo::num_velocity_dofs)
      .def_ro("limits", &JointInfo::limits)
      .def_ro("mimic_info", &JointInfo::mimic_info);

  nanobind::class_<JointGroupInfo>(m, "JointGroupInfo")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("joint_names", &JointGroupInfo::joint_names)
      .def_rw("joint_indices", &JointGroupInfo::joint_indices)
      .def_rw("q_indices", &JointGroupInfo::q_indices)
      .def_rw("v_indices", &JointGroupInfo::v_indices)
      .def("__repr__", [](const JointGroupInfo& info) {
        std::stringstream ss;
        ss << info;
        return ss.str();
      });

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

void init_core_geometry_wrappers(nanobind::module_& m) {
  nanobind::class_<Box>(m, "Box").def(nanobind::init<const double, const double, const double>(),
                                      "x"_a, "y"_a, "z"_a);
  nanobind::class_<Sphere>(m, "Sphere").def(nanobind::init<const double>(), "radius"_a);
}

void init_core_scene(nanobind::module_& m) {
  nanobind::class_<Scene>(m, "Scene")
      .def(nanobind::init<const std::string&, const std::filesystem::path&,
                          const std::filesystem::path&, const std::vector<std::filesystem::path>&,
                          const std::filesystem::path&>(),
           "name"_a, "urdf_path"_a, "srdf_path"_a,
           "package_paths"_a = std::vector<std::filesystem::path>(),
           "yaml_config_path"_a = std::filesystem::path())
      // There's an ambiguity issue due to file paths vs strings in python. So to use this
      // constructor you MUST specify argument names to clarify to the bindings that you are passing
      // pre-processed string, and not filepaths.
      .def(
          nanobind::init<const std::string&, const std::string&, const std::string&,
                         const std::vector<std::filesystem::path>&, const std::filesystem::path&>(),
          "name"_a, "urdf"_a, "srdf"_a, "package_paths"_a = std::vector<std::filesystem::path>(),
          "yaml_config_path"_a = std::filesystem::path())
      .def("getName", &Scene::getName)
      .def("getJointNames", &Scene::getJointNames)
      .def("getActuatedJointNames", &Scene::getActuatedJointNames)
      .def("getJointInfo", unwrap_expected(&Scene::getJointInfo))
      .def("configurationDistance", &Scene::configurationDistance)
      .def("setRngSeed", &Scene::setRngSeed)
      .def("randomPositions", &Scene::randomPositions)
      .def("randomCollisionFreePositions", &Scene::randomCollisionFreePositions,
           "max_samples"_a = 1000)
      .def("hasCollisions", &Scene::hasCollisions, "q"_a, "debug"_a = false)
      .def("isValidPose", &Scene::isValidPose)
      .def("applyMimics", &Scene::applyMimics)
      .def("toFullJointPositions", &Scene::toFullJointPositions)
      .def("interpolate", &Scene::interpolate)
      .def("forwardKinematics", &Scene::forwardKinematics)
      .def("getFrameId", unwrap_expected(&Scene::getFrameId))
      .def("getJointGroupInfo", unwrap_expected(&Scene::getJointGroupInfo))
      .def("getCurrentJointPositions", &Scene::getCurrentJointPositions)
      .def("setJointPositions", &Scene::setJointPositions)
      .def("getJointPositionIndices", &Scene::getJointPositionIndices)
      .def("addBoxGeometry", unwrap_expected(&Scene::addBoxGeometry))
      .def("addSphereGeometry", unwrap_expected(&Scene::addSphereGeometry))
      .def("updateGeometryPlacement", unwrap_expected(&Scene::updateGeometryPlacement))
      .def("removeGeometry", unwrap_expected(&Scene::removeGeometry))
      .def("getCollisionGeometryIDs", unwrap_expected(&Scene::getCollisionGeometryIds))
      .def("setCollisions", unwrap_expected(&Scene::setCollisions))
      .def("__repr__", [](const Scene& scene) {
        std::stringstream ss;
        ss << scene;
        return ss.str();
      });
}

void init_core_path_utils(nanobind::module_& m) {
  m.def("computeFramePath", &computeFramePath);
  m.def("hasCollisionsAlongPath", &hasCollisionsAlongPath);

  nanobind::class_<PathShortcutter>(m, "PathShortcutter")
      .def(nanobind::init<const std::shared_ptr<Scene>, const std::string&>())
      .def("shortcut", &PathShortcutter::shortcut, "path"_a, "max_step_size"_a, "max_iters"_a = 100,
           "seed"_a = 0)
      .def("getPathLengths", unwrap_expected(&PathShortcutter::getPathLengths))
      .def("getNormalizedPathScaling", unwrap_expected(&PathShortcutter::getNormalizedPathScaling))
      .def("getConfigurationfromNormalizedPathScaling",
           &PathShortcutter::getConfigurationFromNormalizedPathScaling);
}

void init_core_scene_utils(nanobind::module_& m) {
  m.def("collapseContinuousJointPositions", unwrap_expected(&collapseContinuousJointPositions));
  m.def("expandContinuousJointPositions", unwrap_expected(&expandContinuousJointPositions));
}

}  // namespace roboplan
