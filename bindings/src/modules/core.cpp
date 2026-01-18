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

  nanobind::class_<JointConfiguration>(m, "JointConfiguration",
                                       "Represents a robot joint configuration.")
      .def(nanobind::init<>())  // Default constructor
      .def(nanobind::init<const std::vector<std::string>&, const Eigen::VectorXd&>(),
           "joint_names"_a, "positions"_a)
      .def_rw("joint_names", &JointConfiguration::joint_names, "The names of the joints.")
      .def_rw("positions", &JointConfiguration::positions,
              "The joint positions, in the same order as the names.")
      .def_rw("velocities", &JointConfiguration::velocities,
              "The joint velocities, in the same order as the names.")
      .def_rw("accelerations", &JointConfiguration::accelerations,
              "The joint accelerations, in the same order as the names.");

  nanobind::class_<CartesianConfiguration>(m, "CartesianConfiguration",
                                           "Represents a robot Cartesian configuration.")
      .def(nanobind::init<>())  // Default constructor
      .def(nanobind::init<const std::string&, const std::string&, const Eigen::Matrix4d&>(),
           "base_frame"_a, "tip_frame"_a, "tform"_a)
      .def_rw("base_frame", &CartesianConfiguration::base_frame,
              "The name of the base (or reference) frame.")
      .def_rw("tip_frame", &CartesianConfiguration::tip_frame,
              "The name of the tip (or target) frame.")
      .def_rw("tform", &CartesianConfiguration::tform,
              "The transformation matrix from the base to the tip frame.");

  nanobind::enum_<JointType>(m, "JointType",
                             "Enumeration that describes different types of joints.")
      .value("UNKNOWN", JointType::UNKNOWN)
      .value("PRISMATIC", JointType::PRISMATIC)
      .value("REVOLUTE", JointType::REVOLUTE)
      .value("CONTINUOUS", JointType::CONTINUOUS)
      .value("PLANAR", JointType::PLANAR)
      .value("FLOATING", JointType::FLOATING);

  nanobind::class_<JointLimits>(m, "JointLimits", "Contains joint limit information.")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("min_position", &JointLimits::min_position, "The minimum positions of the joint.")
      .def_rw("max_position", &JointLimits::max_position, "The maximum positions of the joint.")
      .def_rw("max_velocity", &JointLimits::max_velocity,
              "The maximum (symmetric) velocities of the joint.")
      .def_rw("max_acceleration", &JointLimits::max_acceleration,
              "The maximum (symmetric) accelerations of the joint.")
      .def_rw("max_jerk", &JointLimits::max_jerk, "The maximum (symmetric) jerks of the joint.");

  nanobind::class_<JointMimicInfo>(m, "JointMimicInfo", "Contains joint mimic information.")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("mimicked_joint_name", &JointMimicInfo::mimicked_joint_name,
              "The name of the joint being mimicked.")
      .def_rw("scaling", &JointMimicInfo::scaling, "The scaling factor for the mimic relationship.")
      .def_rw("offset", &JointMimicInfo::offset, "The offset for the mimic relationship.");

  nanobind::class_<JointInfo>(m, "JointInfo",
                              "Contains joint information relevant to motion planning and control.")
      .def(nanobind::init<const JointType>(), "joint_type"_a)
      .def_ro("type", &JointInfo::type, "The type of the joint.")
      .def_ro("num_position_dofs", &JointInfo::num_position_dofs,
              "The number of positional degrees of freedom.")
      .def_ro("num_velocity_dofs", &JointInfo::num_velocity_dofs,
              "The number of velocity degrees of freedom.")
      .def_ro("limits", &JointInfo::limits,
              "The joint limit information for each degree of freedom.")
      .def_ro("mimic_info", &JointInfo::mimic_info, "The joint mimic information.");

  nanobind::class_<JointGroupInfo>(m, "JointGroupInfo",
                                   "Contains information about a named group of joints.")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("joint_names", &JointGroupInfo::joint_names,
              "The joint names that make up the group.")
      .def_rw("joint_indices", &JointGroupInfo::joint_indices, "The joint indices in the group.")
      .def_rw("q_indices", &JointGroupInfo::q_indices, "The position vector indices in the group.")
      .def_rw("v_indices", &JointGroupInfo::v_indices, "The velocity vector indices in the group.")
      .def_rw("has_continuous_dofs", &JointGroupInfo::has_continuous_dofs,
              "Whether the group has any continuous degrees of freedom.")
      .def_rw("nq_collapsed", &JointGroupInfo::nq_collapsed,
              "The number of collapsed degrees of freedom.")
      .def("__repr__", [](const JointGroupInfo& info) {
        std::stringstream ss;
        ss << info;
        return ss.str();
      });

  nanobind::class_<JointPath>(m, "JointPath", "Contains a path of joint configurations.")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("joint_names", &JointPath::joint_names, "The list of joint names.")
      .def_rw("positions", &JointPath::positions, "The list of joint configuration positions.")
      .def("__repr__", [](const JointPath& path) {
        std::stringstream ss;
        ss << path;
        return ss.str();
      });

  nanobind::class_<JointTrajectory>(m, "JointTrajectory",
                                    "Contains a trajectory of joint configurations.")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("joint_names", &JointTrajectory::joint_names, "The list of joint names.")
      .def_rw("times", &JointTrajectory::times, "The list of times.")
      .def_rw("positions", &JointTrajectory::positions, "The list of joint positions.")
      .def_rw("velocities", &JointTrajectory::velocities, "The list of joint velocities.")
      .def_rw("accelerations", &JointTrajectory::accelerations, "The list of joint acceleration.")
      .def("__repr__", [](const JointTrajectory& traj) {
        std::stringstream ss;
        ss << traj;
        return ss.str();
      });
}

void init_core_geometry_wrappers(nanobind::module_& m) {
  nanobind::class_<Box>(m, "Box", "Temporary wrapper struct to represent a box geometry.")
      .def(nanobind::init<const double, const double, const double>(), "x"_a, "y"_a, "z"_a);
  nanobind::class_<Sphere>(m, "Sphere", "Temporary wrapper struct to represent a sphere geometry.")
      .def(nanobind::init<const double>(), "radius"_a);
}

void init_core_scene(nanobind::module_& m) {
  nanobind::class_<Scene>(m, "Scene", "Primary scene representation for planning and control.")
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
      .def("getName", &Scene::getName, "Gets the scene's name.")
      .def("getJointNames", &Scene::getJointNames,
           "Gets the scene's full joint names, including mimic joints.")
      .def("getActuatedJointNames", &Scene::getActuatedJointNames,
           "Gets the scene's actuated (non-mimic) joint names.")
      .def("getJointInfo", unwrap_expected(&Scene::getJointInfo),
           "Gets the information for a specific joint.", "joint_name"_a)
      .def("configurationDistance", &Scene::configurationDistance,
           "Gets the distance between two joint configurations.", "q_start"_a, "q_end"_a)
      .def("setRngSeed", &Scene::setRngSeed, "Sets the seed for the random number generator (RNG).",
           "seed"_a)
      .def("randomPositions", &Scene::randomPositions,
           "Generates random positions for the robot model.")
      .def("randomCollisionFreePositions", &Scene::randomCollisionFreePositions,
           "Generates random collision-free positions for the robot model.", "max_samples"_a = 1000)
      .def("hasCollisions", &Scene::hasCollisions,
           "Checks collisions at specified joint positions.", "q"_a, "debug"_a = false)
      .def("isValidPose", &Scene::isValidPose,
           "Checks if the specified joint positions are valid with respect to joint limits.", "q"_a)
      // Modification is not guaranteed to be done in place for python objects, as they
      // may be copied by nanobind. To guarantee values are correctly updated, we use
      // a lambda function to return a reference to the changed value.
      .def(
          "applyMimics",
          [](const Scene& self, Eigen::VectorXd& q) -> Eigen::VectorXd {
            self.applyMimics(q);
            return q;
          },
          "Applies mimic joint relationships to a position vector.", "q"_a)
      .def("toFullJointPositions", &Scene::toFullJointPositions,
           "Converts partial joint positions to full joint positions.", "group_name"_a, "q"_a)
      .def("interpolate", &Scene::interpolate, "Interpolates between two joint configurations.",
           "q_start"_a, "q_end"_a, "fraction"_a)
      .def("forwardKinematics", &Scene::forwardKinematics,
           "Calculates forward kinematics for a specific frame.", "q"_a, "frame_name"_a)
      .def("getFrameId", unwrap_expected(&Scene::getFrameId),
           "Get the Pinocchio model ID of a frame by its name.", "name"_a)
      .def("getJointGroupInfo", unwrap_expected(&Scene::getJointGroupInfo),
           "Get the joint group information of a scene by its name.", "name"_a)
      .def("getCurrentJointPositions", &Scene::getCurrentJointPositions,
           "Get the current joint positions for the full robot state.")
      .def("setJointPositions", &Scene::setJointPositions,
           "Set the joint positions for the full robot state.", "positions"_a)
      .def("getJointPositionIndices", &Scene::getJointPositionIndices,
           "Get the joint position indices for a set of joint names.", "joint_names"_a)
      .def("addBoxGeometry", unwrap_expected(&Scene::addBoxGeometry),
           "Adds a box geometry to the scene.", "name"_a, "parent_frame"_a, "box"_a, "tform"_a,
           "color"_a)
      .def("addSphereGeometry", unwrap_expected(&Scene::addSphereGeometry),
           "Adds a sphere geometry to the scene.", "name"_a, "parent_frame"_a, "sphere"_a,
           "tform"_a, "color"_a)
      .def("updateGeometryPlacement", unwrap_expected(&Scene::updateGeometryPlacement),
           "Updates the placement of an object geometry in the scene.", "name"_a, "parent_frame"_a,
           "tform"_a)
      .def("removeGeometry", unwrap_expected(&Scene::removeGeometry),
           "Removes a geometry from the scene.", "name"_a)
      .def("getCollisionGeometryIDs", unwrap_expected(&Scene::getCollisionGeometryIds),
           "Gets a list of collision geometry IDs corresponding to a specified body.", "body"_a)
      .def("setCollisions", unwrap_expected(&Scene::setCollisions),
           "Sets the allowable collisions for a pair of bodies in the model.", "body1"_a, "body2"_a,
           "enable"_a)
      .def("__repr__", [](const Scene& scene) {
        std::stringstream ss;
        ss << scene;
        return ss.str();
      });
}

void init_core_path_utils(nanobind::module_& m) {
  m.def("computeFramePath", &computeFramePath, "Computes the Cartesian path of a specified frame.",
        "scene"_a, "q_start"_a, "q_end"_a, "frame_name"_a, "max_step_size"_a);
  m.def("hasCollisionsAlongPath", &hasCollisionsAlongPath,
        "Checks collisions along a specified configuration space path.", "scene"_a, "q_start"_a,
        "q_end"_a, "max_step_size"_a);

  nanobind::class_<PathShortcutter>(
      m, "PathShortcutter", "Shortcuts joint paths with random sampling and checking connections.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const std::string&>(), "scene"_a,
           "group_name"_a)
      .def("shortcut", &PathShortcutter::shortcut, "Attempts to shortcut a specified path.",
           "path"_a, "max_step_size"_a, "max_iters"_a = 100, "seed"_a = 0)
      .def("getPathLengths", unwrap_expected(&PathShortcutter::getPathLengths),
           "Computes configuration distances from the start to each pose in a path.", "path"_a)
      .def("getNormalizedPathScaling", unwrap_expected(&PathShortcutter::getNormalizedPathScaling),
           "Computes length-normalized scaling values along a JointPath.", "path"_a)
      .def("getConfigurationfromNormalizedPathScaling",
           &PathShortcutter::getConfigurationFromNormalizedPathScaling,
           "Gets joint configurations from a path with normalized joint scalings.", "path"_a,
           "path_scalings"_a, "value"_a);
}

void init_core_scene_utils(nanobind::module_& m) {
  m.def("collapseContinuousJointPositions", unwrap_expected(&collapseContinuousJointPositions),
        "Collapses a joint position vector's continuous joints for downstream algorithms.",
        "scene"_a, "group_name"_a, "q_orig"_a);
  m.def("expandContinuousJointPositions", unwrap_expected(&expandContinuousJointPositions),
        "Expands a joint position vector's continuous joints from downstream algorithms.",
        "scene"_a, "group_name"_a, "q_orig"_a);
}

}  // namespace roboplan
