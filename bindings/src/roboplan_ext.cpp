#include <iostream>
#include <memory>

#include <tl/expected.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_rrt/graph.hpp>
#include <roboplan_rrt/rrt.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

namespace roboplan {

using namespace nanobind::literals;

/// @brief Helper function for checking if a tl::expected return type has a value or an error.
/// @details If a value exists, we return it as is. When handling unexpected values, if
/// the error is string convertible then we can pass it along through a `std::runtime_error`.
/// Otherwise we do not know the details of the underlying exception.
/// @return The unwrapped value, or throw an exception.
/// @throw std::runtime_error if the result is an error.
template <typename Expected> typename Expected::value_type handle_expected(Expected&& result) {
  if (result.has_value()) {
    return result.value();
  } else {
    // TODO: Consider wrapping with a streamable option.
    if constexpr (std::is_convertible_v<typename Expected::error_type, std::string>) {
      throw std::runtime_error(std::string(result.error()));
    } else {
      throw std::runtime_error("Unknown error occurred.");
    }
  }
};

/// @brief Wrapper function binding tl::expected return types in class functions with nanobind.
/// @return The unwrapped value, or throw a runtime_error.
template <typename Class, typename Ret, typename Err, typename... Args>
auto unwrap_expected(tl::expected<Ret, Err> (Class::*method)(Args...)) {
  return [method](Class& self, Args... args) -> Ret {
    return handle_expected((self.*method)(args...));
  };
}

/// @brief Wrapper function binding tl::expected return types in free functions with nanobind.
/// @return The unwrapped value, or throw a runtime_error.
template <typename Ret, typename Err, typename... Args>
auto unwrap_expected(tl::expected<Ret, Err> (*method)(Args...)) {
  return [method](Args... args) -> Ret { return handle_expected((*method)(args...)); };
}

NB_MODULE(roboplan, m) {

  /// Core module
  nanobind::module_ m_core = m.def_submodule("core", "Core roboplan module");

  nanobind::class_<JointConfiguration>(m_core, "JointConfiguration")
      .def(nanobind::init<>())  // Default constructor
      .def(nanobind::init<const std::vector<std::string>&, const Eigen::VectorXd&>())
      .def_rw("joint_names", &JointConfiguration::joint_names)
      .def_rw("positions", &JointConfiguration::positions)
      .def_rw("velocities", &JointConfiguration::velocities)
      .def_rw("accelerations", &JointConfiguration::accelerations);

  nanobind::class_<CartesianConfiguration>(m_core, "CartesianConfiguration")
      .def(nanobind::init<>())  // Default constructor
      .def(nanobind::init<const std::string&, const std::string&, const Eigen::Matrix4d&>())
      .def_rw("base_frame", &CartesianConfiguration::base_frame)
      .def_rw("tip_frame", &CartesianConfiguration::tip_frame)
      .def_rw("tform", &CartesianConfiguration::tform);

  nanobind::enum_<JointType>(m_core, "JointType")
      .value("PRISMATIC", JointType::PRISMATIC)
      .value("REVOLUTE", JointType::REVOLUTE)
      .value("CONTINUOUS", JointType::CONTINUOUS)
      .value("PLANAR", JointType::PLANAR)
      .value("FLOATING", JointType::FLOATING);

  nanobind::class_<JointLimits>(m_core, "JointLimits")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("min_position", &JointLimits::min_position)
      .def_rw("max_position", &JointLimits::max_position)
      .def_rw("max_velocity", &JointLimits::max_velocity);

  nanobind::class_<JointInfo>(m_core, "JointInfo")
      .def(nanobind::init<const JointType>())
      .def_ro("type", &JointInfo::type)
      .def_ro("num_position_dofs", &JointInfo::num_position_dofs)
      .def_ro("num_velocity_dofs", &JointInfo::num_velocity_dofs)
      .def_ro("limits", &JointInfo::limits);

  nanobind::class_<JointPath>(m_core, "JointPath")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("joint_names", &JointPath::joint_names)
      .def_rw("positions", &JointPath::positions)
      .def("__repr__", [](const JointPath& path) {
        std::stringstream ss;
        ss << path;
        return ss.str();
      });

  nanobind::class_<Scene>(m_core, "Scene")
      .def(
          nanobind::init<const std::string&, const std::filesystem::path&,
                         const std::filesystem::path&, const std::vector<std::filesystem::path>&>())
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
      .def("__repr__", [](const Scene& scene) {
        std::stringstream ss;
        ss << scene;
        return ss.str();
      });

  m_core.def("computeFramePath", &computeFramePath);
  m_core.def("hasCollisionsAlongPath", &hasCollisionsAlongPath);
  m_core.def("shortcutPath", &shortcutPath, "scene"_a, "path"_a, "max_step_size"_a,
             "max_iters"_a = 100, "seed"_a = 0);
  m_core.def("getPathLengths", unwrap_expected(&getPathLengths));
  m_core.def("getNormalizedPathScaling", unwrap_expected(&getNormalizedPathScaling));
  m_core.def("getConfigurationFromNormalizedPathScaling",
             &getConfigurationFromNormalizedPathScaling);

  /// Examples module
  nanobind::module_ m_example_models = m.def_submodule("example_models", "Example models");

  m_example_models.def("get_install_prefix", &roboplan_example_models::get_install_prefix);
  m_example_models.def("get_package_share_dir", &roboplan_example_models::get_package_share_dir);

  /// Simple IK module
  nanobind::module_ m_simple_ik = m.def_submodule("simple_ik", "Simple IK solver module");

  nanobind::class_<SimpleIkOptions>(m_simple_ik, "SimpleIkOptions")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("max_iters", &SimpleIkOptions::max_iters)
      .def_rw("step_size", &SimpleIkOptions::step_size)
      .def_rw("damping", &SimpleIkOptions::damping)
      .def_rw("max_error_norm", &SimpleIkOptions::max_error_norm);

  nanobind::class_<SimpleIk>(m_simple_ik, "SimpleIk")
      .def(nanobind::init<const std::shared_ptr<Scene>, const SimpleIkOptions&>())
      .def("solveIk", &SimpleIk::solveIk);

  /// RRT module
  nanobind::module_ m_rrt = m.def_submodule("rrt", "RRT module");

  nanobind::class_<Node>(m_rrt, "Node")
      .def(nanobind::init<const Eigen::VectorXd&, int>())
      .def_ro("config", &Node::config)
      .def_ro("parent_id", &Node::parent_id);

  nanobind::class_<RRTOptions>(m_rrt, "RRTOptions")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("max_nodes", &RRTOptions::max_nodes)
      .def_rw("max_connection_distance", &RRTOptions::max_connection_distance)
      .def_rw("collision_check_step_size", &RRTOptions::collision_check_step_size)
      .def_rw("goal_biasing_probability", &RRTOptions::goal_biasing_probability)
      .def_rw("max_planning_time", &RRTOptions::max_planning_time)
      .def_rw("rrt_connect", &RRTOptions::rrt_connect);

  nanobind::class_<RRT>(m_rrt, "RRT")
      .def(nanobind::init<const std::shared_ptr<Scene>, const RRTOptions&>())
      .def("plan", unwrap_expected(&RRT::plan))
      .def("setRngSeed", &RRT::setRngSeed)
      .def("getNodes", &RRT::getNodes);
}

}  // namespace roboplan
