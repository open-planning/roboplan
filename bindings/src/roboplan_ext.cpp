#include <iostream>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

NB_MODULE(roboplan, m) {

  nanobind::module_ m_core = m.def_submodule("core", "Core roboplan module");

  nanobind::class_<JointConfiguration>(m_core, "JointConfiguration")
      .def(nanobind::init<>()) // Default constructor
      .def(nanobind::init<const std::vector<std::string>&,
                          const Eigen::VectorXd&>())
      .def_rw("joint_names", &JointConfiguration::joint_names)
      .def_rw("positions", &JointConfiguration::positions)
      .def_rw("velocities", &JointConfiguration::velocities)
      .def_rw("accelerations", &JointConfiguration::accelerations);

  nanobind::class_<CartesianConfiguration>(m_core, "CartesianConfiguration")
      .def(nanobind::init<>()) // Default constructor
      .def(nanobind::init<const std::string&, const std::string&,
                          const Eigen::Matrix4d&>())
      .def_rw("base_frame", &CartesianConfiguration::base_frame)
      .def_rw("tip_frame", &CartesianConfiguration::tip_frame)
      .def_rw("tform", &CartesianConfiguration::tform);

  nanobind::class_<Scene>(m_core, "Scene")
      .def(nanobind::init<const std::string&, const std::filesystem::path&,
                          const std::filesystem::path&,
                          const std::vector<std::filesystem::path>&>())
      .def("print", &Scene::print);
}

} // namespace roboplan
