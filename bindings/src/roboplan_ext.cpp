#include <iostream>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/scene.hpp>
#include <roboplan/types.hpp>

namespace roboplan {

NB_MODULE(roboplan, m) {

  nanobind::module_ m_types = m.def_submodule("types", "Core roboplan types");
  nanobind::class_<JointConfiguration>(m_types, "JointConfiguration")
      .def(nanobind::init<>()) // Default constructor
      .def(nanobind::init<const std::vector<std::string>,
                          const Eigen::VectorXd>())
      .def_rw("joint_names", &JointConfiguration::joint_names)
      .def_rw("positions", &JointConfiguration::positions)
      .def_rw("velocities", &JointConfiguration::velocities)
      .def_rw("accelerations", &JointConfiguration::accelerations);

  nanobind::class_<Scene>(m, "Scene")
      .def(nanobind::init<const std::filesystem::path,
                          const std::filesystem::path>())
      .def("print", &Scene::print);
}

} // namespace roboplan
