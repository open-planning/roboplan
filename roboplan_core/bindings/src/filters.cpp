#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

#include <roboplan/filters/se3_low_pass_filter.hpp>

#include <roboplan_bindings/filters.hpp>

namespace roboplan {

using namespace nanobind::literals;

namespace {

pinocchio::SE3 matrixToSE3(const Eigen::Matrix4d& matrix) {
  return pinocchio::SE3(matrix.block<3, 3>(0, 0), matrix.block<3, 1>(0, 3));
}

Eigen::Matrix4d se3ToMatrix(const pinocchio::SE3& pose) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = pose.rotation();
  matrix.block<3, 1>(0, 3) = pose.translation();
  return matrix;
}

}  // namespace

void init_filters(nanobind::module_& m) {
  nanobind::class_<SE3LowPassFilter>(m, "SE3LowPassFilter",
                                     "First-order low-pass filter for SE3 poses.")
      .def(nanobind::init<double>(), "tau"_a = 0.1)
      .def(
          "reset",
          [](SE3LowPassFilter& self, const Eigen::Matrix4d& pose) {
            self.reset(matrixToSE3(pose));
          },
          "Resets the filter state to a specific pose.", "pose"_a)
      .def(
          "update",
          [](SE3LowPassFilter& self, const Eigen::Matrix4d& target_pose, double dt)
              -> Eigen::Matrix4d { return se3ToMatrix(self.update(matrixToSE3(target_pose), dt)); },
          "Updates the filtered state toward a target pose.", "target_pose"_a, "dt"_a)
      .def("tau", &SE3LowPassFilter::tau, "Returns the filter time constant in seconds.")
      .def("setTau", &SE3LowPassFilter::setTau, "Sets the filter time constant.", "tau"_a)
      .def("isInitialized", &SE3LowPassFilter::isInitialized,
           "Checks whether the filter has an active filtered state.");
}

}  // namespace roboplan
