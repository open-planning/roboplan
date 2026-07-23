#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>

// Internal detail headers (not installed); test/CMakeLists.txt adds ../src to the include path,
// and this target links aligator directly so it can read the discrete-dynamics Jacobians.
#include "problem_builder.hpp"
#include "reduced_group_model.hpp"

namespace roboplan {
namespace {

// SO-101 with the 5-DoF "arm" group: 5 fixed-base revolute joints (gripper locked). A strict
// reduction that loads reliably in core (see test_reduced_group_model.cpp for the fixture
// rationale), which is exactly the fixed-base, fully-actuated setting §3.2 targets.
std::shared_ptr<Scene> makeSo101Scene() {
  const auto model_prefix = example_models::get_package_models_dir();
  const std::vector<std::filesystem::path> package_paths = {
      example_models::get_package_share_dir()};
  return std::make_shared<Scene>("test_scene", model_prefix / "so101_robot_model" / "so101.urdf",
                                 model_prefix / "so101_robot_model" / "so101.srdf", package_paths);
}

}  // namespace

// The discretized dynamics are the physics backbone of the whole package, so §8 mandates a
// finite-difference check of their Jacobians. We finite-difference the SAME integrated dynamics
// the optimizer builds (via the shared problem_builder), so this verifies the exact code path.
TEST(DynamicsTest, IntegratedDynamicsJacobiansMatchFiniteDifferences) {
  auto scene = makeSo101Scene();
  const ReducedGroupModel rgm(*scene, "arm");
  const auto space = aligator_detail::makePhaseSpace(rgm.reducedModel());

  const int ndx = space.ndx();  // 2 * nv
  const int nu = rgm.nv();      // fully-actuated: nu = nv
  const double dt = 0.01;

  // Central differences in the tangent space: truncation O(eps^2) and roundoff O(macheps/eps), so
  // eps ~ 1e-6 leaves total error well under the tolerance.
  const double eps = 1e-6;
  const double tol = 1e-5;

  for (const IntegratorType integ : {IntegratorType::SemiImplicitEuler, IntegratorType::RK2}) {
    const auto dyn = aligator_detail::makeDiscreteDynamics(space, integ, dt);
    const auto data = dyn->createData();

    // A deterministic, non-trivial operating point (reproducible across runs): a fixed tangent
    // offset from the neutral configuration and a constant nonzero torque, so gravity and the
    // actuation both contribute to the Jacobians.
    Eigen::VectorXd delta(ndx);
    for (int i = 0; i < ndx; ++i) {
      delta(i) = 0.1 * (i + 1);
    }
    const Eigen::VectorXd x = space.integrate(space.neutral(), delta);
    const Eigen::VectorXd u = Eigen::VectorXd::Constant(nu, 0.3);

    // aligator's convention is evaluate-then-differentiate: dForward assumes forward has populated
    // the (continuous) dynamics data at the same (x, u), so call forward first.
    dyn->forward(x, u, *data);
    dyn->dForward(x, u, *data);
    const Eigen::MatrixXd J_x = data->Jx();  // d xnext / d x, ndx x ndx
    const Eigen::MatrixXd J_u = data->Ju();  // d xnext / d u, ndx x nu

    // Finite-difference d xnext / d x, perturbing x along each tangent basis direction and taking
    // the retraction (difference) of the two perturbed next-states.
    Eigen::MatrixXd J_x_fd(ndx, ndx);
    for (int i = 0; i < ndx; ++i) {
      Eigen::VectorXd dv = Eigen::VectorXd::Zero(ndx);
      dv(i) = eps;
      dyn->forward(space.integrate(x, dv), u, *data);
      const Eigen::VectorXd xnext_plus = data->xnext_;
      dv(i) = -eps;
      dyn->forward(space.integrate(x, dv), u, *data);
      const Eigen::VectorXd xnext_minus = data->xnext_;
      J_x_fd.col(i) = space.difference(xnext_minus, xnext_plus) / (2.0 * eps);
    }

    // Finite-difference d xnext / d u (controls are Euclidean).
    Eigen::MatrixXd J_u_fd(ndx, nu);
    for (int j = 0; j < nu; ++j) {
      Eigen::VectorXd u_plus = u;
      u_plus(j) += eps;
      dyn->forward(x, u_plus, *data);
      const Eigen::VectorXd xnext_plus = data->xnext_;
      Eigen::VectorXd u_minus = u;
      u_minus(j) -= eps;
      dyn->forward(x, u_minus, *data);
      const Eigen::VectorXd xnext_minus = data->xnext_;
      J_u_fd.col(j) = space.difference(xnext_minus, xnext_plus) / (2.0 * eps);
    }

    const double j_x_err = (J_x - J_x_fd).cwiseAbs().maxCoeff();
    const double j_u_err = (J_u - J_u_fd).cwiseAbs().maxCoeff();
    EXPECT_LT(j_x_err, tol) << "J_x mismatch for integrator " << static_cast<int>(integ);
    EXPECT_LT(j_u_err, tol) << "J_u mismatch for integrator " << static_cast<int>(integ);
  }
}

}  // namespace roboplan
