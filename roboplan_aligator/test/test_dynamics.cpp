#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>

#include "problem_builder.hpp"
#include "reduced_group_model.hpp"
#include "test_fd_util.hpp"
#include "test_util.hpp"

namespace roboplan {
namespace {

using testing::deterministicState;
using testing::makeSo101Scene;

}  // namespace

// The discretized dynamics are the physics backbone of the package, so their Jacobians get a
// finite-difference check. We FD the SAME integrated dynamics the optimizer builds (via the shared
// problem_builder), verifying the exact code path.
TEST(DynamicsTest, IntegratedDynamicsJacobiansMatchFiniteDifferences) {
  auto scene = makeSo101Scene();
  const ReducedGroupModel rgm(*scene, "arm");
  const auto space = aligator_detail::makePhaseSpace(rgm.reducedModel());

  const int ndx = space.ndx();  // 2 * nv
  const int nu = rgm.nv();      // fully-actuated: nu = nv
  const double dt = 0.01;

  // Central differences in the tangent space: eps ~ 1e-6 leaves total error well under 1e-5.
  const double eps = 1e-6;
  const double tol = 1e-5;

  for (const IntegratorType integ : {IntegratorType::SemiImplicitEuler, IntegratorType::RK2}) {
    const auto dyn = aligator_detail::makeDiscreteDynamics(space, integ, dt);
    const auto data = dyn->createData();

    // A deterministic, non-trivial operating point: fixed tangent offset + constant torque, so both
    // gravity and actuation contribute to the Jacobians.
    const Eigen::VectorXd x = deterministicState(space);
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
