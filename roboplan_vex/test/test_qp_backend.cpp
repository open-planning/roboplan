#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <roboplan_vex/qp_backend.hpp>

namespace roboplan::detail {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

Eigen::SparseMatrix<double> toSparse(const Eigen::MatrixXd& dense) { return dense.sparseView(); }

}  // namespace

// min 1/2 ||x||^2  s.t.  x0 + x1 + x2 = 3  ->  x = (1, 1, 1).
TEST(SparseQpBackend, EqualityOnly) {
  SparseQp qp;
  qp.H = toSparse(Eigen::MatrixXd::Identity(3, 3));
  qp.g = Eigen::VectorXd::Zero(3);
  qp.A = toSparse(Eigen::RowVector3d(1.0, 1.0, 1.0));
  qp.b = Eigen::VectorXd::Constant(1, 3.0);
  qp.C.resize(0, 3);

  const auto result = solveSparseQp(qp, SparseQpSettings{});
  ASSERT_TRUE(result.has_value()) << result.error();
  // Solver eps_abs is 1e-9; 1e-7 leaves margin for the residual-to-solution error ratio.
  EXPECT_TRUE(result->isApprox(Eigen::Vector3d::Ones(), 1e-7));
}

// min 1/2 ||x - (2, -1)||^2  s.t.  x0 <= 1, -inf <= x1 <= inf  ->  x = (1, -1).
TEST(SparseQpBackend, InequalityActive) {
  SparseQp qp;
  qp.H = toSparse(Eigen::MatrixXd::Identity(2, 2));
  qp.g = -Eigen::Vector2d(2.0, -1.0);
  qp.A.resize(0, 2);
  qp.C = toSparse(Eigen::MatrixXd::Identity(2, 2));
  qp.l = Eigen::Vector2d(-kInfinity, -kInfinity);
  qp.u = Eigen::Vector2d(1.0, kInfinity);

  const Eigen::VectorXd warm_start = Eigen::Vector2d(0.5, 0.0);
  const auto result = solveSparseQp(qp, SparseQpSettings{}, warm_start);
  ASSERT_TRUE(result.has_value()) << result.error();
  // Solver eps_abs is 1e-9; 1e-7 leaves margin for the residual-to-solution error ratio.
  EXPECT_NEAR((*result)(0), 1.0, 1e-7);
  EXPECT_NEAR((*result)(1), -1.0, 1e-7);
}

// x0 = 2 and x0 <= 1 cannot both hold.
TEST(SparseQpBackend, InfeasibleReturnsError) {
  SparseQp qp;
  qp.H = toSparse(Eigen::MatrixXd::Identity(1, 1));
  qp.g = Eigen::VectorXd::Zero(1);
  qp.A = toSparse(Eigen::MatrixXd::Ones(1, 1));
  qp.b = Eigen::VectorXd::Constant(1, 2.0);
  qp.C = toSparse(Eigen::MatrixXd::Ones(1, 1));
  qp.l = Eigen::VectorXd::Constant(1, -1.0);
  qp.u = Eigen::VectorXd::Constant(1, 1.0);

  EXPECT_FALSE(solveSparseQp(qp, SparseQpSettings{}).has_value());
}

TEST(SparseQpBackend, InconsistentDimensionsReturnError) {
  SparseQp qp;
  qp.H = toSparse(Eigen::MatrixXd::Identity(2, 2));
  qp.g = Eigen::VectorXd::Zero(3);
  EXPECT_FALSE(solveSparseQp(qp, SparseQpSettings{}).has_value());
}

}  // namespace roboplan::detail
