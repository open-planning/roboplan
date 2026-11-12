#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "scp.hpp"

namespace roboplan::detail {
namespace {

constexpr double kRadius = 0.5;

// Nonconvex inequality g(q) = r^2 - q_k^2 <= 0 on joint 0 of node k: keep |q_k| >= r.
class OutsideIntervalTerm : public NonconvexTerm {
public:
  OutsideIntervalTerm(Eigen::Index node, double radius) : node_{node}, radius_{radius} {}

  tl::expected<void, std::string> evaluate(const VexLayout& layout, const Eigen::VectorXd& z,
                                           bool compute_jacobian,
                                           std::vector<NonconvexRow>& rows) override {
    const Eigen::Index index = layout.q(node_);
    NonconvexRow row;
    row.value = radius_ * radius_ - z(index) * z(index);
    if (compute_jacobian) {
      row.jacobian.emplace_back(index, -2.0 * z(index));
    }
    rows.push_back(std::move(row));
    return {};
  }

private:
  Eigen::Index node_;
  double radius_;
};

// One joint, rest-to-rest from 0 to 0 in 1 s: without the term the optimum stays at 0. The term
// forces the middle node out to |q| >= 0.5, which needs the linearization, the ℓ1 slack, the
// trust box and the ratio test.
TEST(ScpTest, NonconvexTermIsSatisfied) {
  const VexLayout layout{.n = 1, .num_nodes = 11, .dt = 0.1};
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd unbounded =
      Eigen::VectorXd::Constant(1, std::numeric_limits<double>::max());
  const JointBounds bounds{
      .q_min = -unbounded, .q_max = unbounded, .v_max = unbounded, .a_max = unbounded};
  const auto problem = buildConvexProblem(layout, zero, zero, bounds, 1.0, 1e-2);

  // A seed slightly on the positive side, so the gradient of the term is nonzero.
  std::vector<Eigen::VectorXd> q_seed(11, zero);
  q_seed.at(5)(0) = 0.05;
  const auto z0 = projectOntoConstraints(problem, q_seed);
  ASSERT_TRUE(z0.has_value()) << z0.error();

  VexOptions options;
  options.max_iterations = 200;
  const std::vector<std::shared_ptr<NonconvexTerm>> terms = {
      std::make_shared<OutsideIntervalTerm>(5, kRadius)};
  const auto result = runScp(problem, terms, *z0, options);
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result->status, VexStatus::Converged);
  EXPECT_GT(result->iterations, 1u);
  EXPECT_LE(result->max_violation, options.tolerance);
  // Converged means the violation is within the SCP tolerance.
  EXPECT_GE(result->z(layout.q(5)), kRadius - options.tolerance);
  // The hard rows still hold (1e-6: QP tolerance 1e-9 accumulated over the 10 segments).
  EXPECT_LT((problem.A * result->z - problem.b).lpNorm<Eigen::Infinity>(), 1e-6);

  // Same input, same output.
  const auto again = runScp(problem, terms, *z0, options);
  ASSERT_TRUE(again.has_value());
  EXPECT_EQ(again->z, result->z);
}

TEST(ScpTest, WithoutTermsConvergesInOneIteration) {
  const VexLayout layout{.n = 1, .num_nodes = 5, .dt = 0.25};
  const Eigen::VectorXd start = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd goal = Eigen::VectorXd::Ones(1);
  const Eigen::VectorXd unbounded =
      Eigen::VectorXd::Constant(1, std::numeric_limits<double>::max());
  const JointBounds bounds{
      .q_min = -unbounded, .q_max = unbounded, .v_max = unbounded, .a_max = unbounded};
  const auto problem = buildConvexProblem(layout, start, goal, bounds, 1.0, 1e-2);
  const auto z0 = projectOntoConstraints(problem, std::vector<Eigen::VectorXd>(5, start));
  ASSERT_TRUE(z0.has_value()) << z0.error();

  const auto result = runScp(problem, {}, *z0, VexOptions{});
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result->status, VexStatus::Converged);
  EXPECT_EQ(result->iterations, 1u);
  EXPECT_LT(result->cost, problem.cost(*z0));
}

}  // namespace
}  // namespace roboplan::detail
