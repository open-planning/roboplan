#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tl/expected.hpp>

#include <aligator/core/cost-abstract.hpp>
#include <aligator/core/constraint-set.hpp>
#include <aligator/core/function-abstract.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <aligator/third-party/polymorphic_cxx14.h>

#include <roboplan_aligator/constraint_spec.hpp>
#include <roboplan_aligator/constraints.hpp>
#include <roboplan_aligator/cost_spec.hpp>
#include <roboplan_aligator/costs.hpp>
#include <roboplan_aligator/problem_builder.hpp>
#include <roboplan_aligator/reduced_group_model.hpp>
#include <roboplan_aligator/types.hpp>

namespace roboplan {

class Scene;

class TrajectoryOptimizer {
public:
  TrajectoryOptimizer(std::shared_ptr<Scene> scene, std::string group_name, int horizon, double dt,
                      TrajOptOptions options = {});
  ~TrajectoryOptimizer();
  TrajectoryOptimizer(TrajectoryOptimizer&&) noexcept;
  TrajectoryOptimizer& operator=(TrajectoryOptimizer&&) noexcept;
  TrajectoryOptimizer(const TrajectoryOptimizer&) = delete;
  TrajectoryOptimizer& operator=(const TrajectoryOptimizer&) = delete;

  int horizon() const;
  double dt() const;
  int nq() const;
  int nv() const;
  int nx() const;
  void setInitialState(const Eigen::VectorXd& q, const Eigen::VectorXd& v = Eigen::VectorXd());

  // Costs: spec-based (existing)
  CostHandle addCost(const CostSpec& cost, const StageWindow& window = StageWindow::all(),
                     double weight = 1.0);

  // Costs: direct aligator (NEW)
  CostHandle addCost(xyz::polymorphic<aligator::CostAbstractTpl<double>> cost,
                     const StageWindow& window = StageWindow::all(), double weight = 1.0);

  // Constraints: spec-based (existing)
  void addConstraint(const ConstraintSpec& constraint,
                     const StageWindow& window = StageWindow::all());

  // Constraints: direct aligator (NEW)
  void addConstraint(xyz::polymorphic<aligator::StageFunctionTpl<double>> residual,
                     xyz::polymorphic<aligator::ConstraintSetTpl<double>> set,
                     const StageWindow& window = StageWindow::all());

  void build();
  void resetProblem();

  TrajOptSeed interpolatePath(const std::vector<Eigen::VectorXd>& waypoints) const;
  TrajOptSeed shift(const TrajOptResult& result, int n_steps = 1) const;
  tl::expected<TrajOptResult, std::string> solve(const TrajOptSeed& seed);
  tl::expected<TrajOptResult, std::string> solve(const TrajOptResult& previous);

  // Public accessors for advanced users
  const ReducedGroupModel& reducedGroupModel() const { return rgm_; }
  const aligator_detail::PhaseSpace& phaseSpace() const { return space_; }
  aligator_detail::Problem& problem() { return *problem_; }
  const aligator_detail::Problem& problem() const { return *problem_; }

private:
  std::shared_ptr<Scene> scene_;
  std::string group_name_;
  int horizon_;
  double dt_;
  TrajOptOptions options_;
  ReducedGroupModel rgm_;
  aligator_detail::PhaseSpace space_;
  Eigen::VectorXd x0_;
  std::unique_ptr<aligator_detail::Problem> problem_;
  aligator::SolverProxDDPTpl<double> solver_;
  bool locked_ = false;
};

}  // namespace roboplan
