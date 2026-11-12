#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <tl/expected.hpp>

#include <roboplan/core/scene_context.hpp>
#include <roboplan/core/types.hpp>

#include "scp.hpp"

// Internal collision term and continuous post-check of roboplan_vex (design §6, L2, L10).
namespace roboplan::detail {

/// @brief Collision avoidance d_safe - sd(q(t)) <= 0 for every active collision pair of the
/// Scene, at every interior node and at `samples_per_segment` interior points of every segment.
/// @details A sample at time t in segment k uses the exact M2 polynomial
/// q(t) = q_k + v_k t + a_k t^2 / 2, so its Jacobian with respect to (q_k, v_k, a_k) is
/// (I, t I, t^2/2 I) times the group columns of the signed-distance gradient. Joints outside the
/// group are taken from the context's configuration. Every active pair with sd < d_check yields
/// one inequality row, with or without the Jacobian, so the merit sees the same pairs as the QP
/// (pairs with sd >= d_check have a negative value and no violation).
class CollisionTerm final : public NonconvexTerm {
public:
  CollisionTerm(const SceneContext& context, std::string group_name, Eigen::VectorXi v_indices,
                size_t samples_per_segment, double collision_margin,
                double collision_check_distance);

  tl::expected<void, std::string> evaluate(const VexLayout& layout, const Eigen::VectorXd& z,
                                           bool compute_jacobian,
                                           std::vector<NonconvexRow>& rows) override;

private:
  tl::expected<void, std::string> evaluateSample(const VexLayout& layout, const Eigen::VectorXd& z,
                                                 Eigen::Index segment, double t,
                                                 bool compute_jacobian,
                                                 std::vector<NonconvexRow>& rows);

  const SceneContext& context_;
  std::string group_name_;
  Eigen::VectorXi v_indices_;
  size_t samples_per_segment_;
  double collision_margin_;
  double collision_check_distance_;
  Eigen::RowVectorXd gradient_;  ///< Scratch row of size model.nv.
};

/// @brief Largest per-joint displacement (rad or m) between consecutive configurations of the
/// continuous post-check.
constexpr double kPostCheckStep = 0.01;

/// @brief Checks the trajectory for collisions by sampling the M2 polynomial of every segment so
/// that no group joint moves more than kPostCheckStep between consecutive checks.
/// @details Within a segment the velocity is linear in t, so max(|v_k|, |v_{k+1}|) bounds every
/// joint's speed; the step count follows from that bound. Both endpoints of every segment are
/// checked.
/// @return True if no checked configuration is in collision.
bool isContinuousCollisionFree(const SceneContext& context, const std::string& group_name,
                               const JointTrajectory& trajectory);

}  // namespace roboplan::detail
