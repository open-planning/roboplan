#pragma once

#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <tl/expected.hpp>

#include <roboplan/core/scene_context.hpp>
#include <roboplan/core/types.hpp>

#include "scp.hpp"

// Internal collision term and continuous post-check of roboplan_vex (not installed).
namespace roboplan::detail {

/// @brief Calls `visit(pair_index, signed_distance)` for every active pair, using the distances
/// last computed by SceneContext::computeDistances.
/// @param where Suffix of the error message, e.g. " at the start configuration".
/// @return An error naming the pair if a distance is NaN.
template <typename Visitor>
tl::expected<void, std::string> forEachActivePairDistance(const SceneContext& context,
                                                          std::string_view where, Visitor&& visit) {
  const auto& geometry_data = context.getCollisionData();
  for (size_t pair = 0; pair < geometry_data.distanceResults.size(); ++pair) {
    if (!geometry_data.activeCollisionPairs[pair]) {
      continue;
    }
    const double distance = geometry_data.distanceResults[pair].min_distance;
    if (std::isnan(distance)) {
      return tl::make_unexpected("Collision pair " + std::to_string(pair) +
                                 " returned a NaN signed distance" + std::string(where) + ".");
    }
    visit(pair, distance);
  }
  return {};
}

/// @brief Rows d_safe - sd(q(t)) <= 0 for every active pair with sd < d_check, at every interior
/// node and `samples_per_segment` points inside every segment (vex.rst, ③).
/// @details Joints outside the group keep the context's configuration.
class CollisionTerm final : public NonconvexTerm {
public:
  CollisionTerm(const SceneContext& context, std::string group_name, Eigen::VectorXi v_indices,
                size_t samples_per_segment, double collision_margin,
                double collision_check_distance);

  tl::expected<void, std::string> evaluate(const VexLayout& layout, const Eigen::VectorXd& z,
                                           std::vector<NonconvexRow>& rows) override;

private:
  tl::expected<void, std::string> evaluateSample(const VexLayout& layout, const Eigen::VectorXd& z,
                                                 Eigen::Index segment, double t,
                                                 std::vector<NonconvexRow>& rows);

  const SceneContext& context_;
  std::string group_name_;
  Eigen::VectorXi v_indices_;
  size_t samples_per_segment_;
  double collision_margin_;
  double collision_check_distance_;
  Eigen::RowVectorXd gradient_;  ///< Scratch row of size model.nv.
};

/// @brief Largest per-joint displacement (rad or m) between two post-check configurations.
constexpr double kPostCheckStep = 0.01;

/// @brief Checks the trajectory for contact (not for the margin) between the nodes (vex.rst, ⑦).
/// @return True if no checked configuration is in collision.
[[nodiscard]] bool isContinuousCollisionFree(const SceneContext& context,
                                             const std::string& group_name,
                                             const JointTrajectory& trajectory);

}  // namespace roboplan::detail
