#include <roboplan_vex/vex.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene_context.hpp>

#include "collision_term.hpp"
#include "scp.hpp"

namespace roboplan {

namespace {

tl::expected<void, std::string> checkConfiguration(const std::string& label,
                                                   const JointConfiguration& configuration,
                                                   const JointGroupInfo& group,
                                                   const detail::JointBounds& bounds) {
  if (!configuration.joint_names.empty() && configuration.joint_names != group.joint_names) {
    return tl::make_unexpected("The " + label + " joint names do not match the group joints.");
  }
  if (configuration.positions.size() != bounds.q_min.size()) {
    return tl::make_unexpected(
        "The " + label + " configuration has " + std::to_string(configuration.positions.size()) +
        " positions, but the group has " + std::to_string(bounds.q_min.size()) + ".");
  }
  if ((configuration.positions.array() < bounds.q_min.array()).any() ||
      (configuration.positions.array() > bounds.q_max.array()).any()) {
    return tl::make_unexpected("The " + label + " configuration is outside the joint limits.");
  }
  return {};
}

// Resamples the seed to `count` nodes, uniformly in arc length. resampleUniform only downsamples
// (it caps the output at the input size), so each seed segment is first subdivided linearly,
// which is exact for vector-space joints.
std::vector<Eigen::VectorXd> resampleSeed(const std::vector<Eigen::VectorXd>& positions,
                                          size_t count, const Scene& scene,
                                          const Eigen::VectorXi& q_indices) {
  const size_t num_segments = positions.size() - 1;
  const size_t pieces = (count - 1 + num_segments - 1) / num_segments;
  std::vector<Eigen::VectorXd> dense;
  dense.reserve(num_segments * pieces + 1);
  for (size_t i = 0; i < num_segments; ++i) {
    for (size_t piece = 0; piece < pieces; ++piece) {
      const double fraction = static_cast<double>(piece) / static_cast<double>(pieces);
      dense.emplace_back(positions.at(i) + fraction * (positions.at(i + 1) - positions.at(i)));
    }
  }
  dense.push_back(positions.back());

  auto resampled = resampleUniform(dense, count, scene, q_indices);
  if (resampled.size() != count) {
    // A seed of zero length is returned with only its endpoints.
    resampled.assign(count, dense.front());
  }
  return resampled;
}

}  // namespace

Vex::Vex(std::shared_ptr<Scene> scene, VexOptions options)
    : scene_{std::move(scene)}, options_{std::move(options)} {
  if (!scene_) {
    throw std::invalid_argument("Vex requires a non-null scene.");
  }
}

tl::expected<VexResult, std::string> Vex::solve(const JointConfiguration& start,
                                                const JointConfiguration& goal, double duration,
                                                const JointPath& seed) {
  if (!std::isfinite(duration) || duration <= 0.0) {
    return tl::make_unexpected("The duration must be positive and finite.");
  }
  if (options_.num_nodes < 3) {
    return tl::make_unexpected("VexOptions::num_nodes must be at least 3.");
  }
  if (options_.velocity_weight < 0.0 || options_.acceleration_weight < 0.0) {
    return tl::make_unexpected("VexOptions cost weights must be nonnegative.");
  }
  if (!(options_.collision_check_distance > options_.collision_margin)) {
    return tl::make_unexpected(
        "VexOptions::collision_check_distance must be greater than collision_margin.");
  }

  const auto maybe_group = scene_->getJointGroupInfo(options_.group_name);
  if (!maybe_group) {
    return tl::make_unexpected(maybe_group.error());
  }
  const auto& group = *maybe_group;
  // Lie-group joints (nq != nv) are not supported yet (design L9).
  for (const auto& joint_name : group.joint_names) {
    const auto joint_info = scene_->getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected(joint_info.error());
    }
    if (joint_info->num_position_dofs != joint_info->num_velocity_dofs) {
      return tl::make_unexpected("Joint '" + joint_name +
                                 "' is not a vector-space joint (nq != nv); vex supports "
                                 "vector-space joints only.");
    }
  }
  if (group.q_indices.size() != group.v_indices.size()) {
    return tl::make_unexpected("The group has nq != nv; vex supports vector-space joints only.");
  }

  const auto position_limits = scene_->getPositionLimitVectors(options_.group_name);
  const auto velocity_limits = scene_->getVelocityLimitVectors(options_.group_name);
  const auto acceleration_limits = scene_->getAccelerationLimitVectors(options_.group_name);
  if (!position_limits || !velocity_limits || !acceleration_limits) {
    return tl::make_unexpected("Failed to read the joint limits of group '" + options_.group_name +
                               "'.");
  }
  const detail::JointBounds bounds{.q_min = position_limits->first,
                                   .q_max = position_limits->second,
                                   .v_max = velocity_limits->second,
                                   .a_max = acceleration_limits->second};

  if (auto checked = checkConfiguration("start", start, group, bounds); !checked) {
    return tl::make_unexpected(checked.error());
  }
  if (auto checked = checkConfiguration("goal", goal, group, bounds); !checked) {
    return tl::make_unexpected(checked.error());
  }
  if (seed.joint_names != group.joint_names) {
    return tl::make_unexpected("The seed joint names do not match the group joints.");
  }
  if (seed.positions.size() < 2) {
    return tl::make_unexpected("The seed must have at least 2 waypoints.");
  }
  for (const auto& positions : seed.positions) {
    if (positions.size() != group.q_indices.size()) {
      return tl::make_unexpected("A seed waypoint has the wrong number of positions.");
    }
  }

  // Per-solve scratch (design L3): never touches the Scene's shared state. The context reads the
  // Scene's current joint positions once, here; joints outside the group keep them.
  const SceneContext context(*scene_);
  if (context.hasCollisions(context.toFullJointPositions(options_.group_name, start.positions))) {
    return tl::make_unexpected("The start configuration is in collision.");
  }
  if (context.hasCollisions(context.toFullJointPositions(options_.group_name, goal.positions))) {
    return tl::make_unexpected("The goal configuration is in collision.");
  }

  const detail::VexLayout layout{.n = group.v_indices.size(),
                                 .num_nodes = static_cast<Eigen::Index>(options_.num_nodes),
                                 .dt = duration / static_cast<double>(options_.num_nodes - 1)};
  const auto problem =
      detail::buildConvexProblem(layout, start.positions, goal.positions, bounds,
                                 options_.velocity_weight, options_.acceleration_weight);

  // The hard rows must hold at the reference before the trust-box loop, so project the seed.
  const auto q_seed = resampleSeed(seed.positions, options_.num_nodes, *scene_, group.q_indices);
  const auto z0 = detail::projectOntoConstraints(problem, q_seed);
  if (!z0) {
    return tl::make_unexpected("No trajectory of duration " + std::to_string(duration) +
                               " s satisfies the boundary conditions and joint limits (" +
                               z0.error() + ")");
  }

  std::vector<std::shared_ptr<detail::NonconvexTerm>> terms;
  const auto& active_pairs = context.getCollisionData().activeCollisionPairs;
  if (std::ranges::find(active_pairs, true) != active_pairs.end()) {
    terms.push_back(std::make_shared<detail::CollisionTerm>(
        context, options_.group_name, group.v_indices, options_.samples_per_segment,
        options_.collision_margin, options_.collision_check_distance));
  }
  const auto scp = detail::runScp(problem, terms, *z0, options_);
  if (!scp) {
    return tl::make_unexpected(scp.error());
  }

  VexResult result;
  result.status = scp->status;
  result.iterations = scp->iterations;
  result.cost = scp->cost;
  result.max_violation = scp->max_violation;

  auto& trajectory = result.trajectory;
  trajectory.joint_names = group.joint_names;
  const Eigen::Index n = layout.n;
  for (Eigen::Index k = 0; k < layout.num_nodes; ++k) {
    trajectory.times.push_back(static_cast<double>(k) * layout.dt);
    trajectory.positions.emplace_back(scp->z.segment(layout.q(k), n));
    trajectory.velocities.emplace_back(scp->z.segment(layout.v(k), n));
    // Acceleration is constant per segment; the last node reports the last segment's value.
    const Eigen::Index segment = std::min(k, layout.num_nodes - 2);
    trajectory.accelerations.emplace_back(scp->z.segment(layout.a(segment), n));
  }
  // The boundary conditions are hard; remove the QP's residual from the reported endpoints.
  trajectory.positions.front() = start.positions;
  trajectory.positions.back() = goal.positions;
  trajectory.velocities.front().setZero();
  trajectory.velocities.back().setZero();

  // Independent of the status: the flag reports exactly what this dense check covered.
  result.continuous_collision_free =
      detail::isContinuousCollisionFree(context, options_.group_name, trajectory);
  return result;
}

}  // namespace roboplan
