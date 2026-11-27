#include <roboplan_vex/vex.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
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

// Rejects an endpoint closer than the collision margin to anything, over all active collision
// pairs: the trajectory starts / ends there, so such a margin row can never be satisfied (and a
// pair the group cannot move never changes). In collision is the same case; hasCollisions() backs
// up the distances because coal reports mesh-mesh distances unsigned.
tl::expected<void, std::string> checkClearance(const std::string& label,
                                               const SceneContext& context,
                                               const Eigen::VectorXd& q_full,
                                               const VexOptions& options) {
  context.computeDistances(q_full, options.collision_check_distance);
  const auto& geometry_model = context.getCollisionModel();
  double min_distance = std::numeric_limits<double>::infinity();
  size_t closest_pair = 0;
  const auto scanned = detail::forEachActivePairDistance(
      context, " at the " + label + " configuration", [&](size_t pair, double distance) {
        if (distance < min_distance) {
          min_distance = distance;
          closest_pair = pair;
        }
      });
  if (!scanned) {
    return scanned;
  }
  const bool in_collision = min_distance <= 0.0 || context.hasCollisions(q_full);
  if (!in_collision && min_distance >= options.collision_margin) {
    return {};
  }
  const auto& pair = geometry_model.collisionPairs.at(closest_pair);
  std::ostringstream message;
  message << "The " << label << " configuration "
          << (in_collision ? "is in collision" : "has a clearance below collision_margin") << ": '"
          << geometry_model.geometryObjects.at(pair.first).name << "' and '"
          << geometry_model.geometryObjects.at(pair.second).name << "' have a signed distance of "
          << min_distance << " m, below VexOptions::collision_margin = " << options.collision_margin
          << " m. Increase the " << label << "'s clearance or reduce VexOptions::collision_margin.";
  return tl::make_unexpected(message.str());
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

// What validateInput() resolves from the scene: the group and its joint limits.
struct ValidatedInput {
  JointGroupInfo group;
  detail::JointBounds bounds;
};

// Stage ① (input check), everything that does not need collision geometry.
tl::expected<ValidatedInput, std::string>
validateInput(const Scene& scene, const VexOptions& options, const JointConfiguration& start,
              const JointConfiguration& goal, double duration, const JointPath& seed) {
  if (!std::isfinite(duration) || duration <= 0.0) {
    return tl::make_unexpected("The duration must be positive and finite.");
  }
  if (options.num_nodes < 3) {
    return tl::make_unexpected("VexOptions::num_nodes must be at least 3.");
  }
  if (options.velocity_weight < 0.0 || options.acceleration_weight < 0.0) {
    return tl::make_unexpected("VexOptions cost weights must be nonnegative.");
  }
  if (!(options.collision_check_distance > options.collision_margin)) {
    return tl::make_unexpected(
        "VexOptions::collision_check_distance must be greater than collision_margin.");
  }

  auto maybe_group = scene.getJointGroupInfo(options.group_name);
  if (!maybe_group) {
    return tl::make_unexpected(maybe_group.error());
  }
  auto& group = *maybe_group;
  // Lie-group joints (nq != nv) are not supported yet (design L9).
  for (const auto& joint_name : group.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected(joint_info.error());
    }
    if (joint_info->num_position_dofs != joint_info->num_velocity_dofs) {
      return tl::make_unexpected("Joint '" + joint_name +
                                 "' is not a vector-space joint (nq != nv); vex supports "
                                 "vector-space joints only.");
    }
  }

  const auto position_limits = scene.getPositionLimitVectors(options.group_name);
  const auto velocity_limits = scene.getVelocityLimitVectors(options.group_name);
  const auto acceleration_limits = scene.getAccelerationLimitVectors(options.group_name);
  if (!position_limits || !velocity_limits || !acceleration_limits) {
    return tl::make_unexpected("Failed to read the joint limits of group '" + options.group_name +
                               "'.");
  }
  detail::JointBounds bounds{.q_min = position_limits->first,
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
  return ValidatedInput{.group = std::move(group), .bounds = std::move(bounds)};
}

// The convex problem and a starting point for the SCP loop.
struct InitialPoint {
  detail::ConvexProblem problem;
  Eigen::VectorXd z0;
};

// Stage ②: builds the convex problem and projects the resampled seed onto its hard rows.
tl::expected<InitialPoint, std::string>
computeInitialPoint(const Scene& scene, const VexOptions& options, const ValidatedInput& input,
                    const JointConfiguration& start, const JointConfiguration& goal,
                    double duration, const JointPath& seed) {
  const detail::VexLayout layout{.n = input.group.v_indices.size(),
                                 .num_nodes = static_cast<Eigen::Index>(options.num_nodes),
                                 .dt = duration / static_cast<double>(options.num_nodes - 1)};
  auto problem = detail::buildConvexProblem(layout, start.positions, goal.positions, input.bounds,
                                            options.velocity_weight, options.acceleration_weight);

  // The hard rows must hold at the reference before the trust-box loop, so project the seed.
  const auto q_seed = resampleSeed(seed.positions, options.num_nodes, scene, input.group.q_indices);
  auto z0 = detail::projectOntoConstraints(problem, q_seed);
  if (!z0) {
    return tl::make_unexpected("No trajectory of duration " + std::to_string(duration) +
                               " s satisfies the boundary conditions and joint limits (" +
                               z0.error() + ")");
  }
  return InitialPoint{.problem = std::move(problem), .z0 = std::move(*z0)};
}

// Stage ③'s term: collision avoidance, only if the scene has active collision pairs.
std::vector<std::shared_ptr<detail::NonconvexTerm>>
makeTerms(const SceneContext& context, const VexOptions& options, const JointGroupInfo& group) {
  std::vector<std::shared_ptr<detail::NonconvexTerm>> terms;
  const auto& active_pairs = context.getCollisionData().activeCollisionPairs;
  if (std::ranges::find(active_pairs, true) != active_pairs.end()) {
    terms.push_back(std::make_shared<detail::CollisionTerm>(
        context, options.group_name, group.v_indices, options.samples_per_segment,
        options.collision_margin, options.collision_check_distance));
  }
  return terms;
}

// Unpacks z into a trajectory with t_k = k dt.
JointTrajectory toTrajectory(const Eigen::VectorXd& z, const detail::VexLayout& layout,
                             const JointGroupInfo& group, const JointConfiguration& start,
                             const JointConfiguration& goal) {
  JointTrajectory trajectory;
  trajectory.joint_names = group.joint_names;
  const Eigen::Index n = layout.n;
  for (Eigen::Index k = 0; k < layout.num_nodes; ++k) {
    trajectory.times.push_back(static_cast<double>(k) * layout.dt);
    trajectory.positions.emplace_back(z.segment(layout.q(k), n));
    trajectory.velocities.emplace_back(z.segment(layout.v(k), n));
    // Acceleration is constant per segment; the last node reports the last segment's value.
    const Eigen::Index segment = std::min(k, layout.num_nodes - 2);
    trajectory.accelerations.emplace_back(z.segment(layout.a(segment), n));
  }
  // The boundary conditions are hard; remove the QP's residual from the reported endpoints.
  trajectory.positions.front() = start.positions;
  trajectory.positions.back() = goal.positions;
  trajectory.velocities.front().setZero();
  trajectory.velocities.back().setZero();
  return trajectory;
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
  // ① Input check.
  const auto input = validateInput(*scene_, options_, start, goal, duration, seed);
  if (!input) {
    return tl::make_unexpected(input.error());
  }
  // Per-solve scratch (design L3): never touches the Scene's shared state. The context reads the
  // Scene's current joint positions once, here; joints outside the group keep them.
  const SceneContext context(*scene_);
  if (auto checked = checkClearance(
          "start", context, context.toFullJointPositions(options_.group_name, start.positions),
          options_);
      !checked) {
    return tl::make_unexpected(checked.error());
  }
  if (auto checked = checkClearance(
          "goal", context, context.toFullJointPositions(options_.group_name, goal.positions),
          options_);
      !checked) {
    return tl::make_unexpected(checked.error());
  }

  // ② Initial point.
  const auto initial = computeInitialPoint(*scene_, options_, *input, start, goal, duration, seed);
  if (!initial) {
    return tl::make_unexpected(initial.error());
  }

  // ③–⑥ SCP loop.
  const auto terms = makeTerms(context, options_, input->group);
  const auto scp = detail::runScp(initial->problem, terms, initial->z0, options_);
  if (!scp) {
    return tl::make_unexpected(scp.error());
  }

  VexResult result;
  result.status = scp->status;
  result.iterations = scp->iterations;
  result.cost = scp->cost;
  result.max_violation = scp->max_violation;
  result.trajectory = toTrajectory(scp->z, initial->problem.layout, input->group, start, goal);

  // ⑦ Post-check. Independent of the status: the flag reports exactly what this dense check
  // covered.
  result.continuous_collision_free =
      detail::isContinuousCollisionFree(context, options_.group_name, result.trajectory);
  return result;
}

}  // namespace roboplan
