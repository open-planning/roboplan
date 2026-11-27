#include <roboplan_vex/vex.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene_context.hpp>
#include <roboplan/core/scene_utils.hpp>

#include "collision_term.hpp"
#include "scp.hpp"

namespace roboplan {

namespace {

// Checks an endpoint and returns it collapsed (continuous joints as one angle each).
tl::expected<Eigen::VectorXd, std::string>
checkConfiguration(const std::string& label, const JointConfiguration& configuration,
                   const Scene& scene, const std::string& group_name, const JointGroupInfo& group,
                   const detail::JointBounds& bounds) {
  if (!configuration.joint_names.empty() && configuration.joint_names != group.joint_names) {
    return tl::make_unexpected("The " + label + " joint names do not match the group joints.");
  }
  if (configuration.positions.size() != group.q_indices.size()) {
    return tl::make_unexpected(
        "The " + label + " configuration has " + std::to_string(configuration.positions.size()) +
        " positions, but the group has " + std::to_string(group.q_indices.size()) + ".");
  }
  if (!configuration.positions.allFinite()) {
    return tl::make_unexpected("The " + label + " configuration has non-finite positions.");
  }
  auto collapsed = collapseContinuousJointPositions(scene, group_name, configuration.positions);
  if (!collapsed) {
    return tl::make_unexpected("The " + label + " configuration is invalid: " + collapsed.error());
  }
  if ((collapsed->array() < bounds.q_min.array()).any() ||
      (collapsed->array() > bounds.q_max.array()).any()) {
    return tl::make_unexpected("The " + label + " configuration is outside the joint limits.");
  }
  return collapsed;
}

// Rejects an endpoint whose clearance on any active pair is below the margin. A margin row there
// could never be satisfied. hasCollisions() backs up the distances: coal reports mesh-mesh
// distances unsigned.
tl::expected<void, std::string> checkClearance(const std::string& label,
                                               const SceneContext& context,
                                               const Eigen::VectorXd& q_full,
                                               const VexOptions& options) {
  context.computeDistances(q_full, options.collision_check_distance);
  const auto& geometry_model = context.getCollisionModel();
  double min_distance = std::numeric_limits<double>::infinity();
  size_t closest_pair = 0;
  bool found_pair = false;
  auto scanned = detail::forEachActivePairDistance(context, " at the " + label + " configuration",
                                                   [&](size_t pair, double distance) {
                                                     if (distance < min_distance) {
                                                       min_distance = distance;
                                                       closest_pair = pair;
                                                       found_pair = true;
                                                     }
                                                   });
  if (!scanned) {
    return scanned;
  }
  const bool in_collision = min_distance <= 0.0 || context.hasCollisions(q_full);
  if (!in_collision && min_distance >= options.collision_margin) {
    return {};
  }
  if (!found_pair) {
    return tl::make_unexpected("The " + label + " configuration is in collision.");
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

// Shifts each angle of `q` by a multiple of 2 pi to lie within pi of `reference`.
void unwrapTowards(Eigen::VectorXd& q, const Eigen::VectorXd& reference,
                   const std::vector<bool>& is_angle) {
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    if (is_angle.at(static_cast<size_t>(i))) {
      q(i) = reference(i) + std::remainder(q(i) - reference(i), 2.0 * std::numbers::pi);
    }
  }
}

// Resamples the seed to `count` nodes, uniformly in arc length, and returns them collapsed with
// every angle unwrapped continuously from `start`. resampleUniform only downsamples, so each
// seed segment is first subdivided linearly (in the angle, not in cos and sin).
tl::expected<std::vector<Eigen::VectorXd>, std::string>
resampleSeed(const Scene& scene, const std::string& group_name, const JointGroupInfo& group,
             const std::vector<bool>& is_angle, const Eigen::VectorXd& start,
             const std::vector<Eigen::VectorXd>& positions, size_t count) {
  std::vector<Eigen::VectorXd> collapsed;
  collapsed.reserve(positions.size());
  Eigen::VectorXd previous = start;
  for (const auto& waypoint : positions) {
    auto q = collapseContinuousJointPositions(scene, group_name, waypoint);
    if (!q) {
      return tl::make_unexpected("A seed waypoint is invalid: " + q.error());
    }
    unwrapTowards(*q, previous, is_angle);
    previous = *q;
    collapsed.push_back(std::move(*q));
  }

  const size_t num_segments = collapsed.size() - 1;
  const size_t pieces = (count - 1 + num_segments - 1) / num_segments;
  std::vector<Eigen::VectorXd> dense;
  dense.reserve(num_segments * pieces + 1);
  const auto expand = [&](const Eigen::VectorXd& q) {
    return expandContinuousJointPositions(scene, group_name, q);
  };
  for (size_t i = 0; i < num_segments; ++i) {
    for (size_t piece = 0; piece < pieces; ++piece) {
      const double fraction = static_cast<double>(piece) / static_cast<double>(pieces);
      auto q = expand(collapsed.at(i) + fraction * (collapsed.at(i + 1) - collapsed.at(i)));
      if (!q) {
        return tl::make_unexpected(q.error());
      }
      dense.push_back(std::move(*q));
    }
  }
  dense.push_back(positions.back());

  auto resampled = resampleUniform(dense, count, scene, group.q_indices);
  if (resampled.size() != count) {
    // A seed of zero length is returned with only its endpoints.
    resampled.assign(count, dense.front());
  }
  previous = start;
  std::vector<Eigen::VectorXd> nodes;
  nodes.reserve(count);
  for (const auto& q_expanded : resampled) {
    auto q = collapseContinuousJointPositions(scene, group_name, q_expanded);
    if (!q) {
      return tl::make_unexpected(q.error());
    }
    unwrapTowards(*q, previous, is_angle);
    previous = *q;
    nodes.push_back(std::move(*q));
  }
  return nodes;
}

tl::expected<void, std::string> validateOptions(const VexOptions& options) {
  const auto finite_in = [](double value, double low, double high) {
    return std::isfinite(value) && value >= low && value <= high;
  };
  constexpr double max_value = std::numeric_limits<double>::max();
  if (options.num_nodes < 3) {
    return tl::make_unexpected("VexOptions::num_nodes must be at least 3.");
  }
  if (!finite_in(options.velocity_weight, 0.0, max_value) ||
      !finite_in(options.acceleration_weight, 0.0, max_value)) {
    return tl::make_unexpected("VexOptions cost weights must be finite and nonnegative.");
  }
  if (!std::isfinite(options.collision_margin) ||
      !(options.collision_check_distance > options.collision_margin) ||
      !std::isfinite(options.collision_check_distance)) {
    return tl::make_unexpected(
        "VexOptions::collision_check_distance must be finite and greater than collision_margin.");
  }
  if (!(finite_in(options.initial_penalty, 0.0, max_value) && options.initial_penalty > 0.0)) {
    return tl::make_unexpected("VexOptions::initial_penalty must be positive and finite.");
  }
  if (!(finite_in(options.penalty_increase, 1.0, max_value) && options.penalty_increase > 1.0)) {
    return tl::make_unexpected("VexOptions::penalty_increase must be finite and greater than 1.");
  }
  if (!(finite_in(options.initial_trust_box, 0.0, max_value) && options.initial_trust_box > 0.0)) {
    return tl::make_unexpected("VexOptions::initial_trust_box must be positive and finite.");
  }
  if (!(finite_in(options.trust_shrink, 0.0, 1.0) && options.trust_shrink > 0.0 &&
        options.trust_shrink < 1.0)) {
    return tl::make_unexpected("VexOptions::trust_shrink must be in (0, 1).");
  }
  if (!finite_in(options.trust_expand, 1.0, max_value)) {
    return tl::make_unexpected("VexOptions::trust_expand must be finite and at least 1.");
  }
  if (!finite_in(options.improve_ratio, 0.0, 1.0)) {
    return tl::make_unexpected("VexOptions::improve_ratio must be in [0, 1].");
  }
  if (!(finite_in(options.tolerance, 0.0, max_value) && options.tolerance > 0.0)) {
    return tl::make_unexpected("VexOptions::tolerance must be positive and finite.");
  }
  if (!(options.max_time >= 0.0)) {
    return tl::make_unexpected("VexOptions::max_time must not be negative or NaN.");
  }
  return {};
}

struct ValidatedInput {
  JointGroupInfo group;
  detail::JointBounds bounds;
  Eigen::VectorXd start;       ///< Collapsed: one angle per continuous joint.
  Eigen::VectorXd goal;        ///< Collapsed, not yet unwrapped.
  std::vector<bool> is_angle;  ///< Per collapsed coordinate, true for a continuous joint.
};

// Stage ①, everything that does not need collision geometry.
tl::expected<ValidatedInput, std::string>
validateInput(const Scene& scene, const VexOptions& options, const JointConfiguration& start,
              const JointConfiguration& goal, double duration, const JointPath& seed) {
  if (!std::isfinite(duration) || duration <= 0.0) {
    return tl::make_unexpected("The duration must be positive and finite.");
  }
  if (auto checked = validateOptions(options); !checked) {
    return tl::make_unexpected(checked.error());
  }

  auto maybe_group = scene.getJointGroupInfo(options.group_name);
  if (!maybe_group) {
    return tl::make_unexpected(maybe_group.error());
  }
  auto& group = *maybe_group;
  // Vector-space and continuous joints. Continuous joints are optimized as one unbounded angle.
  std::vector<bool> is_angle;
  for (const auto& joint_name : group.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected(joint_info.error());
    }
    if (joint_info->mimic_info) {
      continue;
    }
    if (joint_info->type == JointType::CONTINUOUS) {
      is_angle.push_back(true);
    } else if (joint_info->num_position_dofs == joint_info->num_velocity_dofs) {
      is_angle.insert(is_angle.end(), joint_info->num_position_dofs, false);
    } else {
      return tl::make_unexpected("Joint '" + joint_name +
                                 "' is neither a vector-space nor a continuous joint (nq != nv); "
                                 "vex does not support it.");
    }
  }

  const auto position_limits =
      scene.getPositionLimitVectors(options.group_name, /*collapsed=*/true);
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

  auto start_q = checkConfiguration("start", start, scene, options.group_name, group, bounds);
  if (!start_q) {
    return tl::make_unexpected(start_q.error());
  }
  auto goal_q = checkConfiguration("goal", goal, scene, options.group_name, group, bounds);
  if (!goal_q) {
    return tl::make_unexpected(goal_q.error());
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
    if (!positions.allFinite()) {
      return tl::make_unexpected("A seed waypoint has non-finite positions.");
    }
  }
  return ValidatedInput{.group = std::move(group),
                        .bounds = std::move(bounds),
                        .start = std::move(*start_q),
                        .goal = std::move(*goal_q),
                        .is_angle = std::move(is_angle)};
}

struct InitialPoint {
  detail::ConvexProblem problem;
  Eigen::VectorXd z0;
};

// Stage ②: builds the convex problem and projects the resampled seed onto its hard rows.
tl::expected<InitialPoint, std::string>
computeInitialPoint(const Scene& scene, const VexOptions& options, const ValidatedInput& input,
                    double duration, const JointPath& seed) {
  const detail::VexLayout layout{.n = input.group.v_indices.size(),
                                 .num_nodes = static_cast<Eigen::Index>(options.num_nodes),
                                 .dt = duration / static_cast<double>(options.num_nodes - 1)};
  const auto q_seed = resampleSeed(scene, options.group_name, input.group, input.is_angle,
                                   input.start, seed.positions, options.num_nodes);
  if (!q_seed) {
    return tl::make_unexpected(q_seed.error());
  }
  // The goal angle closest to the seed's end, so the seed decides which way a joint turns.
  Eigen::VectorXd goal = input.goal;
  unwrapTowards(goal, q_seed->back(), input.is_angle);
  auto problem = detail::buildConvexProblem(layout, input.start, goal, input.bounds,
                                            options.velocity_weight, options.acceleration_weight);

  auto z0 = detail::projectOntoConstraints(problem, *q_seed);
  if (!z0) {
    return tl::make_unexpected("No trajectory of duration " + std::to_string(duration) +
                               " s satisfies the boundary conditions and joint limits (" +
                               z0.error() + ")");
  }
  return InitialPoint{.problem = std::move(problem), .z0 = std::move(*z0)};
}

// Collision term (stage ③), only if the scene has active collision pairs.
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

// Positions stay collapsed (one angle per continuous joint) until expandPositions().
JointTrajectory toTrajectory(const Eigen::VectorXd& z, const detail::VexLayout& layout,
                             const JointGroupInfo& group) {
  JointTrajectory trajectory;
  trajectory.joint_names = group.joint_names;
  const Eigen::Index n = layout.n;
  for (Eigen::Index k = 0; k < layout.num_nodes; ++k) {
    trajectory.times.push_back(static_cast<double>(k) * layout.dt);
    trajectory.positions.emplace_back(z.segment(layout.q(k), n));
    trajectory.velocities.emplace_back(z.segment(layout.v(k), n));
    // The last node reports the last segment's acceleration.
    const Eigen::Index segment = std::min(k, layout.num_nodes - 2);
    trajectory.accelerations.emplace_back(z.segment(layout.a(segment), n));
  }
  return trajectory;
}

// Expands the angles to [cos, sin] and removes the QP residual from the hard boundary values.
tl::expected<void, std::string> expandPositions(JointTrajectory& trajectory, const Scene& scene,
                                                const std::string& group_name,
                                                const JointConfiguration& start,
                                                const JointConfiguration& goal) {
  for (auto& q : trajectory.positions) {
    auto expanded = expandContinuousJointPositions(scene, group_name, q);
    if (!expanded) {
      return tl::make_unexpected(expanded.error());
    }
    q = std::move(*expanded);
  }
  trajectory.positions.front() = start.positions;
  trajectory.positions.back() = goal.positions;
  trajectory.velocities.front().setZero();
  trajectory.velocities.back().setZero();
  return {};
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
  // Per-solve scratch: the Scene is never modified. Joints outside the group keep the Scene's
  // current positions.
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
  const auto initial = computeInitialPoint(*scene_, options_, *input, duration, seed);
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
  result.trajectory = toTrajectory(scp->z, initial->problem.layout, input->group);

  // ⑦ Post-check, independent of the status.
  result.continuous_collision_free =
      detail::isContinuousCollisionFree(context, options_.group_name, result.trajectory);
  if (auto expanded = expandPositions(result.trajectory, *scene_, options_.group_name, start, goal);
      !expanded) {
    return tl::make_unexpected(expanded.error());
  }
  return result;
}

}  // namespace roboplan
