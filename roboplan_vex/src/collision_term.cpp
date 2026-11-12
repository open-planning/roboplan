#include "collision_term.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

namespace roboplan::detail {

CollisionTerm::CollisionTerm(const SceneContext& context, std::string group_name,
                             Eigen::VectorXi v_indices, size_t samples_per_segment,
                             double collision_margin, double collision_check_distance)
    : context_{context}, group_name_{std::move(group_name)}, v_indices_{std::move(v_indices)},
      samples_per_segment_{samples_per_segment}, collision_margin_{collision_margin},
      collision_check_distance_{collision_check_distance},
      gradient_{Eigen::RowVectorXd::Zero(context.getModel().nv)} {}

tl::expected<void, std::string> CollisionTerm::evaluate(const VexLayout& layout,
                                                        const Eigen::VectorXd& z,
                                                        bool compute_jacobian,
                                                        std::vector<NonconvexRow>& rows) {
  const auto inner = static_cast<double>(samples_per_segment_ + 1);
  for (Eigen::Index k = 0; k + 1 < layout.num_nodes; ++k) {
    // Node k at t = 0 (the start node 0 is fixed), then the interior samples of segment k.
    if (k > 0) {
      if (auto sampled = evaluateSample(layout, z, k, 0.0, compute_jacobian, rows); !sampled) {
        return sampled;
      }
    }
    for (size_t i = 1; i <= samples_per_segment_; ++i) {
      const double t = static_cast<double>(i) / inner * layout.dt;
      if (auto sampled = evaluateSample(layout, z, k, t, compute_jacobian, rows); !sampled) {
        return sampled;
      }
    }
  }
  return {};
}

tl::expected<void, std::string> CollisionTerm::evaluateSample(const VexLayout& layout,
                                                              const Eigen::VectorXd& z,
                                                              Eigen::Index segment, double t,
                                                              bool compute_jacobian,
                                                              std::vector<NonconvexRow>& rows) {
  const Eigen::Index n = layout.n;
  Eigen::VectorXd q = z.segment(layout.q(segment), n);
  q.noalias() += t * z.segment(layout.v(segment), n);
  q.noalias() += 0.5 * t * t * z.segment(layout.a(segment), n);
  const Eigen::VectorXd q_full = context_.toFullJointPositions(group_name_, q);

  context_.computeDistances(q_full, collision_check_distance_);
  if (compute_jacobian) {
    context_.computeJointJacobians(q_full);
  }

  const auto& geometry_data = context_.getCollisionData();
  const std::array<double, 3> coefficients{1.0, t, 0.5 * t * t};
  const std::array<Eigen::Index, 3> offsets{layout.q(segment), layout.v(segment),
                                            layout.a(segment)};
  const size_t num_blocks = t > 0.0 ? 3 : 1;
  for (size_t pair = 0; pair < geometry_data.distanceResults.size(); ++pair) {
    if (!geometry_data.activeCollisionPairs[pair]) {
      continue;
    }
    const double distance = geometry_data.distanceResults[pair].min_distance;
    if (std::isnan(distance)) {
      return tl::make_unexpected("Collision pair " + std::to_string(pair) +
                                 " returned a NaN signed distance.");
    }
    if (distance >= collision_check_distance_) {
      continue;
    }
    NonconvexRow row{.value = collision_margin_ - distance, .equality = false, .jacobian = {}};
    if (compute_jacobian) {
      context_.computeDistanceJacobian(pair, gradient_);
      row.jacobian.reserve(num_blocks * static_cast<size_t>(n));
      for (size_t block = 0; block < num_blocks; ++block) {
        for (Eigen::Index j = 0; j < n; ++j) {
          row.jacobian.emplace_back(offsets.at(block) + j,
                                    -coefficients.at(block) * gradient_(v_indices_(j)));
        }
      }
    }
    rows.push_back(std::move(row));
  }
  return {};
}

bool isContinuousCollisionFree(const SceneContext& context, const std::string& group_name,
                               const JointTrajectory& trajectory) {
  const auto collides = [&](const Eigen::VectorXd& q) {
    return context.hasCollisions(context.toFullJointPositions(group_name, q));
  };
  for (size_t k = 0; k + 1 < trajectory.positions.size(); ++k) {
    const double dt = trajectory.times.at(k + 1) - trajectory.times.at(k);
    const Eigen::VectorXd& q = trajectory.positions.at(k);
    const Eigen::VectorXd& v = trajectory.velocities.at(k);
    const Eigen::VectorXd& a = trajectory.accelerations.at(k);
    // v(t) = v + a t is linear, so its largest magnitude over [0, dt] is at an endpoint.
    const double max_speed =
        std::max(v.lpNorm<Eigen::Infinity>(), (v + a * dt).lpNorm<Eigen::Infinity>());
    const auto steps =
        static_cast<size_t>(std::max(1.0, std::ceil(max_speed * dt / kPostCheckStep)));
    for (size_t s = 0; s < steps; ++s) {
      const double t = dt * static_cast<double>(s) / static_cast<double>(steps);
      if (collides(q + v * t + 0.5 * t * t * a)) {
        return false;
      }
    }
  }
  return !collides(trajectory.positions.back());
}

}  // namespace roboplan::detail
