#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene_utils.hpp>
#include <roboplan_ompl/scene_space.hpp>

namespace roboplan {

namespace ob = ompl::base;

SceneStateSpace::SceneStateSpace(const Scene& scene, const std::string& group_name)
    : scene_{scene}, group_name_{group_name}, q_reference_{scene.getCurrentJointPositions()} {
  const auto maybe_info = scene.getJointGroupInfo(group_name);
  if (!maybe_info) {
    throw std::runtime_error("Could not create OMPL state space: " + maybe_info.error());
  }
  joint_group_info_ = maybe_info.value();

  const auto maybe_limits = scene.getPositionLimitVectors(group_name, /*collapsed=*/true);
  if (!maybe_limits) {
    throw std::runtime_error("Could not create OMPL state space: " + maybe_limits.error());
  }
  const auto& [lower, upper] = maybe_limits.value();

  std::vector<bool> planar_translation_dims;
  for (const auto& joint_name : joint_group_info_.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;  // Mimic joints have no coordinates of their own.
    }
    switch (joint_info.type) {
    case JointType::FLOATING:
      throw std::runtime_error("Floating joints not yet supported by the OMPL state space.");
    case JointType::PLANAR:
      angle_dims_.insert(angle_dims_.end(), {false, false, true});
      planar_translation_dims.insert(planar_translation_dims.end(), {true, true, false});
      has_planar_ = true;
      break;
    case JointType::CONTINUOUS:
      angle_dims_.push_back(true);
      planar_translation_dims.push_back(false);
      break;
    default:
      angle_dims_.push_back(false);
      planar_translation_dims.push_back(false);
    }
  }
  if (angle_dims_.size() != static_cast<size_t>(lower.size())) {
    throw std::runtime_error("Could not create OMPL state space: dimension mismatch for group '" +
                             group_name + "'.");
  }

  // Each run of bounded coordinates is one real vector subspace; each angle is its own SO(2).
  for (size_t i = 0; i < angle_dims_.size();) {
    if (angle_dims_[i]) {
      addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
      ++i;
      continue;
    }
    size_t j = i;
    while (j < angle_dims_.size() && !angle_dims_[j]) {
      ++j;
    }
    const auto size = static_cast<unsigned int>(j - i);
    auto subspace = std::make_shared<ob::RealVectorStateSpace>(size);
    ob::RealVectorBounds bounds(size);
    for (size_t k = i; k < j; ++k) {
      double lo = lower(k);
      double hi = upper(k);
      // OMPL samples uniformly within the bounds, so every coordinate needs finite limits. Planar
      // translation without limits gets the same default range as Scene's random sampling.
      const bool unbounded = !(std::abs(lo) < std::numeric_limits<double>::max()) ||
                             !(std::abs(hi) < std::numeric_limits<double>::max());
      if (unbounded && planar_translation_dims[k]) {
        lo = -kDefaultPlanarJointTranslationLimit;
        hi = kDefaultPlanarJointTranslationLimit;
      } else if (unbounded) {
        throw std::runtime_error("Could not create OMPL state space: coordinate " +
                                 std::to_string(k) + " of group '" + group_name +
                                 "' has no finite position limits.");
      }
      bounds.setLow(k - i, lo);
      bounds.setHigh(k - i, hi);
    }
    subspace->setBounds(bounds);
    addSubspace(subspace, 1.0);
    i = j;
  }
  lock();
  setup();  // Computes the value layout used by copyToReals/copyFromReals.
}

double SceneStateSpace::distance(const ob::State* state1, const ob::State* state2) const {
  if (has_planar_) {
    return scene_.configurationDistance(toFullPositions(state1), toFullPositions(state2));
  }
  const auto* c1 = state1->as<ob::CompoundState>();
  const auto* c2 = state2->as<ob::CompoundState>();
  double sum_squares = 0.0;
  for (unsigned int i = 0; i < componentCount_; ++i) {
    const double d = components_[i]->distance(c1->components[i], c2->components[i]);
    sum_squares += d * d;
  }
  return std::sqrt(sum_squares);
}

void SceneStateSpace::interpolate(const ob::State* from, const ob::State* to, double t,
                                  ob::State* state) const {
  if (!has_planar_) {
    ob::CompoundStateSpace::interpolate(from, to, t, state);
    return;
  }
  const auto q = scene_.interpolate(toFullPositions(from), toFullPositions(to), t);
  fromGroupPositions(q(joint_group_info_.q_indices), state);
}

Eigen::VectorXd SceneStateSpace::toFullPositions(const ob::State* state,
                                                 const Eigen::VectorXd& q_reference) const {
  // Scene::toFullJointPositions looks the group up by name on every call, too slow for the
  // nearest-neighbor hot path.
  Eigen::VectorXd q = q_reference;
  q(joint_group_info_.q_indices) = toGroupPositions(state);
  return q;
}

double SceneStateSpace::getMaximumExtent() const {
  double sum_squares = 0.0;
  for (const auto& component : components_) {
    const double extent = component->getMaximumExtent();
    sum_squares += extent * extent;
  }
  return std::sqrt(sum_squares);
}

Eigen::VectorXd SceneStateSpace::toGroupPositions(const ob::State* state) const {
  std::vector<double> reals;
  copyToReals(reals, state);
  Eigen::VectorXd q(joint_group_info_.q_indices.size());
  Eigen::Index j = 0;
  for (size_t i = 0; i < reals.size(); ++i) {
    if (angle_dims_[i]) {
      q(j++) = std::cos(reals[i]);
      q(j++) = std::sin(reals[i]);
    } else {
      q(j++) = reals[i];
    }
  }
  return q;
}

void SceneStateSpace::fromGroupPositions(const Eigen::VectorXd& q_group, ob::State* state) const {
  if (q_group.size() != joint_group_info_.q_indices.size()) {
    throw std::invalid_argument("Expected " + std::to_string(joint_group_info_.q_indices.size()) +
                                " group positions but got " + std::to_string(q_group.size()) + ".");
  }
  std::vector<double> reals;
  reals.reserve(angle_dims_.size());
  Eigen::Index j = 0;
  for (const bool is_angle : angle_dims_) {
    if (is_angle) {
      reals.push_back(std::atan2(q_group(j + 1), q_group(j)));
      j += 2;
    } else {
      reals.push_back(q_group(j++));
    }
  }
  copyFromReals(state, reals);
}

SceneStateValidityChecker::SceneStateValidityChecker(const ob::SpaceInformationPtr& si,
                                                     std::shared_ptr<SceneContext> context)
    : ob::StateValidityChecker(si), context_{std::move(context)},
      space_{si->getStateSpace()->as<SceneStateSpace>()} {}

bool SceneStateValidityChecker::isValid(const ob::State* state) const {
  ++num_checks_;
  if (!si_->satisfiesBounds(state)) {
    return false;
  }
  const auto q_full =
      context_->toFullJointPositions(space_->getGroupName(), space_->toGroupPositions(state));
  return !context_->hasCollisions(q_full);
}

SceneMotionValidator::SceneMotionValidator(const ob::SpaceInformationPtr& si,
                                           std::shared_ptr<SceneContext> context, double step_size,
                                           bool bisection)
    : ob::MotionValidator(si), context_{std::move(context)},
      space_{si->getStateSpace()->as<SceneStateSpace>()}, step_size_{step_size},
      bisection_{bisection} {}

bool SceneMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2) const {
  // As in OMPL's default validator, s1 is assumed valid and s2 is checked here.
  const auto& q_reference = context_->getJointPositions();
  const bool valid = si_->isValid(s2) &&
                     !hasCollisionsAlongPath(context_->getScene(), *context_,
                                             space_->toFullPositions(s1, q_reference),
                                             space_->toFullPositions(s2, q_reference), step_size_,
                                             bisection_, /*check_endpoints=*/false);
  valid ? ++valid_ : ++invalid_;
  return valid;
}

bool SceneMotionValidator::checkMotion(const ob::State* s1, const ob::State* s2,
                                       std::pair<ob::State*, double>& last_valid) const {
  // Linear walk so the last valid state is known on failure.
  const auto num_steps =
      std::max(1u, static_cast<unsigned int>(std::ceil(space_->distance(s1, s2) / step_size_)));
  auto* test = si_->allocState();
  bool valid = true;
  for (unsigned int i = 1; i <= num_steps && valid; ++i) {
    space_->interpolate(s1, s2, static_cast<double>(i) / num_steps, test);
    if (!si_->isValid(test)) {
      valid = false;
      last_valid.second = static_cast<double>(i - 1) / num_steps;
      if (last_valid.first) {
        space_->interpolate(s1, s2, last_valid.second, last_valid.first);
      }
    }
  }
  si_->freeState(test);
  valid ? ++valid_ : ++invalid_;
  return valid;
}

ob::SpaceInformationPtr makeSpaceInformation(const std::shared_ptr<SceneStateSpace>& space,
                                             const std::shared_ptr<SceneContext>& context,
                                             double collision_check_step_size,
                                             bool collision_check_use_bisection) {
  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker(std::make_shared<SceneStateValidityChecker>(si, context));
  si->setMotionValidator(std::make_shared<SceneMotionValidator>(
      si, context, collision_check_step_size, collision_check_use_bisection));
  si->setup();
  return si;
}

}  // namespace roboplan
