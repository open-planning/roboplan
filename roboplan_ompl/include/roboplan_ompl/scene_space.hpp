#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_context.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

/// @brief OMPL state space over a joint group of a Scene.
/// @details Bounded joints become real vector coordinates and continuous rotations (including the
/// rotation of planar joints) become SO(2) subspaces, so the flat state layout equals roboplan's
/// collapsed group positions. Distance and interpolation match the Scene's: the Euclidean norm
/// over all coordinates, except that groups with planar joints defer to the Scene so those joints
/// follow SE(2) geodesics as they do everywhere else in roboplan. Floating joints are not
/// supported.
class SceneStateSpace : public ompl::base::CompoundStateSpace {
public:
  /// @param scene The scene, which must outlive this space.
  SceneStateSpace(const Scene& scene, const std::string& group_name);

  double distance(const ompl::base::State* state1, const ompl::base::State* state2) const override;

  void interpolate(const ompl::base::State* from, const ompl::base::State* to, double t,
                   ompl::base::State* state) const override;

  double getMaximumExtent() const override;

  /// @brief Converts a state to expanded group joint positions, as used by JointConfiguration.
  Eigen::VectorXd toGroupPositions(const ompl::base::State* state) const;

  /// @brief Writes expanded group joint positions into a state.
  void fromGroupPositions(const Eigen::VectorXd& q_group, ompl::base::State* state) const;

  /// @brief Converts a state to full joint positions, taking the joints outside the group from
  /// `q_reference` (by default, the Scene's positions when this space was built).
  Eigen::VectorXd toFullPositions(const ompl::base::State* state) const {
    return toFullPositions(state, q_reference_);
  }
  Eigen::VectorXd toFullPositions(const ompl::base::State* state,
                                  const Eigen::VectorXd& q_reference) const;

  const std::string& getGroupName() const { return group_name_; }

  const JointGroupInfo& getJointGroupInfo() const { return joint_group_info_; }

private:
  const Scene& scene_;
  std::string group_name_;
  JointGroupInfo joint_group_info_;

  /// @brief The Scene's joint positions when this space was built, filling non-group joints.
  Eigen::VectorXd q_reference_;

  /// @brief Per collapsed coordinate, true if it is an angle expanded to [cos, sin].
  std::vector<bool> angle_dims_;

  /// @brief True if the group has a planar joint, whose SE(2) geodesics only the Scene knows.
  bool has_planar_ = false;
};

/// @brief Validity checker that rejects out-of-bounds and colliding configurations.
/// @details All collision checks go through the given SceneContext, so give each planner thread
/// its own context.
class SceneStateValidityChecker : public ompl::base::StateValidityChecker {
public:
  SceneStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                            std::shared_ptr<SceneContext> context);

  bool isValid(const ompl::base::State* state) const override;

  /// @brief The number of states checked since construction.
  size_t getNumChecks() const { return num_checks_; }

private:
  std::shared_ptr<SceneContext> context_;
  const SceneStateSpace* space_;
  mutable size_t num_checks_ = 0;
};

/// @brief Motion validator that collision checks straight-line edges at a fixed configuration-space
/// step size, with the same semantics as the roboplan RRT planner.
class SceneMotionValidator : public ompl::base::MotionValidator {
public:
  /// @param step_size The configuration-space step between checked interior states.
  /// @param bisection If true, visits the interior states coarse-to-fine instead of linearly.
  SceneMotionValidator(const ompl::base::SpaceInformationPtr& si,
                       std::shared_ptr<SceneContext> context, double step_size, bool bisection);

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2,
                   std::pair<ompl::base::State*, double>& last_valid) const override;

private:
  std::shared_ptr<SceneContext> context_;
  const SceneStateSpace* space_;
  double step_size_;
  bool bisection_;
};

/// @brief Builds a ready-to-use OMPL SpaceInformation over a scene state space, with the validity
/// checker and motion validator above installed. Use this to run any OMPL planner directly.
ompl::base::SpaceInformationPtr makeSpaceInformation(const std::shared_ptr<SceneStateSpace>& space,
                                                     const std::shared_ptr<SceneContext>& context,
                                                     double collision_check_step_size,
                                                     bool collision_check_use_bisection);

}  // namespace roboplan
