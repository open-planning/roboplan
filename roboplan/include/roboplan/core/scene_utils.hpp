#pragma once

#include <map>
#include <string>

#include <pinocchio/multibody/model.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

/// @brief Creates a map of the robot's frame names to IDs.
/// @param model The Pinocchio model.
/// @return The map of robot frame names to IDs.
std::unordered_map<std::string, pinocchio::FrameIndex>
createFrameMap(const pinocchio::Model& model);

/// @brief Creates the joint group information for the scene;
/// @param model The Pinocchio model.
/// @param srdf_stream The SRDF file contents.
/// @return The map of robot joint group names to group info.
std::unordered_map<std::string, JointGroupInfo> createJointGroupInfo(const pinocchio::Model& model,
                                                                     const std::string& srdf);

/// @brief Collapses a joint position vector's continuous joints for downstream algorithms.
/// @details That is, positions that are expressed as [cos(theta), sin(theta)] will be collapsed
/// to [theta], assuming being between +/- pi.
/// @param scene The scene from which to look up joint information.
/// @param group_name The name of the joint group corresponding to the position vector.
/// @param q_orig The original position vectors.
/// @return The collapsed position vectors if successful, else a string describing the error.
tl::expected<Eigen::VectorXd, std::string>
collapseContinuousJointPositions(const Scene& scene, const std::string& group_name,
                                 const Eigen::VectorXd& q_orig);

/// @brief Expands a joint position vector's continuous joints from downstream algorithms.
/// @details That is, positions that are expressed as [theta] will be expanded to
/// [cos(theta), sin(theta)].
/// @param scene The scene from which to look up joint information.
/// @param group_name The name of the joint group corresponding to the position vector.
/// @param q_orig The original position vectors.
/// @return The expanded position vectors if successful, else a string describing the error.
tl::expected<Eigen::VectorXd, std::string>
expandContinuousJointPositions(const Scene& scene, const std::string& group_name,
                               const Eigen::VectorXd& q_orig);

}  // namespace roboplan
