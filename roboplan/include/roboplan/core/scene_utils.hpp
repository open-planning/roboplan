#pragma once

#include <map>
#include <string>

#include <pinocchio/multibody/model.hpp>

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
std::map<std::string, JointGroupInfo> createJointGroupInfo(const pinocchio::Model& model,
                                                           const std::string& srdf);

}  // namespace roboplan
