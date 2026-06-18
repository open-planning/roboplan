#pragma once

#include <map>
#include <optional>
#include <string>
#include <unordered_map>

#include <pinocchio/multibody/model.hpp>
#include <yaml-cpp/yaml.h>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

/// @brief Map from Pinocchio joint model short names to RoboPlan joint type enums.
const std::map<std::string, roboplan::JointType> kPinocchioJointTypeMap = {
    {"JointModelPrismaticUnaligned", JointType::PRISMATIC},
    {"JointModelPX", roboplan::JointType::PRISMATIC},
    {"JointModelPY", roboplan::JointType::PRISMATIC},
    {"JointModelPZ", roboplan::JointType::PRISMATIC},
    {"JointModelRX", roboplan::JointType::REVOLUTE},
    {"JointModelRY", roboplan::JointType::REVOLUTE},
    {"JointModelRZ", roboplan::JointType::REVOLUTE},
    {"JointModelRevoluteUnaligned", roboplan::JointType::REVOLUTE},
    {"JointModelRUBX", roboplan::JointType::CONTINUOUS},
    {"JointModelRUBY", roboplan::JointType::CONTINUOUS},
    {"JointModelRUBZ", roboplan::JointType::CONTINUOUS},
    {"JointModelRevoluteUnboundedUnaligned", roboplan::JointType::CONTINUOUS},
    {"JointModelPlanar", roboplan::JointType::PLANAR},
    {"JointModelFreeFlyer", roboplan::JointType::FLOATING},
    {"JointModelMimic", roboplan::JointType::UNKNOWN},
};

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

/// @brief Builds joint positions for all joints in getJointNamesWithMimics() order.
/// @details Non-mimic joints copy their Pinocchio q block; mimic joints use the mimic law.
/// @param scene The scene from which to look up joint information.
/// @param q The Pinocchio configuration vector (model.nq).
/// @return Position vector aligned with getJointNamesWithMimics().
Eigen::VectorXd jointPositionsWithMimicsFromPinocchio(const Scene& scene, const Eigen::VectorXd& q);

/// @brief Holds extended joint limits (acceleration, jerk) parsed from a URDF <limit> tag.
/// @details This is a temporary holdover until Pinocchio properly supports URDF 1.2 extended
/// limits in its own parsers. See https://github.com/stack-of-tasks/pinocchio/issues/2893
struct UrdfExtendedJointLimits {
  std::optional<double> acceleration;
  std::optional<double> jerk;
};

/// @brief Parses extended joint limits (acceleration, jerk) from URDF <limit> tags.
/// @details Reads acceleration and jerk attributes if present, regardless of URDF version.
/// Returns an empty map only if parsing fails.
/// This is a temporary holdover until Pinocchio properly supports URDF 1.2 extended limits.
/// See https://github.com/stack-of-tasks/pinocchio/issues/2893
/// @param urdf The URDF XML string.
/// @return A map from joint name to its extended limits.
std::unordered_map<std::string, UrdfExtendedJointLimits>
parseUrdfExtendedJointLimits(const std::string& urdf);

/// @brief Overrides a joint's limits in-place from a YAML configuration.
/// @details Position, velocity, acceleration, and jerk limits may each be overridden via a
/// `joint_limits/<joint_name>` entry, where every limit is a sequence sized to the joint's number
/// of velocity DOFs. When no override is present, velocity limits fall back to the URDF values from
/// the model and acceleration/jerk limits fall back to the extended URDF limits. Position limits
/// for free-rotating DOFs (continuous joints and the orientation DOFs of planar/floating joints)
/// are meaningless and are discarded with a warning unless given as '.inf' / '-.inf'.
/// @param model The Pinocchio model, used for URDF-derived velocity limits.
/// @param yaml_config The parsed YAML configuration node (may be empty/null).
/// @param urdf_extended_limits Extended (acceleration, jerk) limits parsed from the URDF.
/// @param joint_name The name of the joint to override.
/// @param info The joint info to modify in-place.
void overrideJointLimitsFromYaml(
    const pinocchio::Model& model, const YAML::Node& yaml_config,
    const std::unordered_map<std::string, UrdfExtendedJointLimits>& urdf_extended_limits,
    const std::string& joint_name, JointInfo& info);

}  // namespace roboplan
