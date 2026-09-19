#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <pinocchio/multibody/model.hpp>

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

/// @brief Creates the default joint group containing the entire model.
std::unordered_map<std::string, JointGroupInfo>
createDefaultJointGroupInfo(const pinocchio::Model& model);

/// @brief Collects joint names from a kinematic chain between two links.
/// @param model The Pinocchio model.
/// @param base_link The name of the chain's base link.
/// @param tip_link The name of the chain's tip link.
/// @return Joint names from base to tip, excluding the base link's parent joint, if successful.
tl::expected<std::vector<std::string>, std::string>
jointNamesFromChain(const pinocchio::Model& model, const std::string& base_link,
                    const std::string& tip_link);

/// @brief Builds joint group metadata from a list of joints.
/// @param model The Pinocchio model.
/// @param joint_names The joints that make up the group, in user-specified order.
/// @param extra_link_names Additional link names to include besides those driven by the joints.
/// @return The group info if successful, else a string describing the error.
tl::expected<JointGroupInfo, std::string>
makeJointGroupInfo(const pinocchio::Model& model, const std::vector<std::string>& joint_names,
                   const std::vector<std::string>& extra_link_names = {});

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

/// @brief Runs an all-pairs collision check into the given scratch, printing every colliding pair.
/// @details This deliberately skips the broadphase fast path that stops at the first collision.
/// @param model The Pinocchio model.
/// @param data The Pinocchio data to write.
/// @param collision_model The Pinocchio collision (geometry) model.
/// @param geom_data The Pinocchio collision (geometry) data to write.
/// @param q The joint configuration at which to check collisions.
/// @return True if there are collisions, else false.
bool computeCollisionsVerbose(const pinocchio::Model& model, pinocchio::Data& data,
                              const pinocchio::GeometryModel& collision_model,
                              pinocchio::GeometryData& geom_data, const Eigen::VectorXd& q);

/// @brief Returns whether the given velocity-space DOF index of a joint is free-rotating.
/// @details These are the unbounded orientation DOFs for which position limits are meaningless:
/// the single DOF of a continuous joint, the rotational DOF of a planar joint, and the three
/// rotational DOFs of a floating joint. Position limit indices follow the velocity (tangent)
/// space, so a continuous DOF collapses to a single index here.
bool isFreeRotatingDof(JointType type, int dof);

/// @brief Maps an infinite limit to the finite sentinel used to denote "unbounded".
/// @details JointInfo represents an unbounded limit as
/// std::numeric_limits<double>::lowest() / max() (see the JointInfo constructor), not as
/// +/-infinity. A user-supplied '.inf' / '-.inf' is normalized to these sentinels so that an
/// overridden unbounded limit is represented identically to the default unbounded limit.
double sanitizeLimit(double value);

}  // namespace roboplan
