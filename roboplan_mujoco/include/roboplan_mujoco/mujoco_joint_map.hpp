#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>
#include <tl/expected.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_hardware_interface/hardware_interface.hpp>

namespace roboplan {
class MujocoHardwareInterface;
class MujocoKinematicInterface;

/// @brief Precomputed mapping between RoboPlan/Pinocchio and MuJoCo joint representations.
/// @details Scalar joints map to one MuJoCo hinge or slide joint. A continuous joint maps
/// RoboPlan `[cos(theta), sin(theta)]` to one MuJoCo hinge angle. A planar joint maps RoboPlan
/// `[x, y, cos(yaw), sin(yaw)]` to three MuJoCo joints named with `_TX`, `_TY`, and `_RZ`
/// suffixes. The map also propagates scalar mimic joint positions.
class MujocoJointMap {
private:
  friend class MujocoHardwareInterface;
  friend class MujocoKinematicInterface;

  /// @brief Builds and validates a mapping for selected RoboPlan joints.
  /// @param scene RoboPlan scene that supplies joint metadata and vector indices.
  /// @param model Compiled MuJoCo model containing corresponding joints.
  /// @param mapped_joint_names RoboPlan joints to include in the mapping.
  /// @param required_position_actuators Whether every mapped MuJoCo coordinate must have a compatible unit gear position servo.
  /// @return A validated map, or a string describing a missing or incompatible joint.
  static tl::expected<MujocoJointMap, std::string> create(const Scene& scene, const mjModel& model, const std::vector<std::string>& mapped_joint_names, bool required_position_actuators = true);

  /// @brief Reads a full RoboPlan state from MuJoCo data.
  /// @details Positions of unmapped joints retain reference_q_. Their velocity and effort
  /// entries are zero because they are not measured by this map.
  /// @param data MuJoCo state to read.
  /// @return The mapped state, or a string describing a conversion error.
  tl::expected<RoboPlanRobotState, std::string> readState(const mjData& data) const;

  /// @brief Converts a full RoboPlan configuration to a full MuJoCo `qpos` vector.
  /// @details Unmapped MuJoCo entries are copied from `data.qpos`.
  /// @param q Full RoboPlan configuration.
  /// @param data MuJoCo state supplying unmapped `qpos` entries.
  /// @return The converted vector, or a string describing invalid input.
  tl::expected<Eigen::VectorXd, std::string> toMujocoQpos(const Eigen::VectorXd& q, const mjData& data) const;

  /// @brief Writes mapped position targets to MuJoCo actuators.
  /// @param q Full RoboPlan configuration containing the requested targets.
  /// @param data MuJoCo data whose `ctrl` array will be updated.
  /// @return Success, or a string describing invalid input.
  tl::expected<void, std::string> writePositionCommand(const Eigen::VectorXd& q, mjData& data) const;

  /// @brief Writes mapped positions directly into MuJoCo state.
  /// @details Updates reference_q_, writes mapped and mimic `qpos` entries, and zeros
  /// their velocities. It does not call `mj_forward`.
  /// @param q Full RoboPlan configuration to apply.
  /// @param data MuJoCo state to modify.
  /// @return Success, or a string describing invalid input.
  tl::expected<void, std::string> setConfiguration(const Eigen::VectorXd& q, mjData& data);

  /// @brief Encoding used by RoboPlan for a mapped joint.
  enum class Encoding {
    /// @brief One position and one tangent space coordinate.
    SCALAR,

    /// @brief Cosine,sine position pair and one tangent space coordinate.
    CONTINUOUS,

    /// @brief SE(2) position represented by x, y, cosine, and sine.
    PLANAR,
  };

  /// @brief MuJoCo indices associated with one scalar joint coordinate.
  struct CoordinateBinding {
    /// @brief Index in `mjData::qpos`.
    int mujoco_qpos_index;

    /// @brief Index in `mjData::qvel` and `mjData::qfrc_actuator`.
    int mujoco_dof_index;

    /// @brief Index in `mjData::ctrl`, or -1 for a kinematic only map.
    int mujoco_actuator_index;
  };

  /// @brief MuJoCo state indices and affine relation for a scalar mimic joint.
  struct MimicBinding {
    /// @brief Index in `mjData::qpos`.
    int mujoco_qpos_index;

    /// @brief Index in `mjData::qvel`.
    int mujoco_dof_index;

    /// @brief Multiplier applied to the source joint position.
    double scaling;

    /// @brief Offset added after scaling the source joint position.
    double offset;
  };

  /// @brief Complete representation mapping for one RoboPlan joint.
  struct JointBinding {
    /// @brief First position index in the full RoboPlan configuration.
    int roboplan_q_index;

    /// @brief First velocity index in the full RoboPlan tangent vector.
    int roboplan_v_index;

    /// @brief RoboPlan position encoding used by this joint.
    Encoding encoding;

    /// @brief MuJoCo scalar coordinates; one normally, three for planar joints.
    std::vector<CoordinateBinding> coordinates;

    /// @brief Scalar mimic joints driven by this joint.
    std::vector<MimicBinding> mimics;
  };

  /// @brief Constructs a map from validated dimensions and bindings.
  MujocoJointMap(int roboplan_nq, int roboplan_nv, int mujoco_nq, Eigen::VectorXd reference_q, std::vector<JointBinding> bindings);

  /// @brief Decodes a scalar or continuous RoboPlan position.
  tl::expected<double, std::string> scalarPositionFromRoboPlan(const JointBinding& binding, const Eigen::VectorXd& q) const;

  /// @brief Encodes a MuJoCo scalar position into a RoboPlan configuration.
  tl::expected<void, std::string> scalarPositionToRoboPlan(const JointBinding& binding, double value, Eigen::VectorXd& q) const;

  /// @brief Decodes a planar yaw angle from its RoboPlan cosine,sine pair.
  tl::expected<double, std::string> planarAngleFromRoboPlan(const JointBinding& binding, const Eigen::VectorXd& q) const;

  /// @brief Size of a full RoboPlan configuration vector.
  int roboplan_nq_;

  /// @brief Size of a full RoboPlan tangent vector.
  int roboplan_nv_;

  /// @brief Size of a full MuJoCo `qpos` vector.
  int mujoco_nq_;

  /// @brief Full RoboPlan configuration used for unmapped positions.
  Eigen::VectorXd reference_q_;

  /// @brief Validated per-joint representation mappings.
  std::vector<JointBinding> bindings_;
};
}  // namespace roboplan