#include <array>
#include <cassert>
#include <cmath>
#include <numbers>
#include <unordered_set>
#include <utility>

#include <roboplan_mujoco/mujoco_joint_map.hpp>

namespace roboplan {
namespace {

/// @brief Selects the periodic angle equivalent closest to a reference angle.
/// @details MuJoCo stores hinge angles as scalars, while RoboPlan stores continuous
/// rotations as cosine-sine pairs without a winding count. Keeping the current MuJoCo
/// angle as the reference prevents command discontinuities at +/- pi.
double nearestEquivalentAngle(const double angle, const double reference) {
  constexpr double two_pi = 2.0 * std::numbers::pi;
  return angle + two_pi * std::round((reference - angle) / two_pi);
}

/// @brief Finds and validates the single position servo attached directly to a joint.
tl::expected<int, std::string> findPositionActuator(const mjModel& model, const int joint_id, const std::string& joint_name) {
  int actuator_id = -1;
  for (int i = 0; i < model.nu; ++i) {
    if (model.actuator_trntype[i] != mjTRN_JOINT || model.actuator_trnid[2 * i] != joint_id) {
      continue;
    }
    if (actuator_id >= 0) {
      return tl::unexpected("Mujoco joint has multiple direct actuators: " + joint_name);
    }
    actuator_id = i;
  }
  if (actuator_id < 0) {
    return tl::unexpected("Mujoco joint has no direct actuator: " + joint_name);
  }
  // to verify this actuator behaves like a position actuator we can check the bias force and the
  // (stiffness kp) mujoco actuator force: f = gain * control + bias[0] + bias[1] x
  // actuator_position + bias[2] x actuator_velocity a position servo with stiffness kp is compiled
  // as: gain = kp bias[1] = -kp force = kp * (control - position) -kv * velocity
  constexpr double tolerance = 1e-9;
  const double gain = model.actuator_gainprm[actuator_id * mjNGAIN];
  const double position_bias = model.actuator_biasprm[actuator_id * mjNBIAS + 1];
  const bool is_position_servo = model.actuator_gaintype[actuator_id] == mjGAIN_FIXED && model.actuator_biastype[actuator_id] == mjBIAS_AFFINE && gain > 0.0 && std::abs(position_bias + gain) <= tolerance;
  const bool unit_joint_transmission = std::abs(model.actuator_gear[actuator_id * 6] - 1.0) <= tolerance;
  if (!is_position_servo || !unit_joint_transmission) {
    return tl::unexpected("Mujoco joint actuator is not a unit gear position servo: " + joint_name);
  }
  return actuator_id;
}
}  // namespace

MujocoJointMap::MujocoJointMap(const int roboplan_nq, const int roboplan_nv, const int mujoco_nq, Eigen::VectorXd reference_q, const std::vector<JointBinding> bindings) 
      : roboplan_nq_(roboplan_nq), roboplan_nv_(roboplan_nv), mujoco_nq_(mujoco_nq), reference_q_(std::move(reference_q)), bindings_(std::move(bindings)) {}

tl::expected<MujocoJointMap, std::string> MujocoJointMap::create(const Scene& scene, const mjModel& model, const std::vector<std::string>& mapped_joint_names, const bool required_position_actuators) {
  if (mapped_joint_names.empty()) {
    return tl::unexpected("At least one joint name must be provided for mapping to Mujoco joint representation");
  }
  const auto& pinocchio_model = scene.getModel();

  std::vector<JointBinding> bindings;
  bindings.reserve(mapped_joint_names.size());
  std::unordered_set<std::string> mapped_names;
  for (const auto& joint_name : mapped_joint_names) {
    if (!pinocchio_model.existJointName(joint_name)) {
      return tl::unexpected("Joint not found in RoboPlan model: " + joint_name);
    }
    if (!mapped_names.insert(joint_name).second) {
      return tl::unexpected("RoboPlan robot model has multiple joints with the same name: " + joint_name);
    }
    const auto pinocchio_joint_id = pinocchio_model.getJointId(joint_name);
    const auto& pinocchio_joint = pinocchio_model.joints[pinocchio_joint_id];
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::unexpected(joint_info.error());
    }

    if (joint_info->mimic_info) {
      return tl::unexpected("Mimic joints cannot be mapped independently: " + joint_name);
    }

    Encoding encoding;
    if (joint_info->type == JointType::CONTINUOUS) {
      encoding = Encoding::CONTINUOUS;
    } else if (joint_info->type == JointType::REVOLUTE || joint_info->type == JointType::PRISMATIC) {
      encoding = Encoding::SCALAR;
    } else if (joint_info->type == JointType::PLANAR) {
      encoding = Encoding::PLANAR;
    } else {
      return tl::unexpected("Unsupported joint type: " + joint_name);
    }

    const auto make_coordinate = [&](const std::string& mujoco_joint_name) -> tl::expected<CoordinateBinding, std::string> {
      const int mujoco_joint_id = mj_name2id(&model, mjOBJ_JOINT, mujoco_joint_name.c_str());
      if (mujoco_joint_id < 0) {
        return tl::unexpected("Mujoco joint does not exist: " + mujoco_joint_name);
      }

      int actuator_id = -1;
      if (required_position_actuators) {
        const auto actuator = findPositionActuator(model, mujoco_joint_id, joint_name);
        if (!actuator) {
          return tl::unexpected(actuator.error());
        }
        actuator_id = actuator.value();
      }

      return CoordinateBinding{
          .mujoco_qpos_index = model.jnt_qposadr[mujoco_joint_id],
          .mujoco_dof_index = model.jnt_dofadr[mujoco_joint_id],
          .mujoco_actuator_index = actuator_id,
      };
    };

    std::vector<CoordinateBinding> coordinates;
    if (encoding == Encoding::PLANAR) {
      // MuJoCo has no planar joint primitive matching Pinocchio's SE(2)
      // representation, so prepared URDFs split it into canonical x, y, and yaw joints.
      constexpr std::array<const char*, 3> suffixes{"_TX", "_TY", "_RZ"};
      constexpr std::array<int, 3> expected_types{mjJNT_SLIDE, mjJNT_SLIDE, mjJNT_HINGE};
      constexpr std::array<std::array<double, 3>, 3> expected_axes{{
          {1.0, 0.0, 0.0},
          {0.0, 1.0, 0.0},
          {0.0, 0.0, 1.0},
      }};
      int body_id = -1;
      for (std::size_t i = 0; i < suffixes.size(); ++i) {
        const std::string split_name = joint_name + suffixes[i];
        const int mujoco_joint_id = mj_name2id(&model, mjOBJ_JOINT, split_name.c_str());
        if (mujoco_joint_id < 0) {
          return tl::unexpected("Mujoco joint does not exist: " + split_name);
        }
        if (model.jnt_type[mujoco_joint_id] != expected_types[i]) {
          return tl::unexpected("Mujoco Planar joint has the wrong type: " + split_name);
        }
        if (body_id < 0) {
          body_id = model.jnt_bodyid[mujoco_joint_id];
        } else if (model.jnt_bodyid[mujoco_joint_id] != body_id) {
          return tl::unexpected("Mujoco Planar joints do not share one body: " + joint_name);
        }
        for (int axis = 0; axis < 3; ++axis) {
          if (std::abs(model.jnt_axis[3 * mujoco_joint_id + axis] - expected_axes[i][axis]) > 1e-4) {
            return tl::unexpected("Mujoco Planar joint has a noncanonical axis: " + split_name);
          }
        }
        auto coordinate = make_coordinate(split_name);
        if (!coordinate) {
          return tl::unexpected(coordinate.error());
        }
        coordinates.push_back(coordinate.value());
      }
    } else {
      const int mujoco_joint_id = mj_name2id(&model, mjOBJ_JOINT, joint_name.c_str());
      if (mujoco_joint_id < 0) {
        return tl::unexpected("Mujoco joint does not exist: " + joint_name);
      }
      const int expected_type = joint_info->type == JointType::PRISMATIC ? mjJNT_SLIDE : mjJNT_HINGE;
      if (model.jnt_type[mujoco_joint_id] != expected_type) {
        return tl::unexpected("Mujoco joint has the wrong type: " + joint_name);
      }
      auto coordinate = make_coordinate(joint_name);
      if (!coordinate) {
        return tl::unexpected(coordinate.error());
      }
      coordinates.push_back(coordinate.value());
    }

    JointBinding binding{
        .roboplan_q_index = pinocchio_joint.idx_q(),
        .roboplan_v_index = pinocchio_joint.idx_v(),
        .encoding = encoding,
        .coordinates = std::move(coordinates),
        .mimics = {},
    };

    if (encoding == Encoding::PLANAR) {
      bindings.push_back(std::move(binding));
      continue;
    }

    // Mimic joints are represented explicitly in MuJoCo qpos. Record their affine
    // relationship so direct configuration writes remain consistent with RoboPlan.
    for (const auto& candidate_name : scene.getJointNamesWithMimics()) {
      const auto candidate_info = scene.getJointInfo(candidate_name);
      if (!candidate_info || !candidate_info->mimic_info || candidate_info->mimic_info->mimicked_joint_name != joint_name) {
        continue;
      }
      const int mujoco_mimic_joint_id = mj_name2id(&model, mjOBJ_JOINT, candidate_name.c_str());
      if (mujoco_mimic_joint_id < 0) {
        return tl::unexpected("Mujoco mimic joint does not exist: " + candidate_name);
      }
      binding.mimics.push_back({
          .mujoco_qpos_index = model.jnt_qposadr[mujoco_mimic_joint_id],
          .mujoco_dof_index = model.jnt_dofadr[mujoco_mimic_joint_id],
          .scaling = candidate_info->mimic_info->scaling,
          .offset = candidate_info->mimic_info->offset,
      });
    }
    bindings.push_back(std::move(binding));
  }

  return MujocoJointMap(pinocchio_model.nq, pinocchio_model.nv, model.nq, scene.getCurrentJointPositions(), std::move(bindings));
}

tl::expected<RoboPlanRobotState, std::string> MujocoJointMap::readState(const mjData& data) const {
  RoboPlanRobotState state;
  state.time = data.time;
  state.q = reference_q_;
  state.v = Eigen::VectorXd::Zero(roboplan_nv_);
  state.tau = Eigen::VectorXd::Zero(roboplan_nv_);

  for (const auto& binding : bindings_) {
    if (binding.encoding == Encoding::PLANAR) {
      const double angle = data.qpos[binding.coordinates[2].mujoco_qpos_index];
      const double cosine = std::cos(angle);
      const double sine = std::sin(angle);
      state.q[binding.roboplan_q_index] = data.qpos[binding.coordinates[0].mujoco_qpos_index];
      state.q[binding.roboplan_q_index + 1] = data.qpos[binding.coordinates[1].mujoco_qpos_index];
      state.q[binding.roboplan_q_index + 2] = cosine;
      state.q[binding.roboplan_q_index + 3] = sine;
      const double world_vx = data.qvel[binding.coordinates[0].mujoco_dof_index];
      const double world_vy = data.qvel[binding.coordinates[1].mujoco_dof_index];
      // MuJoCo's split translations are world aligned. Pinocchio expresses the planar
      // tangent translation in the moving joint frame, so rotate velocity and force
      // components back by the current yaw.
      state.v[binding.roboplan_v_index] = cosine * world_vx + sine * world_vy;
      state.v[binding.roboplan_v_index + 1] = -sine * world_vx + cosine * world_vy;
      state.v[binding.roboplan_v_index + 2] = data.qvel[binding.coordinates[2].mujoco_dof_index];
      const double world_fx = data.qfrc_actuator[binding.coordinates[0].mujoco_dof_index];
      const double world_fy = data.qfrc_actuator[binding.coordinates[1].mujoco_dof_index];
      state.tau[binding.roboplan_v_index] = cosine * world_fx + sine * world_fy;
      state.tau[binding.roboplan_v_index + 1] = -sine * world_fx + cosine * world_fy;
      state.tau[binding.roboplan_v_index + 2] = data.qfrc_actuator[binding.coordinates[2].mujoco_dof_index];
      continue;
    }
    const auto& coordinate = binding.coordinates.front();
    if (const auto result = scalarPositionToRoboPlan(binding, data.qpos[coordinate.mujoco_qpos_index], state.q); !result) {
      return tl::unexpected(result.error());
    }
    state.v[binding.roboplan_v_index] = data.qvel[coordinate.mujoco_dof_index];
    state.tau[binding.roboplan_v_index] = data.qfrc_actuator[coordinate.mujoco_dof_index];
  }
  return state;
}

tl::expected<Eigen::VectorXd, std::string> MujocoJointMap::toMujocoQpos(const Eigen::VectorXd& q, const mjData& data) const {
  if (q.size() != roboplan_nq_) {
    return tl::unexpected("Visualization configuration must have the size model.nq");
  }
  if (!q.allFinite()) {
    return tl::unexpected("Visualization configuration must contain finite values");
  }
  for (const auto& binding : bindings_) {
    if (binding.encoding == Encoding::PLANAR && !planarAngleFromRoboPlan(binding, q)) {
      return tl::unexpected("Planar orientation must have a nonzero cosine-sine norm");
    }
    if (binding.encoding == Encoding::CONTINUOUS && !scalarPositionFromRoboPlan(binding, q)) {
      return tl::unexpected("Continuous orientation must have a non zero cosine-sine norm");
    }
  }
  // Preserve every MuJoCo coordinate not selected for mapping.
  Eigen::VectorXd qpos = Eigen::Map<const Eigen::VectorXd>(data.qpos, mujoco_nq_);
  for (const auto& binding : bindings_) {
    if (binding.encoding == Encoding::PLANAR) {
      qpos[binding.coordinates[0].mujoco_qpos_index] = q[binding.roboplan_q_index];
      qpos[binding.coordinates[1].mujoco_qpos_index] = q[binding.roboplan_q_index + 1];
      qpos[binding.coordinates[2].mujoco_qpos_index] = planarAngleFromRoboPlan(binding, q).value();
      continue;
    }
    const double position = scalarPositionFromRoboPlan(binding, q).value();
    qpos[binding.coordinates.front().mujoco_qpos_index] = position;
    for (const auto& mimic : binding.mimics) {
      qpos[mimic.mujoco_qpos_index] = mimic.scaling * position + mimic.offset;
    }
  }
  return qpos;
}

tl::expected<void, std::string> MujocoJointMap::writePositionCommand(const Eigen::VectorXd& q, mjData& data) const {
  if (q.size() != roboplan_nq_) {
    return tl::unexpected("Position command must have size model.nq");
  }
  if (!q.allFinite()) {
    return tl::unexpected("Position command must contain finite values");
  }
  for (const auto& binding : bindings_) {
    if (binding.encoding == Encoding::PLANAR && !planarAngleFromRoboPlan(binding, q)) {
      return tl::unexpected("Planar orientation must have a nonzero cosine-sine norm");
    }
    if (binding.encoding == Encoding::CONTINUOUS && !scalarPositionFromRoboPlan(binding, q)) {
      return tl::unexpected("Continuous orientation must have a non zero cosine-sine norm");
    }
  }
  for (const auto& binding : bindings_) {
    if (binding.encoding == Encoding::PLANAR) {
      data.ctrl[binding.coordinates[0].mujoco_actuator_index] = q[binding.roboplan_q_index];
      data.ctrl[binding.coordinates[1].mujoco_actuator_index] = q[binding.roboplan_q_index + 1];
      const double yaw_angle = planarAngleFromRoboPlan(binding, q).value();
      const auto& yaw_coordinate = binding.coordinates[2];
      data.ctrl[yaw_coordinate.mujoco_actuator_index] = nearestEquivalentAngle(yaw_angle, data.qpos[yaw_coordinate.mujoco_qpos_index]);
      continue;
    }
    const auto& coordinate = binding.coordinates.front();
    double position = scalarPositionFromRoboPlan(binding, q).value();
    if (binding.encoding == Encoding::CONTINUOUS) {
      position = nearestEquivalentAngle(position, data.qpos[coordinate.mujoco_qpos_index]);
    }
    data.ctrl[coordinate.mujoco_actuator_index] = position;
  }
  return {};
}

tl::expected<void, std::string> MujocoJointMap::setConfiguration(const Eigen::VectorXd& q, mjData& data) {
  if (q.size() != roboplan_nq_) {
    return tl::unexpected("Reset configuration must have size model.nq");
  }
  if (!q.allFinite()) {
    return tl::unexpected("Configuration must contain finite values");
  }
  for (const auto& binding : bindings_) {
    if (binding.encoding == Encoding::PLANAR && !planarAngleFromRoboPlan(binding, q)) {
      return tl::unexpected("Planar orientation must have a nonzero cosine-sine norm");
    }
    if (binding.encoding == Encoding::CONTINUOUS && !scalarPositionFromRoboPlan(binding, q)) {
      return tl::unexpected("Continuous orientation must have a non zero cosine-sine norm");
    }
  }
  reference_q_ = q;
  for (const auto& binding : bindings_) {
    if (binding.encoding == Encoding::PLANAR) {
      data.qpos[binding.coordinates[0].mujoco_qpos_index] = q[binding.roboplan_q_index];
      data.qpos[binding.coordinates[1].mujoco_qpos_index] = q[binding.roboplan_q_index + 1];
      data.qpos[binding.coordinates[2].mujoco_qpos_index] = planarAngleFromRoboPlan(binding, q).value();
      for (const auto& coordinate : binding.coordinates) {
        data.qvel[coordinate.mujoco_dof_index] = 0.0;
      }
      continue;
    }
    const double position = scalarPositionFromRoboPlan(binding, q).value();
    const auto& coordinate = binding.coordinates.front();
    data.qpos[coordinate.mujoco_qpos_index] = position;
    data.qvel[coordinate.mujoco_dof_index] = 0.0;
    for (const auto& mimic : binding.mimics) {
      data.qpos[mimic.mujoco_qpos_index] = mimic.scaling * position + mimic.offset;
      data.qvel[mimic.mujoco_dof_index] = 0.0;
    }
  }
  return {};
}

tl::expected<double, std::string> MujocoJointMap::scalarPositionFromRoboPlan(const JointBinding& binding, const Eigen::VectorXd& q) const {
  if (binding.encoding == Encoding::CONTINUOUS) {
    const double cosine = q[binding.roboplan_q_index];
    const double sine = q[binding.roboplan_q_index + 1];
    if (std::hypot(cosine, sine) <= 1e-12) {
      return tl::unexpected("Continuous orientation has zero norm");
    }
    return std::atan2(sine, cosine);
  }
  return q[binding.roboplan_q_index];
}

tl::expected<void, std::string> MujocoJointMap::scalarPositionToRoboPlan(const JointBinding& binding, const double value, Eigen::VectorXd& q) const {
  if (binding.encoding == Encoding::CONTINUOUS) {
    q[binding.roboplan_q_index] = std::cos(value);
    q[binding.roboplan_q_index + 1] = std::sin(value);
    return {};
  }
  q[binding.roboplan_q_index] = value;
  return {};
}

tl::expected<double, std::string> MujocoJointMap::planarAngleFromRoboPlan(const JointBinding& binding, const Eigen::VectorXd& q) const {
  const double cosine = q[binding.roboplan_q_index + 2];
  const double sine = q[binding.roboplan_q_index + 3];
  if (std::hypot(cosine, sine) <= 1e-12) {
    return tl::unexpected("Planar orientation has zero norm");
  }
  return std::atan2(sine, cosine);
}
}  // namespace roboplan