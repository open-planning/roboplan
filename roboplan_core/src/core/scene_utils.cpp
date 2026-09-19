#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include <pinocchio/collision/collision.hpp>

#include <roboplan/core/scene_utils.hpp>

namespace roboplan {

bool isFreeRotatingDof(JointType type, int dof) {
  switch (type) {
  case JointType::CONTINUOUS:
    return dof == 0;
  case JointType::PLANAR:
    return dof == 2;  // (x, y, theta) -> theta is free-rotating.
  case JointType::FLOATING:
    return dof >= 3;  // (x, y, z, rx, ry, rz) -> the rotational DOFs are free-rotating.
  default:
    return false;
  }
}

double sanitizeLimit(double value) {
  if (std::isinf(value)) {
    return value > 0.0 ? std::numeric_limits<double>::max() : std::numeric_limits<double>::lowest();
  }
  return value;
}

std::unordered_map<std::string, pinocchio::FrameIndex>
createFrameMap(const pinocchio::Model& model) {
  std::unordered_map<std::string, pinocchio::FrameIndex> frame_map;
  for (const auto& frame : model.frames) {
    auto it = frame_map.find(frame.name);
    if (it != frame_map.end()) {
      throw std::runtime_error(
          "Frame name '" + frame.name +
          "' was already added to the map. Duplicate names for different frame types (body, joint, "
          "sensor, etc.) are not supported in RoboPlan.");
    }
    frame_map[frame.name] = model.getFrameId(frame.name, frame.type);
  }
  return frame_map;
}

tl::expected<std::vector<std::string>, std::string>
jointNamesFromChain(const pinocchio::Model& model, const std::string& base_link,
                    const std::string& tip_link) {
  const auto tip_frame_id = model.getFrameId(tip_link);
  if (tip_frame_id >= static_cast<size_t>(model.nframes)) {
    return tl::make_unexpected("Tip link '" + tip_link + "' not found in the model.");
  }
  const auto base_frame_id = model.getFrameId(base_link);
  if (base_frame_id >= static_cast<size_t>(model.nframes)) {
    return tl::make_unexpected("Base link '" + base_link + "' not found in the model.");
  }

  const auto base_link_parent_joint_id = model.frames.at(base_frame_id).parentJoint;
  std::vector<int> joint_indices;
  auto cur_frame_id = tip_frame_id;
  while (true) {
    const auto& frame = model.frames.at(cur_frame_id);
    const auto parent_joint_id = frame.parentJoint;

    // Sometimes the parent frame of a joint is rigidly attached to the chain's base link,
    // but is not the parent frame itself, so we should check that as well.
    if (parent_joint_id == base_link_parent_joint_id) {
      break;
    }

    const auto& parent_joint_name = model.names.at(parent_joint_id);
    joint_indices.push_back(parent_joint_id);
    cur_frame_id = model.frames.at(model.getFrameId(parent_joint_name)).parentFrame;
    if (cur_frame_id == base_frame_id) {
      break;
    }
    if (cur_frame_id == 0) {
      return tl::make_unexpected("Did not find base link '" + base_link +
                                 "' while walking the chain from tip link '" + tip_link + "'.");
    }
  }

  std::vector<std::string> joint_names;
  joint_names.reserve(joint_indices.size());
  for (auto it = joint_indices.rbegin(); it != joint_indices.rend(); ++it) {
    joint_names.push_back(model.names.at(*it));
  }
  return joint_names;
}

tl::expected<JointGroupInfo, std::string>
makeJointGroupInfo(const pinocchio::Model& model, const std::vector<std::string>& joint_names,
                   const std::vector<std::string>& extra_link_names) {
  std::vector<std::vector<std::string>> bodies_by_joint(static_cast<size_t>(model.njoints));
  for (const auto& frame : model.frames) {
    if (frame.type == pinocchio::BODY) {
      bodies_by_joint.at(frame.parentJoint).push_back(frame.name);
    }
  }

  std::vector<size_t> joint_indices;
  std::unordered_set<std::string> link_name_set(extra_link_names.begin(), extra_link_names.end());
  std::vector<int> q_indices;
  std::vector<int> v_indices;
  size_t num_joints_with_continuous_dofs = 0;
  for (const auto& joint_name : joint_names) {
    const auto joint_id = model.getJointId(joint_name);
    if (joint_id >= static_cast<size_t>(model.njoints)) {
      return tl::make_unexpected("Joint '" + joint_name + "' is not in the model.");
    }
    joint_indices.push_back(joint_id);

    // A single moving joint can support multiple links: the link it actuates plus any links
    // rigidly attached to it through fixed joints, which Pinocchio collapses into the same
    // parent joint.
    for (const auto& link_name : bodies_by_joint.at(joint_id)) {
      link_name_set.insert(link_name);
    }

    const auto& joint = model.joints.at(joint_id);
    const auto& q_idx = model.idx_qs.at(joint_id);
    for (int dof = 0; dof < joint.nq(); ++dof) {
      q_indices.push_back(q_idx + dof);
    }
    const auto& v_idx = model.idx_vs.at(joint_id);
    for (int dof = 0; dof < joint.nv(); ++dof) {
      v_indices.push_back(v_idx + dof);
    }

    auto it = kPinocchioJointTypeMap.find(joint.shortname());
    if (it == kPinocchioJointTypeMap.end()) {
      return tl::make_unexpected("Unsupported Pinocchio joint type: '" + joint.shortname() + "'");
    }
    if (it->second == JointType::CONTINUOUS || it->second == JointType::PLANAR) {
      num_joints_with_continuous_dofs += 1;
    }
  }

  return JointGroupInfo{.joint_names = joint_names,
                        .joint_indices = std::move(joint_indices),
                        .link_names = {link_name_set.begin(), link_name_set.end()},
                        .q_indices = Eigen::VectorXi::Map(
                            q_indices.data(), static_cast<Eigen::Index>(q_indices.size())),
                        .v_indices = Eigen::VectorXi::Map(
                            v_indices.data(), static_cast<Eigen::Index>(v_indices.size())),
                        .has_continuous_dofs = num_joints_with_continuous_dofs > 0,
                        .nq_collapsed = q_indices.size() - num_joints_with_continuous_dofs};
}

std::unordered_map<std::string, JointGroupInfo>
createDefaultJointGroupInfo(const pinocchio::Model& model) {
  std::unordered_map<std::string, JointGroupInfo> joint_group_map;

  std::vector<size_t> all_joint_indices(model.njoints - 1);
  std::iota(all_joint_indices.begin(), all_joint_indices.end(), 0);

  // It is possible for a robot to have continuous joints that are not in any group. So check
  // again to be sure.
  bool default_group_has_continuous_dofs = false;
  size_t default_group_num_continuous_dofs = 0;
  for (size_t jid = 1; jid < static_cast<size_t>(model.njoints); ++jid) {
    const auto& joint = model.joints.at(jid);
    auto it = kPinocchioJointTypeMap.find(joint.shortname());
    if (it == kPinocchioJointTypeMap.end()) {
      throw std::runtime_error("Unsupported Pinocchio joint type: '" + joint.shortname() + "'");
    }
    const auto joint_type = it->second;

    if (joint_type == JointType::CONTINUOUS || joint_type == JointType::PLANAR) {
      default_group_has_continuous_dofs = true;
      default_group_num_continuous_dofs += 1;
    }
  }

  // The default group contains every link (body frame) in the model.
  std::vector<std::string> all_link_names;
  all_link_names.reserve(model.nframes);
  for (const auto& frame : model.frames) {
    if (frame.type == pinocchio::BODY) {
      all_link_names.push_back(frame.name);
    }
  }

  joint_group_map[""] = JointGroupInfo{
      .joint_names = std::vector<std::string>(model.names.begin() + 1, model.names.end()),
      .joint_indices = std::move(all_joint_indices),
      .link_names = std::move(all_link_names),
      .q_indices = Eigen::VectorXi::LinSpaced(model.nq, 0, model.nq - 1),
      .v_indices = Eigen::VectorXi::LinSpaced(model.nv, 0, model.nv - 1),
      .has_continuous_dofs = default_group_has_continuous_dofs,
      .nq_collapsed = static_cast<size_t>(model.nq) - default_group_num_continuous_dofs};

  return joint_group_map;
}

tl::expected<Eigen::VectorXd, std::string>
collapseContinuousJointPositions(const Scene& scene, const std::string& group_name,
                                 const Eigen::VectorXd& q_orig) {
  const auto maybe_joint_group_info = scene.getJointGroupInfo(group_name);
  if (!maybe_joint_group_info) {
    return tl::make_unexpected("Failed to collapse continuous degrees of freedom: " +
                               maybe_joint_group_info.error());
  }
  const auto& joint_group_info = maybe_joint_group_info.value();

  // Return in the trivial case of no continuous degrees of freedom.
  if (!joint_group_info.has_continuous_dofs) {
    return q_orig;
  }

  // Validate the number of degrees of freedom.
  if (q_orig.size() != joint_group_info.q_indices.size()) {
    return tl::make_unexpected("Size mismatch: Expected " +
                               std::to_string(joint_group_info.q_indices.size()) +
                               " elements but got " + std::to_string(q_orig.size()) + ".");
  }
  Eigen::VectorXd q_collapsed = Eigen::VectorXd::Zero(joint_group_info.nq_collapsed);

  // Now collapse the joints
  size_t orig_nq = 0;
  size_t collapsed_nq = 0;
  for (const auto& joint_name : joint_group_info.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }
    switch (joint_info.type) {
    case JointType::REVOLUTE:
    case JointType::PRISMATIC:
      for (size_t dof = 0; dof < joint_info.num_position_dofs; ++dof) {
        q_collapsed(collapsed_nq) = q_orig(orig_nq);
        ++orig_nq;
        ++collapsed_nq;
      }
      break;
    case JointType::CONTINUOUS:
      // This translates to: theta = atan2(sin(theta), cos(theta))
      q_collapsed(collapsed_nq) = std::atan2(q_orig(orig_nq + 1), q_orig(orig_nq));
      orig_nq += 2;
      ++collapsed_nq;
      break;
    case JointType::PLANAR:
      q_collapsed(collapsed_nq) = q_orig(orig_nq);
      q_collapsed(collapsed_nq + 1) = q_orig(orig_nq + 1);
      // This translates to: theta = atan2(sin(theta), cos(theta))
      q_collapsed(collapsed_nq + 2) = std::atan2(q_orig(orig_nq + 3), q_orig(orig_nq + 2));
      orig_nq += 4;
      collapsed_nq += 3;
      break;
    default:
      throw std::runtime_error("Floating and unknown joints not supported.");
    }
  }

  return q_collapsed;
}

tl::expected<Eigen::VectorXd, std::string>
expandContinuousJointPositions(const Scene& scene, const std::string& group_name,
                               const Eigen::VectorXd& q_orig) {
  const auto maybe_joint_group_info = scene.getJointGroupInfo(group_name);
  if (!maybe_joint_group_info) {
    return tl::make_unexpected("Failed to expand continuous degrees of freedom: " +
                               maybe_joint_group_info.error());
  }
  const auto& joint_group_info = maybe_joint_group_info.value();

  // Return in the trivial case of no continuous degrees of freedom.
  if (!joint_group_info.has_continuous_dofs) {
    return q_orig;
  }

  // Validate the number of degrees of freedom.
  if (static_cast<size_t>(q_orig.size()) != joint_group_info.nq_collapsed) {
    return tl::make_unexpected("Size mismatch: Expected " +
                               std::to_string(joint_group_info.nq_collapsed) +
                               " elements but got " + std::to_string(q_orig.size()) + ".");
  }
  Eigen::VectorXd q_expanded = Eigen::VectorXd::Zero(joint_group_info.q_indices.size());

  // Now expand the joints
  size_t orig_nq = 0;
  size_t expanded_nq = 0;
  for (const auto& joint_name : joint_group_info.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }
    switch (joint_info.type) {
    case JointType::REVOLUTE:
    case JointType::PRISMATIC:
      for (size_t dof = 0; dof < joint_info.num_position_dofs; ++dof) {
        q_expanded(expanded_nq) = q_orig(orig_nq);
        ++orig_nq;
        ++expanded_nq;
      }
      break;
    case JointType::CONTINUOUS:
      // This translates theta to [cos(theta), sin(theta)]
      q_expanded(expanded_nq) = std::cos(q_orig(orig_nq));
      q_expanded(expanded_nq + 1) = std::sin(q_orig(orig_nq));
      ++orig_nq;
      expanded_nq += 2;
      break;
    case JointType::PLANAR:
      q_expanded(expanded_nq) = q_orig(orig_nq);
      q_expanded(expanded_nq + 1) = q_orig(orig_nq + 1);
      // This translates theta to [cos(theta), sin(theta)]
      q_expanded(expanded_nq + 2) = std::cos(q_orig(orig_nq + 2));
      q_expanded(expanded_nq + 3) = std::sin(q_orig(orig_nq + 2));
      orig_nq += 3;
      expanded_nq += 4;
      break;
    default:
      throw std::runtime_error("Floating and unknown joints not supported.");
    }
  }

  return q_expanded;
}

Eigen::VectorXd jointPositionsWithMimicsFromPinocchio(const Scene& scene,
                                                      const Eigen::VectorXd& q) {
  const auto& model = scene.getModel();
  const auto& joint_names = scene.getJointNamesWithMimics();

  size_t total_size = 0;
  for (const auto& joint_name : joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    total_size += joint_info.num_position_dofs;
  }

  Eigen::VectorXd positions(static_cast<Eigen::Index>(total_size));
  Eigen::Index out_idx = 0;
  for (const auto& joint_name : joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      const auto& mimic = joint_info.mimic_info.value();
      const auto mimicked_id = model.getJointId(mimic.mimicked_joint_name);
      const auto mimicked_q_idx = model.idx_qs[mimicked_id];
      for (size_t dof = 0; dof < joint_info.num_position_dofs; ++dof) {
        positions(out_idx++) =
            mimic.scaling * q(mimicked_q_idx + static_cast<Eigen::Index>(dof)) + mimic.offset;
      }
    } else {
      const auto joint_id = model.getJointId(joint_name);
      const auto q_idx = model.idx_qs[joint_id];
      const auto nq = model.joints[joint_id].nq();
      for (Eigen::Index dof = 0; dof < nq; ++dof) {
        positions(out_idx++) = q(q_idx + dof);
      }
    }
  }
  return positions;
}

bool computeCollisionsVerbose(const pinocchio::Model& model, pinocchio::Data& data,
                              const pinocchio::GeometryModel& collision_model,
                              pinocchio::GeometryData& geom_data, const Eigen::VectorXd& q) {
  pinocchio::updateGeometryPlacements(model, data, collision_model, geom_data, q);
  const auto result = pinocchio::computeCollisions(model, data, collision_model, geom_data, q,
                                                   /*stop_at_first_collision=*/false);

  for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k) {
    const auto& cp = collision_model.collisionPairs.at(k);
    const auto& cr = geom_data.collisionResults.at(k);
    if (cr.isCollision()) {
      const auto& body1 = collision_model.geometryObjects.at(cp.first).name;
      const auto& body2 = collision_model.geometryObjects.at(cp.second).name;
      std::cout << "Collision detected between " << body1 << " and " << body2 << std::endl;
    }
  }

  return result;
}

}  // namespace roboplan
