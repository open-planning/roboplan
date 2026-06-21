#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_set>

#include <tinyxml2.h>

#include <roboplan/core/scene_utils.hpp>

namespace {

/// @brief Returns whether the given velocity-space DOF index of a joint is free-rotating.
/// @details These are the unbounded orientation DOFs for which position limits are meaningless:
/// the single DOF of a continuous joint, the rotational DOF of a planar joint, and the three
/// rotational DOFs of a floating joint. Position limit indices follow the velocity (tangent)
/// space, so a continuous DOF collapses to a single index here.
bool isFreeRotatingDof(roboplan::JointType type, int dof) {
  switch (type) {
  case roboplan::JointType::CONTINUOUS:
    return dof == 0;
  case roboplan::JointType::PLANAR:
    return dof == 2;  // (x, y, theta) -> theta is free-rotating.
  case roboplan::JointType::FLOATING:
    return dof >= 3;  // (x, y, z, rx, ry, rz) -> the rotational DOFs are free-rotating.
  default:
    return false;
  }
}

/// @brief Maps an infinite position limit to the finite sentinel used to denote "unbounded".
/// @details JointInfo represents an unbounded position limit as
/// std::numeric_limits<double>::lowest() / max() (see the JointInfo constructor), not as
/// +/-infinity. A user-supplied '.inf' / '-.inf' is normalized to these sentinels so that an
/// overridden unbounded limit is represented identically to the default unbounded limit.
double sanitizePositionLimit(double value) {
  if (std::isinf(value)) {
    return value > 0.0 ? std::numeric_limits<double>::max() : std::numeric_limits<double>::lowest();
  }
  return value;
}

}  // namespace

namespace roboplan {

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

std::unordered_map<std::string, JointGroupInfo> createJointGroupInfo(const pinocchio::Model& model,
                                                                     const std::string& srdf) {
  std::unordered_map<std::string, JointGroupInfo> joint_group_map;

  // Parse the document with TinyXML2.
  tinyxml2::XMLDocument doc;
  doc.Parse(srdf.c_str());
  tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
  if (robot == nullptr) {
    throw std::runtime_error("No <robot> tag found in the SRDF file!");
  }

  // Loop through all the "group" elements.
  for (tinyxml2::XMLElement* group = robot->FirstChildElement("group"); group != nullptr;
       group = group->NextSiblingElement("group")) {
    const char* name;
    if (group->QueryStringAttribute("name", &name) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("Found an invalid group with no name in the SRDF!");
    }

    JointGroupInfo group_info;

    // Accumulate the group's links in a set so duplicates collapse automatically.
    std::unordered_set<std::string> link_name_set;

    // There are a few valid elements in groups: "link", "joint", "chain", and "group".
    for (tinyxml2::XMLElement* child = group->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
      const std::string elem_name = child->Name();
      if (elem_name == "link") {
        // Links can be manually specified to be part of a group in the SRDF.
        const char* link_name;
        if (child->QueryStringAttribute("name", &link_name) != tinyxml2::XML_SUCCESS) {
          throw std::runtime_error("Group '" + std::string(name) +
                                   "' specifies a link with no name in the SRDF!");
        }
        link_name_set.insert(link_name);
      } else if (elem_name == "joint") {
        // The joint case is straightforward; just add the joint name.
        const char* joint_name;
        if (child->QueryStringAttribute("name", &joint_name) != tinyxml2::XML_SUCCESS) {
          throw std::runtime_error("Group '" + std::string(name) +
                                   "' specifies a joint with no name in the SRDF!");
        }
        const auto joint_id = model.getJointId(joint_name);
        if (joint_id >= static_cast<size_t>(model.njoints)) {
          continue;
        }
        group_info.joint_names.push_back(joint_name);
        group_info.joint_indices.push_back(joint_id);
      } else if (elem_name == "chain") {
        // In the chain case, we must recurse from the specified tip frame all the way
        // up to the base frame, collecting all joints along the way.
        const char* base_link;
        if (child->QueryStringAttribute("base_link", &base_link) != tinyxml2::XML_SUCCESS) {
          throw std::runtime_error("Group '" + std::string(name) +
                                   "' chain specifies no 'base_link' attribute in the SRDF!");
        }
        const char* tip_link;
        if (child->QueryStringAttribute("tip_link", &tip_link) != tinyxml2::XML_SUCCESS) {
          throw std::runtime_error("Group '" + std::string(name) +
                                   "' chain specifies no 'tip_link' attribute in the SRDF!");
        }

        auto cur_frame_id = model.getFrameId(tip_link);
        const auto base_frame_id = model.getFrameId(base_link);
        const auto base_link_parent_joint_id = model.frames.at(base_frame_id).parentJoint;
        std::vector<int> joint_indices;
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
            throw std::runtime_error("Recursed the whole robot model for chain in group '" +
                                     std::string(name) + "' and did not find the base frame!");
          }
        }
        // Add the joint information in the reverse order.
        for (auto it = joint_indices.rbegin(); it != joint_indices.rend(); ++it) {
          group_info.joint_names.push_back(model.names.at(*it));
          group_info.joint_indices.push_back(*it);
        }
      } else if (elem_name == "group") {
        // In the group case, just add the joints from the parent group.
        // The parent group must be defined first in the SRDF file!
        const char* group_name;
        if (child->QueryStringAttribute("name", &group_name) != tinyxml2::XML_SUCCESS) {
          throw std::runtime_error("Group '" + std::string(name) +
                                   "' specifies a subgroup with no name in the SRDF!");
        }
        auto it = joint_group_map.find(group_name);
        if (it == joint_group_map.end()) {
          throw std::runtime_error("Group '" + std::string(name) + "' specifies a subgroup '" +
                                   std::string(group_name) +
                                   "' which has not yet been parsed in the SRDF.");
        }
        const auto& subgroup_info = it->second;
        group_info.joint_names.insert(group_info.joint_names.end(),
                                      subgroup_info.joint_names.begin(),
                                      subgroup_info.joint_names.end());
        group_info.joint_indices.insert(group_info.joint_indices.end(),
                                        subgroup_info.joint_indices.begin(),
                                        subgroup_info.joint_indices.end());
        link_name_set.insert(subgroup_info.link_names.begin(), subgroup_info.link_names.end());
      }
    }

    // Now that all the group's joints are known, collect the links that they drive.
    // A single moving joint can support multiple links: the link it actuates plus any links rigidly
    // attached to it through fixed joints, which Pinocchio collapses into the same parent joint.
    for (const auto jid : group_info.joint_indices) {
      for (const auto& frame : model.frames) {
        if (frame.type == pinocchio::BODY && frame.parentJoint == jid) {
          link_name_set.insert(frame.name);
        }
      }
    }

    group_info.link_names.assign(link_name_set.begin(), link_name_set.end());

    // Once we've defined all joint names in the group, compute the position and velocity indices.
    std::vector<int> q_indices;
    std::vector<int> v_indices;
    size_t num_joints_with_continuous_dofs = 0;
    for (const auto jid : group_info.joint_indices) {
      const auto& joint = model.joints.at(jid);
      const auto& q_idx = model.idx_qs.at(jid);
      for (int dof = 0; dof < joint.nq(); ++dof) {
        q_indices.push_back(q_idx + dof);
      }
      const auto& v_idx = model.idx_vs.at(jid);
      for (int dof = 0; dof < joint.nv(); ++dof) {
        v_indices.push_back(v_idx + dof);
      }

      // Check for any continuous degrees of freedom.
      auto it = kPinocchioJointTypeMap.find(joint.shortname());
      if (it == kPinocchioJointTypeMap.end()) {
        throw std::runtime_error("Unsupported Pinocchio joint type: '" + joint.shortname() + "'");
      }
      const auto joint_type = it->second;

      if (joint_type == JointType::CONTINUOUS || joint_type == JointType::PLANAR) {
        num_joints_with_continuous_dofs += 1;
      }
    }
    group_info.nq_collapsed = q_indices.size() - num_joints_with_continuous_dofs;
    if (num_joints_with_continuous_dofs > 0) {
      group_info.has_continuous_dofs = true;
    }

    group_info.q_indices.resize(q_indices.size());
    for (size_t idx = 0; idx < q_indices.size(); ++idx) {
      group_info.q_indices(idx) = q_indices.at(idx);
    }
    group_info.v_indices.resize(v_indices.size());
    for (size_t idx = 0; idx < v_indices.size(); ++idx) {
      group_info.v_indices(idx) = v_indices.at(idx);
    }

    joint_group_map[name] = group_info;
  }

  // Create a default empty group with all the indices.
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

std::unordered_map<std::string, UrdfExtendedJointLimits>
parseUrdfExtendedJointLimits(const std::string& urdf) {
  std::unordered_map<std::string, UrdfExtendedJointLimits> result;

  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS) {
    return result;
  }

  const tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
  if (!robot) {
    return result;
  }

  for (const tinyxml2::XMLElement* joint = robot->FirstChildElement("joint"); joint;
       joint = joint->NextSiblingElement("joint")) {
    const char* name = joint->Attribute("name");
    if (!name) {
      continue;
    }
    const tinyxml2::XMLElement* limit = joint->FirstChildElement("limit");
    if (!limit) {
      continue;
    }

    UrdfExtendedJointLimits limits;
    double val = 0.0;
    if (limit->QueryDoubleAttribute("acceleration", &val) == tinyxml2::XML_SUCCESS) {
      limits.acceleration = val;
    }
    if (limit->QueryDoubleAttribute("jerk", &val) == tinyxml2::XML_SUCCESS) {
      limits.jerk = val;
    }
    result.emplace(name, limits);
  }

  return result;
}

void overrideJointLimitsFromYaml(
    const pinocchio::Model& model, const YAML::Node& yaml_config,
    const std::unordered_map<std::string, UrdfExtendedJointLimits>& urdf_extended_limits,
    const std::string& joint_name, JointInfo& info) {
  const int nv = static_cast<int>(info.num_velocity_dofs);
  // Starting index of this joint's DOFs in the model's velocity vector.
  const auto v_start = model.idx_vs[model.getJointId(joint_name)];

  // Override URDF limits if supplied in the YAML file.
  std::optional<YAML::Node> maybe_min_pos_limits;
  std::optional<YAML::Node> maybe_max_pos_limits;
  std::optional<YAML::Node> maybe_vel_limits;
  std::optional<YAML::Node> maybe_acc_limits;
  std::optional<YAML::Node> maybe_jerk_limits;
  if (yaml_config["joint_limits"] && yaml_config["joint_limits"][joint_name]) {
    const auto& limits_config = yaml_config["joint_limits"][joint_name];
    if (limits_config["min_position"]) {
      maybe_min_pos_limits = limits_config["min_position"];
      if (!maybe_min_pos_limits->IsSequence() ||
          (maybe_min_pos_limits->size() != static_cast<size_t>(nv))) {
        throw std::runtime_error("Minimum position limits for joint '" + joint_name +
                                 "' must be a sequence of size " + std::to_string(nv) + ".");
      }
    }
    if (limits_config["max_position"]) {
      maybe_max_pos_limits = limits_config["max_position"];
      if (!maybe_max_pos_limits->IsSequence() ||
          (maybe_max_pos_limits->size() != static_cast<size_t>(nv))) {
        throw std::runtime_error("Maximum position limits for joint '" + joint_name +
                                 "' must be a sequence of size " + std::to_string(nv) + ".");
      }
    }
    if (limits_config["max_velocity"]) {
      maybe_vel_limits = limits_config["max_velocity"];
      if (!maybe_vel_limits->IsSequence() ||
          (maybe_vel_limits->size() != static_cast<size_t>(nv))) {
        throw std::runtime_error("Velocity limits for joint '" + joint_name +
                                 "' must be a sequence of size " + std::to_string(nv) + ".");
      }
    }
    if (limits_config["max_acceleration"]) {
      maybe_acc_limits = limits_config["max_acceleration"];
      if (!maybe_acc_limits->IsSequence() ||
          (maybe_acc_limits->size() != static_cast<size_t>(nv))) {
        throw std::runtime_error("Acceleration limits for joint '" + joint_name +
                                 "' must be a sequence of size " + std::to_string(nv) + ".");
      }
    }
    if (limits_config["max_jerk"]) {
      maybe_jerk_limits = limits_config["max_jerk"];
      if (!maybe_jerk_limits->IsSequence() ||
          (maybe_jerk_limits->size() != static_cast<size_t>(nv))) {
        throw std::runtime_error("Jerk limits for joint '" + joint_name +
                                 "' must be a sequence of size " + std::to_string(nv) + ".");
      }
    }
  }
  const auto urdf_extended_it = urdf_extended_limits.find(joint_name);
  for (int idx = 0; idx < nv; ++idx) {
    // Position limits are overridden per velocity-space DOF. For free-rotating DOFs (continuous
    // joints and the orientation DOFs of planar/floating joints) a position limit is meaningless,
    // so any finite override is discarded with a warning. Users should use '.inf' / '-.inf' to
    // explicitly denote an unbounded position for these DOFs.
    const bool is_free_dof = isFreeRotatingDof(info.type, idx);
    bool discarded_pos_limit = false;
    if (maybe_min_pos_limits) {
      const double val = maybe_min_pos_limits.value()[idx].as<double>();
      if (is_free_dof) {
        discarded_pos_limit |= std::isfinite(val);
      } else {
        info.limits.min_position[idx] = sanitizePositionLimit(val);
      }
    }
    if (maybe_max_pos_limits) {
      const double val = maybe_max_pos_limits.value()[idx].as<double>();
      if (is_free_dof) {
        discarded_pos_limit |= std::isfinite(val);
      } else {
        info.limits.max_position[idx] = sanitizePositionLimit(val);
      }
    }
    if (discarded_pos_limit) {
      std::cout << "Warning: joint '" << joint_name
                << "' has a free-rotating DOF (velocity-space index " << idx
                << "); the specified position limit was discarded. Use '.inf' and '-.inf' in the "
                   "YAML config to denote an unbounded position."
                << std::endl;
    }
    if (maybe_vel_limits) {
      info.limits.max_velocity[idx] = maybe_vel_limits.value()[idx].as<double>();
    } else {
      info.limits.max_velocity[idx] = model.velocityLimit(v_start + idx);
    }
    if (maybe_acc_limits) {
      info.limits.max_acceleration[idx] = maybe_acc_limits.value()[idx].as<double>();
    } else if (urdf_extended_it != urdf_extended_limits.end() &&
               urdf_extended_it->second.acceleration.has_value()) {
      info.limits.max_acceleration[idx] = urdf_extended_it->second.acceleration.value();
    }
    if (maybe_jerk_limits) {
      info.limits.max_jerk[idx] = maybe_jerk_limits.value()[idx].as<double>();
    } else if (urdf_extended_it != urdf_extended_limits.end() &&
               urdf_extended_it->second.jerk.has_value()) {
      info.limits.max_jerk[idx] = urdf_extended_it->second.jerk.value();
    }
  }
}

}  // namespace roboplan
