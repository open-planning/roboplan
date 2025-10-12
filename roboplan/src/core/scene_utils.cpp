#include <stdexcept>

#include <tinyxml2.h>

#include <roboplan/core/scene_utils.hpp>

namespace roboplan {

std::unordered_map<std::string, pinocchio::FrameIndex>
createFrameMap(const pinocchio::Model& model) {
  std::unordered_map<std::string, pinocchio::FrameIndex> frame_map;
  for (const auto& frame : model.frames) {
    frame_map[frame.name] = model.getFrameId(frame.name);
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

    // There are a few valid elements in groups: "joint", "chain", and "group".
    for (tinyxml2::XMLElement* child = group->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
      const std::string elem_name = child->Name();
      if (elem_name == "joint") {
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
        std::vector<int> joint_indices;
        while (true) {
          const auto& frame = model.frames.at(cur_frame_id);
          const auto parent_joint_id = frame.parentJoint;
          const auto& parent_joint_name = model.names.at(parent_joint_id);
          // group_info.joint_names.push_back(parent_joint_name);
          joint_indices.push_back(parent_joint_id);
          cur_frame_id = model.frames.at(model.getFrameId(parent_joint_name)).parentFrame;
          if (cur_frame_id == base_frame_id) {
            break;
          }
          if (cur_frame_id == 0) {
            throw std::runtime_error(
                "Recursed the whole robot model and did not find the base frame!");
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
        const auto& subgroup_info = joint_group_map.at(group_name);  // TODO validate
        group_info.joint_names.insert(group_info.joint_names.end(),
                                      subgroup_info.joint_names.begin(),
                                      subgroup_info.joint_names.end());
        group_info.joint_indices.insert(group_info.joint_indices.end(),
                                        subgroup_info.joint_indices.begin(),
                                        subgroup_info.joint_indices.end());
      }
    }

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
      const auto joint_type = kPinocchioJointTypeMap.at(joint.shortname());
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
    const auto joint_type = kPinocchioJointTypeMap.at(joint.shortname());
    if (joint_type == JointType::CONTINUOUS || joint_type == JointType::PLANAR) {
      default_group_has_continuous_dofs = true;
      default_group_num_continuous_dofs += 1;
    }
  }

  joint_group_map[""] = JointGroupInfo{
      .joint_names = std::vector<std::string>(model.names.begin() + 1, model.names.end()),
      .joint_indices = all_joint_indices,
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

}  // namespace roboplan
