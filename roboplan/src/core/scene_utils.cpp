#include <stdexcept>

#include <tinyxml2.h>

#include <roboplan/core/scene_utils.hpp>

namespace roboplan {

std::unordered_map<std::string, pinocchio::FrameIndex>
createFrameMap(const pinocchio::Model& model) {
  std::unordered_map<std::string, pinocchio::FrameIndex> frame_map;
  for (int i = 1; i < model.nframes; ++i) {
    const auto& frame = model.frames[i];
    frame_map[frame.name] = model.getFrameId(frame.name);
  }
  return frame_map;
}

std::map<std::string, JointGroupInfo> createJointGroupInfo(const pinocchio::Model& model,
                                                           const std::string& srdf) {
  std::map<std::string, JointGroupInfo> joint_group_map;

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
  joint_group_map[""] = JointGroupInfo{
      .joint_names = std::vector<std::string>(model.names.begin() + 1, model.names.end()),
      .joint_indices = all_joint_indices,
      .q_indices = Eigen::VectorXi::LinSpaced(model.nq, 0, model.nq - 1),
      .v_indices = Eigen::VectorXi::LinSpaced(model.nv, 0, model.nv - 1)};

  return joint_group_map;
}

}  // namespace roboplan
