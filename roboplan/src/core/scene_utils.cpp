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
    group->QueryStringAttribute("name", &name);  // TODO: Check success
    std::cout << "Found group " << name << "\n";

    JointGroupInfo group_info;

    // There are a few valid elements in groups: "joint", "chain", and "group".
    for (tinyxml2::XMLElement* child = group->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
      const std::string elem_name = child->Name();
      if (elem_name == "joint") {
        // The joint case is straightforward; just add the joint name.
        const char* joint_name;
        child->QueryStringAttribute("name", &joint_name);
        std::cout << "  found a joint name " << joint_name << "\n";
        group_info.joint_names.push_back(joint_name);
      } else if (elem_name == "chain") {
        // In the chain case, we must recurse from the specified tip frame all the way
        // up to the base frame, collecting all joints along the way.
        const char* base_link;
        child->QueryStringAttribute("base_link", &base_link);
        const char* tip_link;
        child->QueryStringAttribute("tip_link", &tip_link);
        std::cout << "  found a chain from " << base_link << " to " << tip_link << "\n";

        auto cur_frame_id = model.getFrameId(tip_link);
        const auto base_frame_id = model.getFrameId(base_link);
        while (true) {
          const auto& frame = model.frames.at(cur_frame_id);
          std::cout << "    cur frame: " << frame.name << "\n";
          const auto parent_joint_id = frame.parentJoint;
          const auto& parent_joint_name = model.names.at(parent_joint_id);
          std::cout << "      parent joint: " << parent_joint_name << "\n";
          group_info.joint_names.push_back(parent_joint_name);
          cur_frame_id = model.frames.at(model.getFrameId(parent_joint_name)).parentFrame;
          if (cur_frame_id == base_frame_id) {
            std::cout << "Found base frame ID\n";
            break;
          }
          if (cur_frame_id == 0) {
            throw std::runtime_error(
                "Recursed the whole robot model and did not find the base frame!");
          }
        }
      } else if (elem_name == "group") {
        // In the group case, just add the joints from the parent group.
        // The parent group must be defined first in the SRDF file!
        const char* group_name;
        child->QueryStringAttribute("name", &group_name);
        std::cout << "  found a group name " << group_name << "\n";
        const auto& subgroup_info = joint_group_map.at(group_name);  // TODO validate
        group_info.joint_names.insert(group_info.joint_names.end(),
                                      subgroup_info.joint_names.begin(),
                                      subgroup_info.joint_names.end());
      }
    }

    // TODO: Once we've defined all the joint names in the group, compute the position and velocity
    // indices.

    joint_group_map[name] = group_info;
  }

  return joint_group_map;
}

}  // namespace roboplan
