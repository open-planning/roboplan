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

  // TODO: Fill in by parsing the SRDF.
  tinyxml2::XMLDocument doc;
  doc.Parse(srdf.c_str());

  return joint_group_map;
}

}  // namespace roboplan
