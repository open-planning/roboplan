#include <filesystem>
#include <vector>

#include <roboplan/core/scene.hpp>

int main(int /*argc*/, char* /*argv*/[]) {
  const std::filesystem::path urdf_path{
      "/home/sebastian/workspace/roboplan_ws/src/roboplan/"
      "roboplan_examples/ur_robot_model/ur5_gripper.urdf"};

  const std::filesystem::path srdf_path{
      "/home/sebastian/workspace/roboplan_ws/src/roboplan/"
      "roboplan_examples/ur_robot_model/ur5_gripper.srdf"};

  const std::vector<std::filesystem::path> package_paths{
      "/home/sebastian/workspace/roboplan_ws/src/roboplan/"};

  auto scene =
      roboplan::Scene("test_scene", urdf_path, srdf_path, package_paths);
  scene.print();

  return 0;
}
