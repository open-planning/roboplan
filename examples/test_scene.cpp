#include <filesystem>

#include <roboplan/scene.hpp>

int main(int /*argc*/, char* /*argv*/[]) {
  const std::filesystem::path urdf_path{
      "/home/sebastian/workspace/roboplan_ws/src/roboplan/"
      "pinocchio_ros_example/ur_robot_model/ur5_gripper.urdf"};

  const std::filesystem::path srdf_path{
      "/home/sebastian/workspace/roboplan_ws/src/roboplan/"
      "pinocchio_ros_example/ur_robot_model/ur5_gripper.srdf"};

  auto scene = roboplan::Scene(urdf_path, srdf_path);
  scene.print();

  return 0;
}
