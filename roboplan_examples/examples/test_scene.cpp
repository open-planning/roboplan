#include <filesystem>
#include <roboplan/core/scene.hpp>
#include <vector>

int main(int /*argc*/, char* /*argv*/[]) {

  const std::string install_prefix = TEST_SCENE_INSTALL_PREFIX;

  const std::filesystem::path urdf_path{
      install_prefix +
      "/share/roboplan_examples/ur_robot_model/ur5_gripper.urdf"};
  const std::filesystem::path srdf_path{
      install_prefix +
      "/share/roboplan_examples/ur_robot_model/ur5_gripper.srdf"};
  const std::vector<std::filesystem::path> package_paths{
      install_prefix + "/share/roboplan_examples/"};

  auto scene =
      roboplan::Scene("test_scene", urdf_path, srdf_path, package_paths);
  scene.print();

  return 0;
}
