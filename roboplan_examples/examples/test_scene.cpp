#include <filesystem>
#include <vector>

#include <examples/example_resources.hpp>
#include <roboplan/core/scene.hpp>

int main(int /*argc*/, char* /*argv*/[]) {

  const std::filesystem::path share_prefix =
      std::filesystem::path(roboplan_examples::PACKAGE_SHARE_DIR);

  const std::filesystem::path urdf_path =
      share_prefix / "ur_robot_model" / "ur5_gripper.urdf";
  const std::filesystem::path srdf_path =
      share_prefix / "ur_robot_model" / "ur5_gripper.srdf";
  const std::vector<std::filesystem::path> package_paths = {share_prefix};

  auto scene =
      roboplan::Scene("test_scene", urdf_path, srdf_path, package_paths);
  scene.print();

  return 0;
}
