#include <filesystem>
#include <vector>

#include <example_resources/example_resources.hpp>
#include <roboplan/core/scene.hpp>

int main(int /*argc*/, char* /*argv*/[]) {

  const std::filesystem::path share_prefix = std::filesystem::path(
      roboplan_example_resources::get_package_share_dir());

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
