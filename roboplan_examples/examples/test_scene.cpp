#include <filesystem>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>

int main(int /*argc*/, char* /*argv*/[]) {

  const auto share_prefix = roboplan_example_models::get_package_share_dir();

  // Set up the scene
  const auto urdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.urdf";
  const auto srdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.srdf";
  const std::vector<std::filesystem::path> package_paths = {share_prefix};

  auto scene =
      roboplan::Scene("test_scene", urdf_path, srdf_path, package_paths);
  scene.print();

  return 0;
}
