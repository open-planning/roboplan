#include <filesystem>
#include <iostream>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>

using namespace roboplan;

int main(int /*argc*/, char* /*argv*/[]) {

  const auto share_prefix = example_models::get_package_share_dir();

  // Set up the scene
  const auto urdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.urdf";
  const auto srdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.srdf";
  const std::vector<std::filesystem::path> package_paths = {share_prefix};

  auto scene = Scene("test_scene", urdf_path, srdf_path, package_paths);
  std::cout << scene;

  // Generate a random state
  auto random_positions = scene.randomPositions();
  std::cout << "Random positions:\n" << random_positions.transpose() << "\n";
  scene.setRngSeed(1234);
  random_positions = scene.randomPositions();
  std::cout << "Random positions (seeded):\n" << random_positions.transpose() << "\n";

  return 0;
}
