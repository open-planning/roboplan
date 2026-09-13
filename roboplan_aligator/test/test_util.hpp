#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>

namespace roboplan::testing {

// Scene fixtures shared by every aligator test target. Kept aligator-free so public-API-only
// targets (test_types, test_trajectory_optimizer, test_reduced_group_model) can include this header
// without linking aligator.

inline std::shared_ptr<Scene> makeScene(const std::string& robot_dir, const std::string& urdf,
                                        const std::string& srdf) {
  const auto model_prefix = example_models::get_package_models_dir();
  const std::vector<std::filesystem::path> package_paths = {
      example_models::get_package_share_dir()};
  return std::make_shared<Scene>("test_scene", model_prefix / robot_dir / urdf,
                                 model_prefix / robot_dir / srdf, package_paths);
}

inline std::shared_ptr<Scene> makeUr5Scene() {
  return makeScene("ur_robot_model", "ur5_gripper.urdf", "ur5_gripper.srdf");
}

inline std::shared_ptr<Scene> makeSo101Scene() {
  return makeScene("so101_robot_model", "so101.urdf", "so101.srdf");
}

// SO-101 arm chain-tip link (movable by the "arm" group).
inline constexpr const char* kTipFrame = "gripper_link";

}  // namespace roboplan::testing
