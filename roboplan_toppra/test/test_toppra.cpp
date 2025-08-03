#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

class RoboPlanToppraTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto share_prefix = roboplan_example_models::get_package_share_dir();
    const auto urdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const std::vector<std::filesystem::path> package_paths = {share_prefix};
    scene_ = std::make_shared<Scene>("test_scene", urdf_path, srdf_path, package_paths);
  }

public:
  // No default constructors, so must be pointers.
  std::shared_ptr<Scene> scene_;
};

}  // namespace roboplan
