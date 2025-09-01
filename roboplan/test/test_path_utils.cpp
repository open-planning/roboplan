#include <gtest/gtest.h>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>

namespace roboplan {

class RoboPlanPathUtilsTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto share_prefix = example_models::get_package_share_dir();
    const auto urdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = share_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const std::vector<std::filesystem::path> package_paths = {share_prefix};
    scene_ = std::make_shared<Scene>("test_scene", urdf_path, srdf_path, package_paths);
  }

public:
  // No default constructors, so must be pointers.
  std::shared_ptr<Scene> scene_;

  JointPath getTestPath(const size_t num_points) {
    JointPath test_path;
    test_path.joint_names = scene_->getJointNames();

    if (num_points == 0)
      return test_path;

    Eigen::VectorXd q1(6);
    Eigen::VectorXd q2(6);
    Eigen::VectorXd q3(6);
    Eigen::VectorXd q4(6);
    q1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    q2 << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0;
    q3 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    q4 << 1.5, 0.0, 0.0, 0.0, 0.0, 0.0;

    if (num_points >= 1)
      test_path.positions.push_back(q1);
    if (num_points >= 2)
      test_path.positions.push_back(q2);
    if (num_points >= 3)
      test_path.positions.push_back(q3);
    if (num_points >= 4)
      test_path.positions.push_back(q4);

    return test_path;
  }
};

TEST_F(RoboPlanPathUtilsTest, testGetNormalizedPathScaling) {

  JointPath empty_path = getTestPath(0);
  auto empty_scalings_maybe = getNormalizedPathScaling(*scene_, empty_path);
  EXPECT_FALSE(empty_scalings_maybe.has_value());

  JointPath length_2_path = getTestPath(2);
  auto length_2_path_scalings = getNormalizedPathScaling(*scene_, length_2_path).value();
  ASSERT_EQ(length_2_path_scalings.size(), 2);
  EXPECT_DOUBLE_EQ(length_2_path_scalings(0), 0.0);
  EXPECT_DOUBLE_EQ(length_2_path_scalings(1), 1.0);

  // Path with 3 evenly spaced points
  auto test_path = getTestPath(3);
  auto path_scalings = getNormalizedPathScaling(*scene_, test_path).value();

  ASSERT_EQ(path_scalings.size(), 3);
  EXPECT_DOUBLE_EQ(path_scalings(0), 0.0);
  EXPECT_DOUBLE_EQ(path_scalings(1), 0.5);
  EXPECT_DOUBLE_EQ(path_scalings(2), 1.0);
}

TEST_F(RoboPlanPathUtilsTest, testGetConfigurationFromNormalizedPathScaling) {
  auto test_path = getTestPath(3);
  auto path_scalings = getNormalizedPathScaling(*scene_, test_path).value();

  auto [config, idx] =
      getConfigurationFromNormalizedPathScaling(*scene_, test_path, path_scalings, 0.25);
  ASSERT_EQ(idx, 1);
  ASSERT_EQ(config(0), 0.25);

  auto [config_end, idx_end] =
      getConfigurationFromNormalizedPathScaling(*scene_, test_path, path_scalings, 0.95);
  ASSERT_EQ(idx_end, 2);
  ASSERT_EQ(config_end(0), 1.0);

  auto [config_start, idx_start] =
      getConfigurationFromNormalizedPathScaling(*scene_, test_path, path_scalings, 0.0);
  ASSERT_EQ(idx_start, 1);
  ASSERT_EQ(config_start(0), 0.0);
}

TEST_F(RoboPlanPathUtilsTest, testShortcutPath) {
  auto test_path = getTestPath(4);
  auto shortcut_path =
      shortcutPath(*scene_, test_path, 0.25, /* max_iters */ 100, /* seed */ 11235);

  ASSERT_EQ(shortcut_path.positions.size(), 2);
  ASSERT_EQ(shortcut_path.positions[0], test_path.positions[0]);
  ASSERT_EQ(shortcut_path.positions[1], test_path.positions[3]);
}

}  // namespace roboplan
