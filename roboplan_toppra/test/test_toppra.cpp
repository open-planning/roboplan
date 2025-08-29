#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

JointPath createTestPathShort() {
  JointPath path;
  path.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  {
    Eigen::VectorXd point(6);
    point << -1.81545, -2.96566, 1.05139, -1.67655, -1.78886, 1.06137;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -1.81565, -2.96506, 1.05111, -1.6767, -1.78838, 1.06168;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -3.02021, 0.604097, -0.607038, -2.56699, 1.04115, 2.91201;
    path.positions.push_back(point);
  }
  return path;
}

JointPath createTestPathLong() {
  JointPath path;
  path.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  {
    Eigen::VectorXd point(6);
    point << -2.39453, 1.58901, -0.244726, 2.55989, -2.87469, 3.13679;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -2.13817, 1.62902, -0.220308, 1.96056, -2.56875, 2.44449;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -2.21516, 1.26655, -0.368015, 1.72299, -1.8913, 1.87397;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -1.96008, 1.36509, 0.129397, 1.11879, -1.57525, 1.4126;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -1.80117, 0.903983, 0.753074, 0.986503, -1.04419, 1.14127;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -1.85521, 1.13646, 1.14821, 0.433204, -0.58247, 0.623983;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -1.60414, 1.2469, 1.26467, 0.0179813, -0.256215, -0.171222;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -1.27481, 1.39845, 0.987443, -0.602589, -0.223673, -0.80805;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -0.94548, 1.55, 0.710214, -1.22316, -0.191131, -1.44488;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -0.616148, 1.70156, 0.432985, -1.84373, -0.158589, -2.08171;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -0.286816, 1.85311, 0.155756, -2.4643, -0.126047, -2.71853;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -0.0736738, 1.95119, -0.0236654, -2.86593, -0.104986, -3.13069;
    path.positions.push_back(point);
  }
  return path;
}

class RoboPlanToppraTest : public ::testing::Test {
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
};

TEST_F(RoboPlanToppraTest, EmptyPath) {
  JointPath path;
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_);
  auto result = toppra.generate(path, dt);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), "Path must have at least 2 points.");
}

TEST_F(RoboPlanToppraTest, NegativeDt) {
  auto path = createTestPathShort();
  double dt = -0.1;

  auto toppra = PathParameterizerTOPPRA(scene_);
  auto result = toppra.generate(path, dt);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), "dt must be strictly positive.");
}

TEST_F(RoboPlanToppraTest, BadVelocityAccelerationScales) {
  auto path = createTestPathShort();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_);

  for (const auto& vel_scale : std::vector<double>{-0.1, 0.0, 1.1}) {
    auto result = toppra.generate(path, dt, vel_scale);
    ASSERT_FALSE(result.has_value());
    ASSERT_EQ(result.error(), "Velocity scale must be greater than 0.0 and less than 1.0.");
  }

  for (const auto& acc_scale : std::vector<double>{-0.1, 0.0, 1.1}) {
    auto result = toppra.generate(path, dt, /* vel_scale */ 0.5, acc_scale);
    ASSERT_FALSE(result.has_value());
    ASSERT_EQ(result.error(), "Acceleration scale must be greater than 0.0 and less than 1.0.");
  }
}

TEST_F(RoboPlanToppraTest, BadJointNames) {
  auto path = createTestPathShort();
  path.joint_names = {"fr3_joint1", "fr3_joint2"};
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_);
  auto result = toppra.generate(path, dt);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), "Path joint names do not match the scene joint names.");
}

TEST_F(RoboPlanToppraTest, ShortPath) {
  auto path = createTestPathShort();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_);
  auto result = toppra.generate(path, dt);
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, LongPath) {
  auto path = createTestPathLong();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_);
  auto result = toppra.generate(path, dt);
  ASSERT_TRUE(result.has_value());
}

}  // namespace roboplan
