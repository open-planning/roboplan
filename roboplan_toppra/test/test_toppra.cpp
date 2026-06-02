#include <cmath>
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

JointPath createTestPathWithCollisions() {
  JointPath path;
  path.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

  {
    Eigen::VectorXd point(6);
    point << 1.57974, 2.83593, 0.760326, 2.13506, 1.8242, -2.20896;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -0.765589, 1.19361, 0.599109, 2.7477, 1.69345, -1.58946;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << -1.49807, 1.85241, 1.07445, 0.736193, 1.12895, 0.264916;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << 0.209106, 2.43908, 1.76354, -0.905323, 0.0319808, 1.43478;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << 1.55133, 2.6235, 2.05615, 0.0521877, -1.88902, -0.137395;
    path.positions.push_back(point);
  }
  {
    Eigen::VectorXd point(6);
    point << 1.60935, 2.63147, 2.06879, 0.0935765, -1.97206, -0.205353;
    path.positions.push_back(point);
  }
  return path;
}

class RoboPlanToppraTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const std::vector<std::filesystem::path> package_paths = {
        example_models::get_package_share_dir()};
    scene_ = std::make_shared<Scene>("test_scene", urdf_path, srdf_path, package_paths);
  }

public:
  // No default constructors, so must be pointers.
  std::shared_ptr<Scene> scene_;
};

TEST_F(RoboPlanToppraTest, EmptyPath) {
  JointPath path;
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_, "arm");
  auto result = toppra.generate(path, dt);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), "Path must have at least 2 points.");
}

TEST_F(RoboPlanToppraTest, BadJointNames) {
  auto path = createTestPathShort();
  path.joint_names = {"fr3_joint1", "fr3_joint2"};
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_, "arm");
  auto result = toppra.generate(path, dt);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), "Path joint names do not match the scene joint names.");
}

TEST_F(RoboPlanToppraTest, NegativeDt) {
  auto path = createTestPathShort();
  double dt = -0.1;

  auto toppra = PathParameterizerTOPPRA(scene_, "arm");
  auto result = toppra.generate(path, dt);
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), "dt must be strictly positive.");
}

TEST_F(RoboPlanToppraTest, BadVelocityAccelerationScales) {
  auto path = createTestPathShort();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_, "arm");

  for (const auto& vel_scale : std::vector<double>{-0.1, 0.0, 1.1}) {
    auto result = toppra.generate(path, dt, SplineFittingMode::Hermite, vel_scale);
    ASSERT_FALSE(result.has_value());
    ASSERT_EQ(result.error(),
              "Velocity scale must be greater than 0.0 and less than or equal to 1.0.");
  }

  for (const auto& acc_scale : std::vector<double>{-0.1, 0.0, 1.1}) {
    auto result =
        toppra.generate(path, dt, SplineFittingMode::Hermite, /* vel_scale */ 0.5, acc_scale);
    ASSERT_FALSE(result.has_value());
    ASSERT_EQ(result.error(),
              "Acceleration scale must be greater than 0.0 and less than or equal to 1.0.");
  }
}

TEST_F(RoboPlanToppraTest, ShortPathHermite) {
  auto path = createTestPathShort();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_, "arm");
  auto result = toppra.generate(path, dt, SplineFittingMode::Hermite);
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, LongPathHermite) {
  auto path = createTestPathLong();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_, "arm");
  auto result = toppra.generate(path, dt, SplineFittingMode::Hermite);
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, ShortPathCubic) {
  auto path = createTestPathShort();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_, "arm");
  auto result = toppra.generate(path, dt, SplineFittingMode::Cubic);
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, LongPathCubic) {
  auto path = createTestPathLong();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_, "arm");
  auto result = toppra.generate(path, dt, SplineFittingMode::Cubic);
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, Adaptive) {
  auto path = createTestPathWithCollisions();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene_, "arm");
  auto result = toppra.generate(path, dt, SplineFittingMode::Adaptive);
  ASSERT_TRUE(result.has_value());
}

// Tests for joint groups containing a planar (mobile base) joint. These exercise the normalized
// arc-length representation that replaces the dense SE(2) resampling.
class RoboPlanToppraPlanarTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    const auto urdf_path = model_prefix / "stretch4_robot_model" / "stretch4_sg4.urdf";
    const auto srdf_path = model_prefix / "stretch4_robot_model" / "stretch4_sg4.srdf";
    const auto config_path = model_prefix / "stretch4_robot_model" / "stretch4_sg4_config.yaml";
    const std::vector<std::filesystem::path> package_paths = {
        example_models::get_package_share_dir()};
    scene_ =
        std::make_shared<Scene>("stretch_scene", urdf_path, srdf_path, package_paths, config_path);
  }

  // Builds an expanded group configuration: planar (x, y, cos, sin) followed by the arm joints
  // held at their home configuration.
  static Eigen::VectorXd makePoint(double x, double y, double theta) {
    Eigen::VectorXd point(9);
    point << x, y, std::cos(theta), std::sin(theta), 0.5, 0.065, 0.0, 0.0, 0.0;
    return point;
  }

public:
  std::shared_ptr<Scene> scene_;
  const std::vector<std::string> joint_names_{
      "mobile_base_planar_joint", "lift_joint",        "arm_l1_joint",
      "wrist_yaw_joint",          "wrist_pitch_joint", "wrist_roll_joint"};
};

TEST_F(RoboPlanToppraPlanarTest, GeneratesInAllModes) {
  JointPath path;
  path.joint_names = joint_names_;
  path.positions = {makePoint(0.0, 0.0, 0.0), makePoint(0.6, 0.2, 0.4), makePoint(1.0, 0.0, -0.3),
                    makePoint(1.4, 0.5, 0.2)};

  auto toppra = PathParameterizerTOPPRA(scene_, "stretch4_arm");
  for (const auto mode :
       {SplineFittingMode::Hermite, SplineFittingMode::Cubic, SplineFittingMode::Adaptive}) {
    auto result = toppra.generate(path, /* dt */ 0.01, mode);
    ASSERT_TRUE(result.has_value()) << result.error();
    const auto& traj = result.value();
    ASSERT_GT(traj.positions.size(), 1u);
    EXPECT_EQ(traj.positions.size(), traj.velocities.size());
    EXPECT_EQ(traj.positions.size(), traj.accelerations.size());
    // The planar position representation must remain on the unit circle.
    for (const auto& pos : traj.positions) {
      EXPECT_NEAR(pos(2) * pos(2) + pos(3) * pos(3), 1.0, 1.0e-6);
    }
    // Endpoints must match the path.
    EXPECT_TRUE(traj.positions.front().isApprox(path.positions.front(), 1.0e-6));
    EXPECT_TRUE(traj.positions.back().isApprox(path.positions.back(), 1.0e-6));
  }
}

// Verifies that the reconstructed base poses lie exactly on the SE(2) geodesic that the planner
// uses, which is the whole point of the normalized representation.
TEST_F(RoboPlanToppraPlanarTest, TracksSE2Geodesic) {
  JointPath path;
  path.joint_names = joint_names_;
  const double theta_end = 0.8;
  path.positions = {makePoint(0.0, 0.0, 0.0), makePoint(1.0, 0.5, theta_end)};

  auto toppra = PathParameterizerTOPPRA(scene_, "stretch4_arm");
  auto result = toppra.generate(path, /* dt */ 0.01, SplineFittingMode::Hermite);
  ASSERT_TRUE(result.has_value()) << result.error();

  const auto q_start_full = scene_->toFullJointPositions("stretch4_arm", path.positions.front());
  const auto q_end_full = scene_->toFullJointPositions("stretch4_arm", path.positions.back());
  const auto& model = scene_->getModel();
  const auto base_q = model.idx_qs[model.getJointId("mobile_base_planar_joint")];

  for (const auto& pos : result.value().positions) {
    const double theta = std::atan2(pos(3), pos(2));
    const double fraction = theta / theta_end;  // theta is linear along a single segment.
    const auto expected = scene_->interpolate(q_start_full, q_end_full, fraction);
    EXPECT_NEAR(pos(0), expected(base_q), 1.0e-6);      // x
    EXPECT_NEAR(pos(1), expected(base_q + 1), 1.0e-6);  // y
  }
}

// Verifies that velocity limits are respected. Uses "clean" segments (either pure translation or
// pure rotation) where the world-frame distances used for normalization are exact.
TEST_F(RoboPlanToppraPlanarTest, RespectsVelocityLimits) {
  JointPath path;
  path.joint_names = joint_names_;
  path.positions = {makePoint(0.0, 0.0, 0.0), makePoint(0.8, 0.0, 0.0), makePoint(0.8, 0.0, 0.9),
                    makePoint(0.8, 0.6, 0.9), makePoint(1.4, 0.6, 0.9)};

  auto toppra = PathParameterizerTOPPRA(scene_, "stretch4_arm");
  auto result = toppra.generate(path, /* dt */ 0.01, SplineFittingMode::Cubic);
  ASSERT_TRUE(result.has_value()) << result.error();

  const auto vel_limits = scene_->getVelocityLimitVectors("stretch4_arm");
  ASSERT_TRUE(vel_limits.has_value());
  const auto& upper = vel_limits->second;
  for (const auto& vel : result.value().velocities) {
    ASSERT_EQ(vel.size(), upper.size());
    for (Eigen::Index d = 0; d < vel.size(); ++d) {
      EXPECT_LE(std::abs(vel(d)), upper(d) + 1.0e-2) << "DOF " << d << " exceeds velocity limit.";
    }
  }
}

}  // namespace roboplan
