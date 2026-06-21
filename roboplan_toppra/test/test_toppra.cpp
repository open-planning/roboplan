#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_toppra/linear_blend_path.hpp>
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
    // Load the YAML config so the scene carries realistic joint acceleration limits (the URDF
    // alone has none); the LinearBlend limit-respect test relies on them.
    const auto yaml_path = model_prefix / "ur_robot_model" / "ur5_config.yaml";
    const std::vector<std::filesystem::path> package_paths = {
        example_models::get_package_share_dir()};
    scene = std::make_shared<Scene>("test_scene", urdf_path, srdf_path, package_paths, yaml_path);
  }

public:
  // No default constructors, so must be pointers.
  std::shared_ptr<Scene> scene;
};

TEST_F(RoboPlanToppraTest, EmptyPath) {
  JointPath path;
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt});
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), "Path must have at least 2 points.");
}

TEST_F(RoboPlanToppraTest, BadJointNames) {
  auto path = createTestPathShort();
  path.joint_names = {"fr3_joint1", "fr3_joint2"};
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt});
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), "Path joint names do not match the scene joint names.");
}

TEST_F(RoboPlanToppraTest, NegativeDt) {
  auto path = createTestPathShort();
  double dt = -0.1;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt});
  ASSERT_FALSE(result.has_value());
  ASSERT_EQ(result.error(), "dt must be strictly positive.");
}

TEST_F(RoboPlanToppraTest, BadVelocityAccelerationScales) {
  auto path = createTestPathShort();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");

  for (const auto& vel_scale : std::vector<double>{-0.1, 0.0, 1.1}) {
    auto result = toppra.generate(path, {dt, SplineFittingMode::Hermite, vel_scale});
    ASSERT_FALSE(result.has_value());
    ASSERT_EQ(result.error(),
              "Velocity scale must be greater than 0.0 and less than or equal to 1.0.");
  }

  for (const auto& acc_scale : std::vector<double>{-0.1, 0.0, 1.1}) {
    auto result =
        toppra.generate(path, {dt, SplineFittingMode::Hermite, /* vel_scale */ 0.5, acc_scale});
    ASSERT_FALSE(result.has_value());
    ASSERT_EQ(result.error(),
              "Acceleration scale must be greater than 0.0 and less than or equal to 1.0.");
  }
}

TEST_F(RoboPlanToppraTest, ShortPathHermite) {
  auto path = createTestPathShort();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt, SplineFittingMode::Hermite});
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, LongPathHermite) {
  auto path = createTestPathLong();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt, SplineFittingMode::Hermite});
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, ShortPathCubic) {
  auto path = createTestPathShort();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt, SplineFittingMode::Cubic});
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, LongPathCubic) {
  auto path = createTestPathLong();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt, SplineFittingMode::Cubic});
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, LongPathAdaptive) {
  auto path = createTestPathLong();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt, SplineFittingMode::Adaptive});
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, ShortPathLinearBlend) {
  auto path = createTestPathShort();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt, SplineFittingMode::LinearBlend});
  ASSERT_TRUE(result.has_value());
}

TEST_F(RoboPlanToppraTest, LongPathLinearBlend) {
  auto path = createTestPathLong();
  double dt = 0.01;

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  auto result = toppra.generate(path, {dt, SplineFittingMode::LinearBlend});
  ASSERT_TRUE(result.has_value());
}

namespace {
/// @brief Peak |value| / |limit| ratio across a trajectory's per-step vectors.
double peakRatio(const std::vector<Eigen::VectorXd>& values, const Eigen::VectorXd& limit) {
  double ratio = 0.0;
  for (const auto& value : values) {
    for (Eigen::Index i = 0; i < value.size(); ++i) {
      if (std::abs(limit(i)) > 1e-9) {
        ratio = std::max(ratio, std::abs(value(i)) / std::abs(limit(i)));
      }
    }
  }
  return ratio;
}
}  // namespace

// A densely sampled sharp corner is the input that makes the interpolating cubic spline crawl:
// the spline overshoots near the corner, inflating curvature across the many dense knots. The
// LinearBlend geometry keeps the legs perfectly straight (zero curvature) and rounds the corner
// within a bounded radius, so it should (a) respect the joint acceleration limit and (b) finish
// much faster than the cubic spline. This distills what the Cartesian planner sees per corner.
TEST_F(RoboPlanToppraTest, LinearBlendIsFastAndRespectsLimitsOnSharpCorner) {
  const double dt = 0.01;
  JointPath path;
  path.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};
  Eigen::VectorXd q0(6);
  q0 << -1.8, -2.0, 1.0, -1.6, -1.7, 1.0;
  Eigen::VectorXd q_corner(6);
  q_corner << -1.0, -1.2, 0.4, -1.0, -1.1, 0.5;
  Eigen::VectorXd q1(6);
  q1 << -1.4, -1.6, 1.1, -0.4, -1.6, 1.2;  // second leg heads off in a different direction

  // Densely sample each straight leg (collinear points the blend collapses into one segment).
  constexpr int kPerLeg = 75;
  const auto sample_leg = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                              bool include_end) {
    for (int i = 0; i < kPerLeg + (include_end ? 1 : 0); ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(kPerLeg);
      path.positions.push_back((a + t * (b - a)).eval());
    }
  };
  sample_leg(q0, q_corner, /*include_end=*/false);
  sample_leg(q_corner, q1, /*include_end=*/true);

  auto toppra = PathParameterizerTOPPRA(scene, "arm");
  const double max_deviation = 0.05;
  auto blend = toppra.generate(
      path, {dt, SplineFittingMode::LinearBlend, 1.0, 1.0, 10, 0.05, max_deviation});
  auto cubic = toppra.generate(path, {dt, SplineFittingMode::Cubic});
  ASSERT_TRUE(blend.has_value());
  ASSERT_TRUE(cubic.has_value());

  // The blend trajectory respects the joint acceleration limit (a small overshoot is the
  // intrinsic curvature*velocity^2 term on the rounded corner, where path acceleration and
  // centripetal acceleration briefly add).
  const Eigen::VectorXd accel_limit =
      scene->getAccelerationLimitVectors("arm").value().second.cwiseAbs();
  const Eigen::VectorXd vel_limit = scene->getVelocityLimitVectors("arm").value().second.cwiseAbs();
  EXPECT_LT(peakRatio(blend->accelerations, accel_limit), 1.2);
  EXPECT_LT(peakRatio(blend->velocities, vel_limit), 1.05);

  // And it finishes substantially faster than the curvature-sensitive cubic spline.
  EXPECT_LT(blend->times.back(), cubic->times.back());
}

// Geometry: three collinear waypoints reduce to a single straight line (zero curvature).
TEST(LinearBlendPathTest, StraightLineHasZeroCurvature) {
  Eigen::VectorXd a(2), b(2), c(2);
  a << 0.0, 0.0;
  b << 1.0, 0.0;
  c << 2.0, 0.0;
  const toppra::Vectors waypoints = {a, b, c};
  LinearBlendPath path(waypoints, /*max_deviation=*/0.1);

  const auto interval = path.pathInterval();
  EXPECT_NEAR(interval[0], 0.0, 1e-9);
  EXPECT_NEAR(interval[1], 2.0, 1e-9);  // collinear -> exact straight-line distance
  EXPECT_NEAR((path.eval_single(0.0, 0) - a).norm(), 0.0, 1e-9);
  EXPECT_NEAR((path.eval_single(interval[1], 0) - c).norm(), 0.0, 1e-9);
  for (double s = 0.0; s <= interval[1] + 1e-12; s += 0.1) {
    EXPECT_LT(path.eval_single(s, 2).norm(), 1e-9);         // zero curvature
    EXPECT_NEAR(path.eval_single(s, 1).norm(), 1.0, 1e-9);  // unit tangent
  }
}

// Geometry: a 90-degree corner is rounded within max_deviation, with continuous position and
// unit tangent across the line<->arc junctions.
TEST(LinearBlendPathTest, CornerBlendIsContinuousAndBounded) {
  Eigen::VectorXd a(2), b(2), c(2);
  a << 0.0, 0.0;
  b << 1.0, 0.0;
  c << 1.0, 1.0;
  const double max_deviation = 0.1;
  const toppra::Vectors waypoints = {a, b, c};
  LinearBlendPath path(waypoints, max_deviation);

  const auto interval = path.pathInterval();
  EXPECT_NEAR((path.eval_single(0.0, 0) - a).norm(), 0.0, 1e-9);
  EXPECT_NEAR((path.eval_single(interval[1], 0) - c).norm(), 0.0, 1e-9);
  EXPECT_GT(interval[1], 0.0);
  EXPECT_LT(interval[1], 2.0);  // corner is cut, so shorter than the polyline

  const double ds = interval[1] / 400.0;
  Eigen::VectorXd previous = path.eval_single(0.0, 0);
  double min_corner_distance = (previous - b).norm();
  for (double s = ds; s <= interval[1] + 1e-12; s += ds) {
    const auto position = path.eval_single(s, 0);
    EXPECT_LT((position - previous).norm(), 5.0 * ds);      // no positional jumps
    EXPECT_NEAR(path.eval_single(s, 1).norm(), 1.0, 1e-6);  // unit tangent everywhere
    min_corner_distance = std::min(min_corner_distance, (position - b).norm());
    previous = position;
  }
  // The blend's closest approach to the sharp corner equals the requested deviation.
  EXPECT_NEAR(min_corner_distance, max_deviation, 1e-3);
}

TEST(LinearBlendPathTest, RequiresAtLeastTwoWaypoints) {
  Eigen::VectorXd a(2);
  a << 0.0, 0.0;
  const toppra::Vectors waypoints = {a};
  EXPECT_THROW(LinearBlendPath(waypoints, 0.1), std::invalid_argument);
}

}  // namespace roboplan
