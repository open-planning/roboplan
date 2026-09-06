#include <memory>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <roboplan/core/robot_body_filter.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>

namespace roboplan {

namespace {

/// A robot made of a single sphere of radius 0.1 fixed at the origin, so the filter's
/// classification can be checked against closed-form distances.
constexpr const char* kBallUrdf = R"(
<robot name="ball_bot">
  <link name="world"/>
  <link name="ball_link">
    <collision>
      <geometry><sphere radius="0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0.0" ixz="0.0" iyz="0.0"/>
    </inertial>
  </link>
  <joint name="world_to_ball" type="fixed">
    <parent link="world"/>
    <child link="ball_link"/>
    <origin xyz="0 0 0"/>
  </joint>
</robot>
)";

constexpr const char* kBallSrdf = R"(<robot name="ball_bot"/>)";

std::shared_ptr<Scene> makeBallScene() {
  return std::make_shared<Scene>("ball_scene", std::string(kBallUrdf), std::string(kBallSrdf));
}

std::shared_ptr<Scene> makeUr5Scene() {
  const auto model_prefix = example_models::get_package_models_dir();
  return std::make_shared<Scene>(
      "ur5_scene", model_prefix / "ur_robot_model" / "ur5_gripper.urdf",
      model_prefix / "ur_robot_model" / "ur5_gripper.srdf",
      std::vector<std::filesystem::path>{example_models::get_package_share_dir()},
      model_prefix / "ur_robot_model" / "ur5_config.yaml");
}

}  // namespace

TEST(RobotBodyFilterTest, NegativePaddingThrows) {
  auto scene = makeBallScene();
  EXPECT_THROW(RobotBodyFilter(scene, RobotBodyFilterOptions{.padding = -0.1}),
               std::invalid_argument);
}

TEST(RobotBodyFilterTest, NarrowphaseMatchesExactSphereDistance) {
  auto scene = makeBallScene();
  RobotBodyFilter filter(
      scene, RobotBodyFilterOptions{.padding = 0.05, .method = RobotBodyFilterMethod::NARROWPHASE});

  // The padded body is the ball of radius 0.1 + 0.05 = 0.15 around the origin.
  RobotBodyFilter::PointMatrix points(4, 3);
  points.row(0) << 0.0, 0.0, 0.0;    // Center: inside.
  points.row(1) << 0.14, 0.0, 0.0;   // Distance 0.14: inside.
  points.row(2) << 0.12, 0.12, 0.0;  // Distance ~0.170: outside the ball, inside its AABB.
  points.row(3) << 0.2, 0.0, 0.0;    // Distance 0.2: outside.

  const auto mask = filter.computeMask(Eigen::VectorXd(0), points);
  EXPECT_TRUE(mask(0));
  EXPECT_TRUE(mask(1));
  EXPECT_FALSE(mask(2));
  EXPECT_FALSE(mask(3));
}

TEST(RobotBodyFilterTest, PaddedObbOverRemovesCorners) {
  auto scene = makeBallScene();
  RobotBodyFilter filter(
      scene, RobotBodyFilterOptions{.padding = 0.05, .method = RobotBodyFilterMethod::PADDED_OBB});

  // The padded OBB is the box of half extent 0.15 around the origin.
  RobotBodyFilter::PointMatrix points(3, 3);
  points.row(0) << 0.14, 0.0, 0.0;   // Inside the ball: removed by both methods.
  points.row(1) << 0.12, 0.12, 0.0;  // Corner region: only the OBB test removes it.
  points.row(2) << 0.2, 0.0, 0.0;    // Outside the box: kept by both methods.

  const auto mask = filter.computeMask(Eigen::VectorXd(0), points);
  EXPECT_TRUE(mask(0));
  EXPECT_TRUE(mask(1));
  EXPECT_FALSE(mask(2));
}

TEST(RobotBodyFilterTest, ExtraPaddingIsAppliedPerPoint) {
  auto scene = makeBallScene();
  RobotBodyFilter filter(
      scene, RobotBodyFilterOptions{.padding = 0.05, .method = RobotBodyFilterMethod::NARROWPHASE});

  // Both points are 0.1 outside the ball surface, past the 0.05 padding; only the one granted
  // 0.1 of extra padding is classified as part of the body.
  RobotBodyFilter::PointMatrix points(2, 3);
  points.row(0) << 0.2, 0.0, 0.0;
  points.row(1) << 0.2, 0.0, 0.0;
  Eigen::VectorXd extra_padding(2);
  extra_padding << 0.0, 0.1;

  const auto mask = filter.computeMask(Eigen::VectorXd(0), points, extra_padding);
  EXPECT_FALSE(mask(0));
  EXPECT_TRUE(mask(1));
}

TEST(RobotBodyFilterTest, FilterPointsKeepsUnmaskedRowsInOrder) {
  auto scene = makeBallScene();
  RobotBodyFilter filter(
      scene, RobotBodyFilterOptions{.padding = 0.05, .method = RobotBodyFilterMethod::NARROWPHASE});

  RobotBodyFilter::PointMatrix points(3, 3);
  points.row(0) << 0.5, 0.0, 0.0;
  points.row(1) << 0.0, 0.0, 0.0;
  points.row(2) << 0.0, 0.6, 0.0;

  const auto kept = filter.filterPoints(Eigen::VectorXd(0), points);
  ASSERT_EQ(kept.rows(), 2);
  EXPECT_TRUE(kept.row(0).isApprox(points.row(0)));
  EXPECT_TRUE(kept.row(1).isApprox(points.row(2)));
}

TEST(RobotBodyFilterTest, PaddedObbMaskIsSupersetOfNarrowphase) {
  auto scene = makeUr5Scene();
  RobotBodyFilter narrowphase_filter(
      scene, RobotBodyFilterOptions{.padding = 0.05, .method = RobotBodyFilterMethod::NARROWPHASE});
  RobotBodyFilter obb_filter(
      scene, RobotBodyFilterOptions{.padding = 0.05, .method = RobotBodyFilterMethod::PADDED_OBB});

  // Random points in a box around the robot at a fixed seed.
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  RobotBodyFilter::PointMatrix points(500, 3);
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    points.row(i) << dist(rng), dist(rng), dist(rng);
  }

  const auto q = scene->getCurrentJointPositions();
  const auto narrowphase_mask = narrowphase_filter.computeMask(q, points);
  const auto obb_mask = obb_filter.computeMask(q, points);

  EXPECT_GT(narrowphase_mask.count(), 0);
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    if (narrowphase_mask(i)) {
      EXPECT_TRUE(obb_mask(i)) << "point " << i << " removed by NARROWPHASE but not PADDED_OBB";
    }
  }
}

TEST(RobotBodyFilterTest, MultiThreadedMaskMatchesSerial) {
  auto scene = makeUr5Scene();

  // A cloud large enough to be split across threads, with the points near the robot clustered
  // together (as in a sensor scan) so the work is unevenly distributed along the array.
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> far_dist(-2.0, 2.0);
  std::uniform_real_distribution<double> near_dist(-0.3, 0.3);
  constexpr Eigen::Index kNumFar = 60000;
  constexpr Eigen::Index kNumNear = 5000;
  RobotBodyFilter::PointMatrix points(kNumFar + kNumNear, 3);
  for (Eigen::Index i = 0; i < kNumFar; ++i) {
    points.row(i) << far_dist(rng), far_dist(rng), far_dist(rng);
  }
  for (Eigen::Index i = kNumFar; i < points.rows(); ++i) {
    points.row(i) << near_dist(rng), near_dist(rng), 0.3 + near_dist(rng);
  }
  Eigen::VectorXd extra_padding = Eigen::VectorXd::Zero(points.rows());
  extra_padding.tail(kNumNear).setConstant(0.02);

  const auto q = scene->getCurrentJointPositions();
  for (const auto method :
       {RobotBodyFilterMethod::NARROWPHASE, RobotBodyFilterMethod::PADDED_OBB}) {
    RobotBodyFilter serial_filter(
        scene, RobotBodyFilterOptions{.padding = 0.05, .method = method, .num_threads = 1});
    RobotBodyFilter threaded_filter(
        scene, RobotBodyFilterOptions{.padding = 0.05, .method = method, .num_threads = 4});

    const auto serial_mask = serial_filter.computeMask(q, points, extra_padding);
    const auto threaded_mask = threaded_filter.computeMask(q, points, extra_padding);
    EXPECT_GT(serial_mask.count(), 0);
    EXPECT_EQ(serial_mask, threaded_mask);
  }
}

TEST(RobotBodyFilterTest, IgnoresGeometryAddedToScene) {
  auto scene = makeUr5Scene();
  const auto num_robot_geoms = scene->getRobotCollisionGeometryIds().size();

  // A point inside an obstacle added to the scene must not be classified as robot body,
  // whether the filter was built before or after the obstacle was added.
  RobotBodyFilter filter_before(scene, RobotBodyFilterOptions{.padding = 0.05});

  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform(0, 3) = 2.0;
  ASSERT_TRUE(scene->addBoxGeometry("obstacle_box", "universe", Box(0.4, 0.4, 0.4), tform,
                                    Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)));
  EXPECT_EQ(scene->getRobotCollisionGeometryIds().size(), num_robot_geoms);

  RobotBodyFilter filter_after(scene, RobotBodyFilterOptions{.padding = 0.05});

  RobotBodyFilter::PointMatrix points(1, 3);
  points.row(0) << 2.0, 0.0, 0.0;

  const auto q = scene->getCurrentJointPositions();
  EXPECT_FALSE(filter_before.computeMask(q, points)(0));
  EXPECT_FALSE(filter_after.computeMask(q, points)(0));
}

}  // namespace roboplan
