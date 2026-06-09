#include <gtest/gtest.h>
#include <limits>
#include <memory>

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/barriers/self_collision_barrier.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/frame.hpp>

namespace {
constexpr double kTolerance = 1e-6;

roboplan::CartesianConfiguration makeCartesianConfig(const std::string& frame_name,
                                                     const Eigen::Vector3d& position,
                                                     const Eigen::Quaterniond& orientation) {
  roboplan::CartesianConfiguration config;
  config.tip_frame = frame_name;
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform.block<3, 3>(0, 0) = orientation.toRotationMatrix();
  tform.block<3, 1>(0, 3) = position;
  config.tform = tform;
  return config;
}
}  // namespace

namespace roboplan {

class SelfCollisionBarrierTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    urdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    srdf_path_ = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    package_paths_ = {example_models::get_package_share_dir()};
    yaml_config_path_ = model_prefix / "ur_robot_model" / "ur5_config.yaml";
    scene_ = std::make_shared<Scene>("test_scene", urdf_path_, srdf_path_, package_paths_,
                                     yaml_config_path_);
    oink_ = std::make_shared<Oink>(*scene_);

    num_variables_ = scene_->getModel().nv;
    num_pairs_ = static_cast<int>(scene_->getCollisionModel().collisionPairs.size());
    dt_ = 0.01;
  }

  std::shared_ptr<Scene> scene_;
  std::shared_ptr<Oink> oink_;
  std::filesystem::path urdf_path_;
  std::filesystem::path srdf_path_;
  std::vector<std::filesystem::path> package_paths_;
  std::filesystem::path yaml_config_path_;
  int num_variables_;
  int num_pairs_;
  double dt_;
};

TEST_F(SelfCollisionBarrierTest, ConstructionStoresParameters) {
  ASSERT_GT(num_pairs_, 0) << "Test scene must have at least one collision pair";

  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_,
                                                        /*gain=*/2.5,
                                                        /*safe_displacement_gain=*/0.5,
                                                        /*d_min=*/0.03,
                                                        /*safety_margin=*/0.01);

  EXPECT_EQ(barrier->getNumBarriers(*scene_), num_pairs_);
  EXPECT_EQ(barrier->n_collision_pairs, num_pairs_);
  EXPECT_DOUBLE_EQ(barrier->d_min, 0.03);
  EXPECT_DOUBLE_EQ(barrier->gain, 2.5);
  EXPECT_DOUBLE_EQ(barrier->dt, dt_);
  EXPECT_DOUBLE_EQ(barrier->safe_displacement_gain, 0.5);
  EXPECT_DOUBLE_EQ(barrier->safety_margin, 0.01);
}

TEST_F(SelfCollisionBarrierTest, ConstructionDefaultsAreReasonable) {
  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_);

  EXPECT_DOUBLE_EQ(barrier->d_min, 0.02);
  EXPECT_DOUBLE_EQ(barrier->gain, 1.0);
  EXPECT_DOUBLE_EQ(barrier->safe_displacement_gain, 1.0);
  EXPECT_DOUBLE_EQ(barrier->safety_margin, 0.0);
}

TEST_F(SelfCollisionBarrierTest, InvalidDmin) {
  EXPECT_THROW(
      {
        auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_, 1.0,
                                                              1.0, /*d_min=*/-0.01);
      },
      std::invalid_argument);
}

TEST_F(SelfCollisionBarrierTest, InvalidPairCount) {
  // Non-positive pair counts are rejected.
  EXPECT_THROW(
      { auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, 0, dt_); },
      std::invalid_argument);
  EXPECT_THROW(
      { auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, -1, dt_); },
      std::invalid_argument);

  // Requesting more pairs than exist in the model is rejected.
  EXPECT_THROW(
      {
        auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_ + 1, dt_);
      },
      std::invalid_argument);
}

TEST_F(SelfCollisionBarrierTest, InvalidGainAndDt) {
  EXPECT_THROW(
      {
        auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_,
                                                              /*gain=*/0.0);
      },
      std::invalid_argument);
  EXPECT_THROW(
      { auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, 0.0); },
      std::invalid_argument);
}

TEST_F(SelfCollisionBarrierTest, BarrierValuesPositiveInSafeConfiguration) {
  // Zero configuration is collision free for the UR5 example.
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  ASSERT_FALSE(scene_->hasCollisions(q));

  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_,
                                                        /*gain=*/1.0,
                                                        /*safe_displacement_gain=*/1.0,
                                                        /*d_min=*/0.0);
  auto result = barrier->computeBarrier(*scene_);
  ASSERT_TRUE(result.has_value()) << result.error();

  EXPECT_EQ(barrier->barrier_values.size(), num_pairs_);
  EXPECT_TRUE((barrier->barrier_values.array() > 0.0).all())
      << "All barrier values should be positive (no contact) at the zero configuration: "
      << barrier->barrier_values.transpose();
}

TEST_F(SelfCollisionBarrierTest, ClosestPairsAreSelectedFirst) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  // Pick a strict subset to force pair selection.
  const int requested = std::min(num_pairs_, std::max(1, num_pairs_ / 2));
  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, requested, dt_, 1.0, 1.0,
                                                        /*d_min=*/0.0);
  auto result = barrier->computeBarrier(*scene_);
  ASSERT_TRUE(result.has_value()) << result.error();

  ASSERT_EQ(static_cast<int>(barrier->closest_pair_indices.size()), requested);

  // Selected pairs should have the smallest distances overall.
  double max_selected = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < requested; ++i) {
    const auto k = barrier->closest_pair_indices[i];
    max_selected = std::max(max_selected, barrier->all_distances[static_cast<int>(k)]);
  }

  // Any non-selected pair should have distance >= max selected.
  for (int k = 0; k < num_pairs_; ++k) {
    const bool was_selected =
        std::find(barrier->closest_pair_indices.begin(), barrier->closest_pair_indices.end(),
                  static_cast<std::size_t>(k)) != barrier->closest_pair_indices.end();
    if (!was_selected) {
      EXPECT_GE(barrier->all_distances[k], max_selected - kTolerance)
          << "Non-selected pair " << k << " distance " << barrier->all_distances[k]
          << " is below max selected " << max_selected;
    }
  }
}

TEST_F(SelfCollisionBarrierTest, DminShiftsBarrierValues) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  auto barrier_no_margin = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_,
                                                                  1.0, 1.0, /*d_min=*/0.0);
  auto barrier_with_margin = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_,
                                                                    dt_, 1.0, 1.0, /*d_min=*/0.05);

  ASSERT_TRUE(barrier_no_margin->computeBarrier(*scene_).has_value());
  ASSERT_TRUE(barrier_with_margin->computeBarrier(*scene_).has_value());

  // For the same pairs (assuming deterministic ordering), the margin barrier is exactly
  // 0.05 less than the unshifted barrier — values just compare at the per-pair level.
  for (int i = 0; i < num_pairs_; ++i) {
    EXPECT_NEAR(barrier_with_margin->barrier_values[i], barrier_no_margin->barrier_values[i] - 0.05,
                kTolerance);
  }
}

TEST_F(SelfCollisionBarrierTest, JacobianHasExpectedDimensions) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_);
  ASSERT_TRUE(barrier->computeBarrier(*scene_).has_value());
  ASSERT_TRUE(barrier->computeJacobian(*scene_).has_value());

  EXPECT_EQ(barrier->jacobian_container.rows(), num_pairs_);
  EXPECT_EQ(barrier->jacobian_container.cols(), num_variables_);
  EXPECT_TRUE(barrier->jacobian_container.allFinite());
}

TEST_F(SelfCollisionBarrierTest, QpInequalitiesAreFinite) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_,
                                                        /*gain=*/5.0);
  const int n = barrier->getNumBarriers(*scene_);
  Eigen::MatrixXd G(n, num_variables_);
  Eigen::VectorXd b(n);

  auto result = barrier->computeQpInequalities(*scene_, G, b);
  ASSERT_TRUE(result.has_value()) << result.error();

  EXPECT_EQ(G.rows(), n);
  EXPECT_EQ(G.cols(), num_variables_);
  EXPECT_EQ(b.size(), n);
  EXPECT_TRUE(G.allFinite());
  EXPECT_TRUE(b.allFinite());
}

TEST_F(SelfCollisionBarrierTest, EvaluateAtConfigurationMatchesBarrierMinimum) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_, 1.0, 1.0,
                                                        /*d_min=*/0.01);
  ASSERT_TRUE(barrier->computeBarrier(*scene_).has_value());

  pinocchio::Data temp_data(scene_->getModel());
  auto eval_result = barrier->evaluateAtConfiguration(scene_->getModel(), temp_data, q);
  ASSERT_TRUE(eval_result.has_value()) << eval_result.error();

  // When all pairs are constrained, the evaluation matches the smallest barrier value.
  EXPECT_NEAR(eval_result.value(), barrier->barrier_values.minCoeff(), 1e-6);
}

TEST_F(SelfCollisionBarrierTest, IkSolvesWithBarrier) {
  // Verify that solveIk() runs end to end with the self-collision barrier.
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  scene_->forwardKinematics(q, "tool0");

  Eigen::Matrix4d current_pose = scene_->forwardKinematics(q, "tool0");
  Eigen::Vector3d current_pos = current_pose.block<3, 1>(0, 3);
  Eigen::Quaterniond current_orientation(current_pose.block<3, 3>(0, 0));

  // Modest target offset so the task remains within reach.
  Eigen::Vector3d target_pos = current_pos + Eigen::Vector3d(0.05, 0.05, 0.05);
  auto target_config = makeCartesianConfig("tool0", target_pos, current_orientation);

  Oink oink(*scene_);
  FrameTaskOptions task_params{.task_gain = 0.5, .lm_damping = 0.1};
  auto frame_task = std::make_shared<FrameTask>(oink, *scene_, target_config, task_params);

  Eigen::VectorXd v_max = Eigen::VectorXd::Constant(num_variables_, 1.0);
  auto vel_limit = std::make_shared<VelocityLimit>(oink, dt_, v_max);

  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, num_pairs_, dt_,
                                                        /*gain=*/5.0,
                                                        /*safe_displacement_gain=*/1.0,
                                                        /*d_min=*/0.02);

  std::vector<std::shared_ptr<Task>> tasks = {frame_task};
  std::vector<std::shared_ptr<Constraints>> constraints = {vel_limit};
  std::vector<std::shared_ptr<Barrier>> barriers = {barrier};

  Eigen::VectorXd q_current = q;
  for (int iter = 0; iter < 20; ++iter) {
    scene_->setJointPositions(q_current);
    scene_->forwardKinematics(q_current, "tool0");

    Eigen::VectorXd delta_q(num_variables_);
    auto result = oink.solveIk(*scene_, tasks, constraints, barriers, delta_q);
    ASSERT_TRUE(result.has_value()) << "IK failed at iteration " << iter << ": " << result.error();
    q_current = pinocchio::integrate(scene_->getModel(), q_current, delta_q);
  }

  // The robot should not collide with itself at the end of the loop.
  EXPECT_FALSE(scene_->hasCollisions(q_current));
}

}  // namespace roboplan

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
