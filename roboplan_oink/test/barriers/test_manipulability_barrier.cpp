#include <gtest/gtest.h>
#include <memory>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/barriers/manipulability_barrier.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

class ManipulabilityBarrierTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    scene_ = std::make_shared<Scene>(
        "test_scene", model_prefix / "ur_robot_model" / "ur5_gripper.urdf",
        model_prefix / "ur_robot_model" / "ur5_gripper.srdf",
        std::vector<std::filesystem::path>{example_models::get_package_share_dir()},
        model_prefix / "ur_robot_model" / "ur5_config.yaml");
    oink_ = std::make_shared<Oink>(*scene_);
    num_variables_ = scene_->getModel().nv;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
    scene_->setJointPositions(q);
  }

  std::shared_ptr<Scene> scene_;
  std::shared_ptr<Oink> oink_;
  int num_variables_;
  static constexpr double kDt = 0.01;
  static constexpr double kSigmaSafe = 0.01;
  static constexpr double kFdEpsilon = 1e-6;
};

TEST_F(ManipulabilityBarrierTest, ConstructionStoresParameters) {
  auto barrier = std::make_shared<ManipulabilityBarrier>(*oink_, *scene_, "tool0", kDt, kSigmaSafe,
                                                         /*gain=*/2.0,
                                                         /*safe_displacement_gain=*/0.5,
                                                         /*safety_margin=*/0.01, kFdEpsilon);
  EXPECT_EQ(barrier->getNumBarriers(*scene_), 1);
  EXPECT_EQ(barrier->frame_name, "tool0");
  EXPECT_DOUBLE_EQ(barrier->sigma_safe, kSigmaSafe);
  EXPECT_DOUBLE_EQ(barrier->fd_epsilon, kFdEpsilon);
  EXPECT_DOUBLE_EQ(barrier->gain, 2.0);
  EXPECT_DOUBLE_EQ(barrier->dt, kDt);
  EXPECT_DOUBLE_EQ(barrier->safe_displacement_gain, 0.5);
  EXPECT_DOUBLE_EQ(barrier->safety_margin, 0.01);
}

TEST_F(ManipulabilityBarrierTest, InvalidSigmaSafe) {
  EXPECT_THROW(
      { ManipulabilityBarrier(*oink_, *scene_, "tool0", kDt, /*sigma_safe=*/-0.01); },
      std::invalid_argument);
}

TEST_F(ManipulabilityBarrierTest, InvalidFdEpsilon) {
  EXPECT_THROW(
      {
        ManipulabilityBarrier(*oink_, *scene_, "tool0", kDt, kSigmaSafe, 1.0, 1.0, 0.0,
                              /*fd_epsilon=*/0.0);
      },
      std::invalid_argument);
  EXPECT_THROW(
      {
        ManipulabilityBarrier(*oink_, *scene_, "tool0", kDt, kSigmaSafe, 1.0, 1.0, 0.0,
                              /*fd_epsilon=*/-1e-6);
      },
      std::invalid_argument);
}

TEST_F(ManipulabilityBarrierTest, InvalidFrameName) {
  EXPECT_THROW(
      { ManipulabilityBarrier(*oink_, *scene_, "nonexistent_frame", kDt, kSigmaSafe); },
      std::runtime_error);
}

TEST_F(ManipulabilityBarrierTest, BarrierValuePositiveAtSafeConfiguration) {
  // UR5 at zero configuration is not singular; h = sigma_min - sigma_safe > 0.
  auto barrier = std::make_shared<ManipulabilityBarrier>(*oink_, *scene_, "tool0", kDt,
                                                         /*sigma_safe=*/0.0);
  auto result = barrier->computeBarrier(*scene_);
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_GT(barrier->barrier_values[0], 0.0);
}

TEST_F(ManipulabilityBarrierTest, JacobianHasExpectedDimensions) {
  auto barrier = std::make_shared<ManipulabilityBarrier>(*oink_, *scene_, "tool0", kDt, kSigmaSafe);
  ASSERT_TRUE(barrier->computeBarrier(*scene_).has_value());
  ASSERT_TRUE(barrier->computeJacobian(*scene_).has_value());

  EXPECT_EQ(barrier->jacobian_container.rows(), 1);
  EXPECT_EQ(barrier->jacobian_container.cols(), num_variables_);
  EXPECT_TRUE(barrier->jacobian_container.allFinite());
}

TEST_F(ManipulabilityBarrierTest, QpInequalitiesAreFinite) {
  auto barrier = std::make_shared<ManipulabilityBarrier>(*oink_, *scene_, "tool0", kDt, kSigmaSafe);
  Eigen::MatrixXd G(1, num_variables_);
  Eigen::VectorXd b(1);

  auto result = barrier->computeQpInequalities(*scene_, G, b);
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_TRUE(G.allFinite());
  EXPECT_TRUE(b.allFinite());
}

TEST_F(ManipulabilityBarrierTest, EvaluateAtConfigurationMatchesBarrier) {
  auto barrier = std::make_shared<ManipulabilityBarrier>(*oink_, *scene_, "tool0", kDt,
                                                         /*sigma_safe=*/0.0);
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  ASSERT_TRUE(barrier->computeBarrier(*scene_).has_value());

  pinocchio::Data temp_data(scene_->getModel());
  auto eval = barrier->evaluateAtConfiguration(scene_->getModel(), temp_data, q);
  ASSERT_TRUE(eval.has_value()) << eval.error();
  EXPECT_NEAR(eval.value(), barrier->barrier_values[0], 1e-6);
}

}  // namespace roboplan

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
