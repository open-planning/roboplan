#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <stdexcept>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_oink/barriers/self_collision_barrier.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <test_utils.hpp>

namespace {
constexpr double kTolerance = 1e-6;
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
    const auto description = loadUrdfSceneDescription(urdf_path_, package_paths_);
    scene_ = std::make_shared<Scene>("test_scene", description);
    scene_->importJointLimitsFromConfig(loadJointLimitsConfig(yaml_config_path_));
    if (const auto imported = scene_->importSrdf(loadTextFile(srdf_path_)); !imported) {
      throw std::runtime_error(imported.error());
    }
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

  auto barrier = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_,
      SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_,
                                  .gain = 2.5,
                                  .safe_displacement_gain = 0.5,
                                  .d_min = 0.03,
                                  .safety_margin = 0.01});

  EXPECT_EQ(barrier->getNumBarriers(posed(*oink_, *scene_)), num_pairs_);
  EXPECT_EQ(barrier->n_collision_pairs, num_pairs_);
  EXPECT_DOUBLE_EQ(barrier->d_min, 0.03);
  EXPECT_DOUBLE_EQ(barrier->gain, 2.5);
  EXPECT_DOUBLE_EQ(barrier->dt, dt_);
  EXPECT_DOUBLE_EQ(barrier->safe_displacement_gain, 0.5);
  EXPECT_DOUBLE_EQ(barrier->safety_margin, 0.01);
}

TEST_F(SelfCollisionBarrierTest, ConstructionDefaultsAreReasonable) {
  auto barrier = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_, SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_});

  EXPECT_DOUBLE_EQ(barrier->d_min, 0.02);
  EXPECT_DOUBLE_EQ(barrier->gain, 1.0);
  EXPECT_DOUBLE_EQ(barrier->safe_displacement_gain, 1.0);
  EXPECT_DOUBLE_EQ(barrier->safety_margin, 0.0);
}

TEST_F(SelfCollisionBarrierTest, InvalidDmin) {
  EXPECT_THROW(
      {
        auto barrier = std::make_shared<SelfCollisionBarrier>(
            *oink_, *scene_, dt_,
            SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_, .d_min = -0.01});
      },
      std::invalid_argument);
}

TEST_F(SelfCollisionBarrierTest, InvalidPairCount) {
  // Non-positive pair counts are rejected.
  EXPECT_THROW(
      {
        auto barrier = std::make_shared<SelfCollisionBarrier>(
            *oink_, *scene_, dt_, SelfCollisionBarrierOptions{.n_collision_pairs = 0});
      },
      std::invalid_argument);
  EXPECT_THROW(
      {
        auto barrier = std::make_shared<SelfCollisionBarrier>(
            *oink_, *scene_, dt_, SelfCollisionBarrierOptions{.n_collision_pairs = -1});
      },
      std::invalid_argument);
}

TEST_F(SelfCollisionBarrierTest, PairCountClippedToSceneCount) {
  // Requesting more pairs than exist in the model is clipped, not rejected.
  auto barrier = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_, SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_ + 5});
  EXPECT_EQ(barrier->n_collision_pairs, num_pairs_);
  EXPECT_EQ(barrier->getNumBarriers(posed(*oink_, *scene_)), num_pairs_);
}

TEST_F(SelfCollisionBarrierTest, InvalidGainAndDt) {
  EXPECT_THROW(
      {
        auto barrier = std::make_shared<SelfCollisionBarrier>(
            *oink_, *scene_, dt_,
            SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_, .gain = 0.0});
      },
      std::invalid_argument);
  EXPECT_THROW(
      {
        auto barrier = std::make_shared<SelfCollisionBarrier>(
            *oink_, *scene_, /*dt=*/0.0,
            SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_});
      },
      std::invalid_argument);
}

TEST_F(SelfCollisionBarrierTest, BarrierValuesPositiveInSafeConfiguration) {
  // Zero configuration is collision free for the UR5 example.
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  ASSERT_FALSE(scene_->hasCollisions(q));

  auto barrier = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_,
      SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_, .d_min = 0.0});
  auto result = barrier->computeBarrier(posed(*oink_, *scene_));
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
  auto barrier = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_,
      SelfCollisionBarrierOptions{.n_collision_pairs = requested, .d_min = 0.0});
  auto result = barrier->computeBarrier(posed(*oink_, *scene_));
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

  auto barrier_no_margin = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_,
      SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_, .d_min = 0.0});
  auto barrier_with_margin = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_,
      SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_, .d_min = 0.05});

  ASSERT_TRUE(barrier_no_margin->computeBarrier(posed(*oink_, *scene_)).has_value());
  ASSERT_TRUE(barrier_with_margin->computeBarrier(posed(*oink_, *scene_)).has_value());

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

  auto barrier = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_, SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_});
  ASSERT_TRUE(barrier->computeBarrier(posed(*oink_, *scene_)).has_value());
  ASSERT_TRUE(barrier->computeJacobian(posed(*oink_, *scene_)).has_value());

  EXPECT_EQ(barrier->jacobian_container.rows(), num_pairs_);
  EXPECT_EQ(barrier->jacobian_container.cols(), num_variables_);
  EXPECT_TRUE(barrier->jacobian_container.allFinite());
}

TEST_F(SelfCollisionBarrierTest, QpInequalitiesAreFinite) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);

  auto barrier = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_,
      SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_, .gain = 5.0});
  const int n = barrier->getNumBarriers(posed(*oink_, *scene_));
  Eigen::MatrixXd G(n, num_variables_);
  Eigen::VectorXd b(n);

  auto result = barrier->computeQpInequalities(posed(*oink_, *scene_), G, b);
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

  auto barrier = std::make_shared<SelfCollisionBarrier>(
      *oink_, *scene_, dt_,
      SelfCollisionBarrierOptions{.n_collision_pairs = num_pairs_, .d_min = 0.01});
  ASSERT_TRUE(barrier->computeBarrier(posed(*oink_, *scene_)).has_value());

  pinocchio::Data temp_data(scene_->getModel());
  auto eval_result = barrier->evaluateAtConfiguration(scene_->getModel(), temp_data, q);
  ASSERT_TRUE(eval_result.has_value()) << eval_result.error();

  // When all pairs are constrained, the evaluation matches the smallest barrier value.
  EXPECT_NEAR(eval_result.value(), barrier->barrier_values.minCoeff(), 1e-6);
}

TEST_F(SelfCollisionBarrierTest, SurvivesContextRefreshAfterGeometryChange) {
  SelfCollisionBarrierOptions options;
  options.n_collision_pairs = 4;
  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, dt_, options);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  ASSERT_TRUE(barrier->computeBarrier(posed(*oink_, *scene_)).has_value());

  // Adding geometry adds collision pairs, which strands every context snapshotted before it.
  const Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  const Eigen::Vector4d color(1.0, 0.0, 0.0, 1.0);
  ASSERT_TRUE(scene_->addSphereGeometry("probe", "tool0", Sphere(0.05), tform, color).has_value());
  EXPECT_FALSE(oink_->getContext().isGeometryCurrent());

  // Refreshing the solver's context is enough; the barrier is not rebuilt.
  oink_->refreshContext(*scene_);
  EXPECT_TRUE(oink_->getContext().isGeometryCurrent());

  scene_->setJointPositions(q);
  ASSERT_TRUE(barrier->computeBarrier(posed(*oink_, *scene_)).has_value());
  ASSERT_TRUE(barrier->computeJacobian(posed(*oink_, *scene_)).has_value());
  EXPECT_TRUE(barrier->barrier_values.allFinite());
  EXPECT_TRUE(barrier->jacobian_container.allFinite());
}

TEST_F(SelfCollisionBarrierTest, ResizesWorkspaceWhenPairCountGrows) {
  // The distance workspace is sized from the collision-pair count. computeBarrier() writes one
  // entry per pair, so a grown geometry would run past a workspace frozen at construction.
  SelfCollisionBarrierOptions options;
  options.n_collision_pairs = num_pairs_ + 8;  // clipped down at construction
  auto barrier = std::make_shared<SelfCollisionBarrier>(*oink_, *scene_, dt_, options);
  EXPECT_EQ(barrier->n_collision_pairs, num_pairs_);
  EXPECT_EQ(barrier->all_distances.size(), num_pairs_);

  const Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  const Eigen::Vector4d color(0.0, 1.0, 0.0, 1.0);
  ASSERT_TRUE(scene_->addSphereGeometry("probe", "tool0", Sphere(0.05), tform, color).has_value());
  oink_->refreshContext(*scene_);

  const int grown_pairs = static_cast<int>(scene_->getCollisionModel().collisionPairs.size());
  ASSERT_GT(grown_pairs, num_pairs_) << "test setup: adding geometry should add collision pairs";

  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_variables_);
  scene_->setJointPositions(q);
  ASSERT_TRUE(barrier->computeBarrier(posed(*oink_, *scene_)).has_value());

  // The workspace follows the new pair count, and the dimension re-clips toward the request.
  EXPECT_EQ(barrier->all_distances.size(), grown_pairs);
  EXPECT_EQ(barrier->n_collision_pairs, std::min(num_pairs_ + 8, grown_pairs));
  EXPECT_EQ(barrier->getNumBarriers(posed(*oink_, *scene_)), barrier->n_collision_pairs);
  EXPECT_EQ(static_cast<int>(barrier->closest_pair_indices.size()), barrier->n_collision_pairs);
  EXPECT_TRUE(barrier->barrier_values.allFinite());
}

}  // namespace roboplan

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
