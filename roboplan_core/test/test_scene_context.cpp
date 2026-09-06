#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_context.hpp>
#include <roboplan_example_models/resources.hpp>

namespace roboplan {

namespace {

/// @brief Number of worker threads used by the concurrency tests.
constexpr int kNumThreads = 8;

/// @brief Number of configurations each worker evaluates.
constexpr int kNumSamples = 40;

}  // namespace

class RoboPlanSceneContextTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto model_prefix = example_models::get_package_models_dir();
    const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const std::vector<std::filesystem::path> package_paths = {
        example_models::get_package_share_dir()};
    scene = std::make_shared<Scene>("test_scene", urdf_path, srdf_path, package_paths);
    scene->setRngSeed(7);
  }

  /// @brief A fixed set of configurations for the workers to evaluate.
  std::vector<Eigen::VectorXd> sampleConfigurations(int count) {
    std::vector<Eigen::VectorXd> configurations;
    configurations.reserve(count);
    for (int i = 0; i < count; ++i) {
      configurations.push_back(scene->randomPositions());
    }
    return configurations;
  }

public:
  std::shared_ptr<Scene> scene;
};

TEST_F(RoboPlanSceneContextTest, ContextAgreesWithScene) {
  // A context is only useful if it is an exact stand-in for the scene's own queries.
  const SceneContext context(*scene);
  for (const auto& q : sampleConfigurations(kNumSamples)) {
    EXPECT_EQ(context.hasCollisions(q), scene->hasCollisions(q));
    EXPECT_TRUE(context.forwardKinematics(q, "tool0")
                    .isApprox(scene->forwardKinematics(q, "tool0"), 1.0e-12));
    EXPECT_TRUE(context.forwardKinematics(q, "tool0", "wrist_1_link")
                    .isApprox(scene->forwardKinematics(q, "tool0", "wrist_1_link"), 1.0e-12));
  }
}

TEST_F(RoboPlanSceneContextTest, DebugCollisionsAgreeWithScene) {
  // The debug path takes the naive all-pairs backend rather than the broadphase tree, so it must
  // still agree with the fast path it replaces -- on the context and against the scene.
  const SceneContext context(*scene);
  for (const auto& q : sampleConfigurations(kNumSamples)) {
    testing::internal::CaptureStdout();
    const bool debug_result = context.hasCollisions(q, /*debug=*/true);
    const std::string output = testing::internal::GetCapturedStdout();

    EXPECT_EQ(debug_result, context.hasCollisions(q, /*debug=*/false));
    EXPECT_EQ(debug_result, scene->hasCollisions(q, /*debug=*/false));

    // Colliding pairs are enumerated on the way out; collision-free configurations stay quiet.
    EXPECT_EQ(debug_result, output.find("Collision detected between") != std::string::npos);
  }
}

TEST_F(RoboPlanSceneContextTest, DebugCollisionsEnumerateEveryPair) {
  // The point of the debug path is that it does not stop at the first collision, so a
  // deliberately colliding configuration should report every pair the scene reports.
  const SceneContext context(*scene);

  Eigen::VectorXd colliding = Eigen::VectorXd::Zero(scene->getModel().nq);
  bool found = false;
  for (const auto& q : sampleConfigurations(500)) {
    if (scene->hasCollisions(q)) {
      colliding = q;
      found = true;
      break;
    }
  }
  ASSERT_TRUE(found) << "test setup: expected at least one colliding sample";

  testing::internal::CaptureStdout();
  EXPECT_TRUE(context.hasCollisions(colliding, /*debug=*/true));
  const std::string context_output = testing::internal::GetCapturedStdout();

  testing::internal::CaptureStdout();
  EXPECT_TRUE(scene->hasCollisions(colliding, /*debug=*/true));
  const std::string scene_output = testing::internal::GetCapturedStdout();

  // Same scratch-independent computation, so the context must enumerate exactly what the scene
  // does.
  EXPECT_EQ(context_output, scene_output);
  EXPECT_NE(context_output.find("Collision detected between"), std::string::npos);
}

TEST_F(RoboPlanSceneContextTest, ConcurrentQueriesMatchSerialResults) {
  // Many threads querying a shared Scene, each through its own context,
  // must produce exactly what a single thread produces.
  // Run this under -fsanitize=thread to also assert that nothing is written concurrently.
  const auto configurations = sampleConfigurations(kNumSamples);

  // Serial reference answers, computed on the scene itself.
  std::vector<char> expected_collisions;
  std::vector<Eigen::Matrix4d> expected_poses;
  expected_collisions.reserve(configurations.size());
  expected_poses.reserve(configurations.size());
  for (const auto& q : configurations) {
    expected_collisions.push_back(static_cast<char>(scene->hasCollisions(q)));
    expected_poses.push_back(scene->forwardKinematics(q, "tool0"));
  }

  std::atomic<int> mismatches{0};
  std::vector<std::thread> workers;
  workers.reserve(kNumThreads);
  for (int t = 0; t < kNumThreads; ++t) {
    workers.emplace_back([&]() {
      // Each thread gets its own scratch over the shared, immutable scene description.
      const SceneContext context(*scene);
      for (size_t i = 0; i < configurations.size(); ++i) {
        const auto& q = configurations.at(i);
        if (static_cast<char>(context.hasCollisions(q)) != expected_collisions.at(i)) {
          ++mismatches;
        }
        if (!context.forwardKinematics(q, "tool0").isApprox(expected_poses.at(i), 1.0e-12)) {
          ++mismatches;
        }
        // Interleave a Jacobian into the same scratch, so a thread that leaked into another's
        // Data would corrupt the frame placements the next FK call reads.
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, scene->getModel().nv);
        context.computeFrameJacobian(q, scene->getFrameId("tool0").value(),
                                     pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
        if (!jacobian.allFinite()) {
          ++mismatches;
        }
      }
    });
  }
  for (auto& worker : workers) {
    worker.join();
  }

  EXPECT_EQ(mismatches.load(), 0);
}

TEST_F(RoboPlanSceneContextTest, SeededContextsAreIndependent) {
  // Two contexts seeded alike draw alike, and neither disturbs the other or the scene.
  // This is what lets two seeded algorithms share a Scene and stay reproducible.
  SceneContext first(*scene);
  SceneContext second(*scene);
  first.setRngSeed(42);
  second.setRngSeed(42);

  const auto scene_before = scene->randomPositions();

  std::vector<Eigen::VectorXd> first_draws;
  std::vector<Eigen::VectorXd> second_draws;
  for (int i = 0; i < 5; ++i) {
    first_draws.push_back(first.randomPositions());
  }
  // Reseeding `second` in between must not perturb `first`, and vice versa.
  second.setRngSeed(42);
  for (int i = 0; i < 5; ++i) {
    second_draws.push_back(second.randomPositions());
  }

  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(first_draws.at(i).isApprox(second_draws.at(i), 1.0e-15));
  }

  // The scene's own generator advanced only for the scene's own draws.
  scene->setRngSeed(7);
  EXPECT_TRUE(scene->randomPositions().isApprox(scene_before, 1.0e-15));
}

TEST_F(RoboPlanSceneContextTest, StaleContextIsRejected) {
  // A context sizes its scratch to the geometry it snapshotted. Once the scene's geometry changes,
  // it must say so rather than index that scratch with the new pair count.
  const SceneContext context(*scene);
  const auto q = scene->randomPositions();
  ASSERT_NO_THROW(context.hasCollisions(q));
  EXPECT_TRUE(context.isGeometryCurrent());

  ASSERT_TRUE(scene
                  ->addBoxGeometry("obstacle", "universe", Box(0.2, 0.2, 0.2),
                                   Eigen::Matrix4d::Identity(), Eigen::Vector4d(1.0, 0.0, 0.0, 1.0))
                  .has_value());

  EXPECT_FALSE(context.isGeometryCurrent());
  EXPECT_THROW(context.hasCollisions(q), std::runtime_error);
  EXPECT_THROW(context.computeDistances(q), std::runtime_error);

  // A fresh context sees the new geometry and works again.
  const SceneContext rebuilt(*scene);
  EXPECT_NO_THROW(rebuilt.hasCollisions(q));
}

TEST_F(RoboPlanSceneContextTest, MovingGeometryDoesNotInvalidateContexts) {
  // Repositioning an existing obstacle resizes nothing, and contexts read placements from the
  // model they share with the scene. Invalidating them here would break the common case of moving
  // an obstacle every control cycle.
  ASSERT_TRUE(scene
                  ->addBoxGeometry("obstacle", "universe", Box(0.2, 0.2, 0.2),
                                   Eigen::Matrix4d::Identity(), Eigen::Vector4d(1.0, 0.0, 0.0, 1.0))
                  .has_value());

  const SceneContext context(*scene);
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
  tform(0, 3) = 1.5;
  ASSERT_TRUE(scene->updateGeometryPlacement("obstacle", "universe", tform).has_value());

  EXPECT_TRUE(context.isGeometryCurrent());
  const auto q = scene->randomPositions();
  EXPECT_NO_THROW(context.hasCollisions(q));
  EXPECT_EQ(context.hasCollisions(q), scene->hasCollisions(q));
}

}  // namespace roboplan
