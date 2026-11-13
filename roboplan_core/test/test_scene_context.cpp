#include <gtest/gtest.h>
#include <stdexcept>

#include <atomic>
#include <cstddef>
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
    const auto description = loadUrdfSceneDescription(urdf_path, package_paths);
    scene = std::make_shared<Scene>("test_scene", description);
    if (const auto imported = scene->importSrdf(loadTextFile(srdf_path)); !imported) {
      throw std::runtime_error(imported.error());
    }
    scene->setRngSeed(7);
  }

  /// @brief The signed distance of collision pair `pair_index` at `q`, computed on `context`.
  static double pairDistance(const SceneContext& context, const Eigen::VectorXd& q,
                             std::size_t pair_index) {
    context.computeDistances(q);
    return context.getCollisionData().distanceResults.at(pair_index).min_distance;
  }

  /// @brief computeDistanceJacobian() for `pair_index` at `q`, after satisfying its preconditions.
  static Eigen::RowVectorXd distanceJacobian(const SceneContext& context, const Eigen::VectorXd& q,
                                             std::size_t pair_index) {
    context.computeDistances(q);
    context.computeJointJacobians(q);
    Eigen::RowVectorXd jacobian(context.getModel().nv);
    context.computeDistanceJacobian(pair_index, jacobian);
    return jacobian;
  }

  /// @brief Central finite difference of the pair's signed distance along each tangent direction.
  Eigen::RowVectorXd finiteDifferenceJacobian(const SceneContext& context, const Eigen::VectorXd& q,
                                              std::size_t pair_index) {
    // 1e-6 rad: small enough that the O(h^2) truncation error (~1e-12) is negligible, large enough
    // that the distance round-off divided by 2h stays well below the comparison tolerances.
    constexpr double kStep = 1.0e-6;
    const auto nv = context.getModel().nv;
    Eigen::RowVectorXd jacobian(nv);
    for (int i = 0; i < nv; ++i) {
      const Eigen::VectorXd step = kStep * Eigen::VectorXd::Unit(nv, i);
      const double d_plus = pairDistance(context, scene->integrate(q, step), pair_index);
      const double d_minus = pairDistance(context, scene->integrate(q, -step), pair_index);
      jacobian[i] = (d_plus - d_minus) / (2.0 * kStep);
    }
    return jacobian;
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

  // The same pairs are available without printing, named by what setCollisions accepts.
  const auto pairs = scene->getCollidingPairs(colliding);
  ASSERT_FALSE(pairs.empty());
  EXPECT_EQ(pairs, context.getCollidingPairs(colliding));
  for (const auto& pair : pairs) {
    EXPECT_NE(scene_output.find(pair), std::string::npos) << pair;
    const auto separator = pair.find(" <-> ");
    ASSERT_NE(separator, std::string::npos) << pair;
    EXPECT_TRUE(scene->setCollisions(pair.substr(0, separator), pair.substr(separator + 5), false))
        << pair;
  }
  EXPECT_FALSE(scene->hasCollisions(colliding));
  EXPECT_TRUE(scene->getCollidingPairs(colliding).empty());
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

TEST_F(RoboPlanSceneContextTest, AttachAndDetachInvalidateContexts) {
  // Unlike moving an obstacle, attaching and detaching change collision pairs.
  ASSERT_TRUE(scene
                  ->addBoxGeometry("cube", "universe", Box(0.05, 0.05, 0.05),
                                   Eigen::Matrix4d::Identity(), Eigen::Vector4d(1.0, 0.0, 0.0, 1.0))
                  .has_value());

  const SceneContext before_attach(*scene);
  ASSERT_TRUE(scene->attachObject("cube", "tool0").has_value());
  EXPECT_FALSE(before_attach.isGeometryCurrent());

  const SceneContext before_detach(*scene);
  ASSERT_TRUE(scene->detachObject("cube").has_value());
  EXPECT_FALSE(before_detach.isGeometryCurrent());
}

TEST_F(RoboPlanSceneContextTest, RandomCollisionFreePositionsForJointSubset) {
  // Only the requested joints are sampled; every other entry comes from the reference.
  const std::vector<std::string> joint_names = {"wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  const Eigen::VectorXd q_reference{{0.5, -1.0, 1.0, 0.0, 0.0, 0.0}};
  ASSERT_FALSE(scene->hasCollisions(q_reference));
  const auto [lower, upper] = scene->getPositionLimitVectors().value();

  SceneContext context(*scene);
  context.setRngSeed(42);
  for (int i = 0; i < kNumSamples; ++i) {
    const auto maybe_scene_q = scene->randomCollisionFreePositions(joint_names, q_reference);
    const auto maybe_context_q = context.randomCollisionFreePositions(joint_names, q_reference);
    for (const auto& maybe_q : {maybe_scene_q, maybe_context_q}) {
      ASSERT_TRUE(maybe_q.has_value());
      const auto& q = maybe_q.value();
      EXPECT_TRUE(q.head(3).isApprox(q_reference.head(3), 0.0));
      EXPECT_FALSE(q.tail(3).isApprox(q_reference.tail(3)));
      EXPECT_TRUE((q.array() >= lower.array()).all() && (q.array() <= upper.array()).all());
      EXPECT_FALSE(scene->hasCollisions(q));
    }
  }

  // The reference must be a full configuration.
  EXPECT_THROW(scene->randomCollisionFreePositions(joint_names, q_reference.head(3)),
               std::invalid_argument);
  EXPECT_THROW(context.randomCollisionFreePositions(joint_names, q_reference.head(3)),
               std::invalid_argument);

  // Wrap the whole robot in a box so no sample can be collision free.
  ASSERT_TRUE(scene
                  ->addBoxGeometry("wall", "universe", Box(4.0, 4.0, 4.0),
                                   Eigen::Matrix4d::Identity(), Eigen::Vector4d(0.5, 0.5, 0.5, 0.5))
                  .has_value());
  EXPECT_FALSE(scene->randomCollisionFreePositions(joint_names, q_reference, 10).has_value());
  SceneContext enclosed_context(*scene);
  EXPECT_FALSE(
      enclosed_context.randomCollisionFreePositions(joint_names, q_reference, 10).has_value());
}

TEST_F(RoboPlanSceneContextTest, DistanceJacobianMatchesFiniteDifferences) {
  // For separated pairs the row must be the derivative of the signed distance.
  const SceneContext context(*scene);
  const Eigen::VectorXd q{{0.5, -1.0, 1.0, 0.3, -0.4, 0.2}};
  ASSERT_FALSE(context.hasCollisions(q));

  const auto num_pairs = context.getCollisionModel().collisionPairs.size();
  int num_checked = 0;
  for (std::size_t k = 0; k < num_pairs; ++k) {
    // Skip near-contact pairs (< 1 cm), where the closest feature may switch within the FD step.
    if (pairDistance(context, q, k) < 0.01) {
      continue;
    }
    const Eigen::RowVectorXd analytic = distanceJacobian(context, q, k);
    const Eigen::RowVectorXd numeric = finiteDifferenceJacobian(context, q, k);
    // 1e-4 relative: the FD error is ~1e-9 here, so this only absorbs the narrow-phase solver's
    // distance tolerance. 1e-6 absolute covers pairs whose gradient is (near) zero.
    EXPECT_TRUE(analytic.isApprox(numeric, 1.0e-4) || (analytic - numeric).norm() < 1.0e-6)
        << "pair " << k << "\n analytic: " << analytic << "\n numeric:  " << numeric;
    ++num_checked;
  }
  EXPECT_GE(num_checked, 3);
}

TEST_F(RoboPlanSceneContextTest, DistanceJacobianIncreasesDistanceUnderPenetration) {
  // A box on the tool frame and a sphere obstacle penetrating it: the row must still point toward
  // increasing signed distance, where the witness-point difference would have flipped sign.
  const Eigen::VectorXd q{{0.5, -1.0, 1.0, 0.3, -0.4, 0.2}};
  const Eigen::Vector4d color(1.0, 0.0, 0.0, 1.0);
  ASSERT_TRUE(scene
                  ->addBoxGeometry("tool_box", "tool0", Box(0.05, 0.05, 0.05),
                                   Eigen::Matrix4d::Identity(), color)
                  .has_value());
  // Sphere center 0.06 m along the tool x axis: 0.035 m from the box face, radius 0.05 m.
  Eigen::Matrix4d world_T_sphere = Eigen::Matrix4d::Identity();
  world_T_sphere.topRightCorner<3, 1>() =
      (scene->forwardKinematics(q, "tool0") * Eigen::Vector4d(0.06, 0.0, 0.0, 1.0)).head<3>();
  ASSERT_TRUE(scene->addSphereGeometry("obstacle", "universe", Sphere(0.05), world_T_sphere, color)
                  .has_value());

  const SceneContext context(*scene);
  const auto& collision_model = context.getCollisionModel();
  const auto box_id = collision_model.getGeometryId("tool_box");
  const auto sphere_id = collision_model.getGeometryId("obstacle");
  std::size_t pair_index = collision_model.collisionPairs.size();
  for (std::size_t k = 0; k < collision_model.collisionPairs.size(); ++k) {
    const auto& pair = collision_model.collisionPairs[k];
    if ((pair.first == box_id && pair.second == sphere_id) ||
        (pair.first == sphere_id && pair.second == box_id)) {
      pair_index = k;
    }
  }
  ASSERT_LT(pair_index, collision_model.collisionPairs.size());

  // 1e-6 m: EPA's default convergence tolerance on the penetration depth.
  const double distance = pairDistance(context, q, pair_index);
  ASSERT_NEAR(distance, -0.015, 1.0e-6);

  const Eigen::RowVectorXd gradient = distanceJacobian(context, q, pair_index);
  ASSERT_GT(gradient.norm(), 0.0);
  // 1e-3 relative: EPA's depth is only accurate to its tolerance (1e-6 m), which the 2e-6 FD
  // denominator amplifies; the sign check below is the property this test is about.
  EXPECT_TRUE(gradient.isApprox(finiteDifferenceJacobian(context, q, pair_index), 1.0e-3));

  // A small step along the gradient reduces the penetration; a step against it deepens it.
  // 1e-3 rad changes the distance by ~1e-3 * |gradient|, far above EPA's tolerance.
  const Eigen::VectorXd step = 1.0e-3 * gradient.transpose() / gradient.norm();
  EXPECT_GT(pairDistance(context, scene->integrate(q, step), pair_index), distance);
  EXPECT_LT(pairDistance(context, scene->integrate(q, -step), pair_index), distance);
}

TEST_F(RoboPlanSceneContextTest, DistanceJacobianIsZeroForCulledPairs) {
  // Pairs culled by the broadphase margin carry no witness points, so their row is zero.
  const SceneContext context(*scene);
  const Eigen::VectorXd q{{0.5, -1.0, 1.0, 0.3, -0.4, 0.2}};
  context.computeDistances(q, /*broadphase_margin=*/1.0e-3);
  context.computeJointJacobians(q);

  const auto& results = context.getCollisionData().distanceResults;
  Eigen::RowVectorXd jacobian(context.getModel().nv);
  int num_culled = 0;
  for (std::size_t k = 0; k < results.size(); ++k) {
    // Exact zero: culling writes literal zeros to the witness points.
    if (results[k].nearest_points[0].isZero(0.0) && results[k].nearest_points[1].isZero(0.0)) {
      context.computeDistanceJacobian(k, jacobian);
      EXPECT_TRUE(jacobian.isZero(0.0)) << "pair " << k;
      ++num_culled;
    }
  }
  EXPECT_GT(num_culled, 0);
}

TEST_F(RoboPlanSceneContextTest, DistanceJacobianRejectsInvalidArguments) {
  const SceneContext context(*scene);
  const auto q = scene->randomPositions();
  context.computeDistances(q);
  context.computeJointJacobians(q);

  Eigen::RowVectorXd jacobian(context.getModel().nv);
  EXPECT_THROW(
      context.computeDistanceJacobian(context.getCollisionModel().collisionPairs.size(), jacobian),
      std::invalid_argument);
  Eigen::RowVectorXd wrong_size(context.getModel().nv + 1);
  EXPECT_THROW(context.computeDistanceJacobian(0, wrong_size), std::invalid_argument);
}

}  // namespace roboplan
