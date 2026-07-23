#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_example_models/resources.hpp>

// Internal detail header (not installed); test/CMakeLists.txt adds ../src to the include path.
#include "reduced_group_model.hpp"

namespace roboplan {
namespace {

std::shared_ptr<Scene> makeScene(const std::string& robot_dir, const std::string& urdf,
                                 const std::string& srdf) {
  const auto model_prefix = example_models::get_package_models_dir();
  const std::vector<std::filesystem::path> package_paths = {
      example_models::get_package_share_dir()};
  return std::make_shared<Scene>("test_scene", model_prefix / robot_dir / urdf,
                                 model_prefix / robot_dir / srdf, package_paths);
}

// SO-101 with the 5-DoF "arm" group: the model also has a 1-DoF "gripper" joint, so the arm
// group is a strict subset and this exercises real reduction (gripper locked). UR5's "arm"
// chain spans the whole movable model, exercising the no-op branch.
std::shared_ptr<Scene> makeSo101Scene() {
  return makeScene("so101_robot_model", "so101.urdf", "so101.srdf");
}
std::shared_ptr<Scene> makeUr5Scene() {
  return makeScene("ur_robot_model", "ur5_gripper.urdf", "ur5_gripper.srdf");
}

// The SO-101 arm group: 5 revolute (non-continuous) joints, and its chain-tip link frame.
constexpr int kSo101ArmNv = 5;
const char* const kSo101ArmJoints[] = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
                                       "wrist_roll"};
constexpr const char* kSo101TipFrame = "gripper_link";
constexpr const char* kSo101LockedJoint = "gripper";

bool contains(const std::vector<std::string>& v, const std::string& s) {
  return std::find(v.begin(), v.end(), s) != v.end();
}

}  // namespace

// --- Real reduction: SO-101 arm, gripper locked -----------------------------------------------

TEST(ReducedGroupModelTest, ReducedDimensionsMatchGroup) {
  auto scene = makeSo101Scene();
  const ReducedGroupModel rgm(*scene, "arm");

  // The SO-101 arm is 5 revolute (non-continuous) joints: nq == nv == 5.
  EXPECT_EQ(rgm.nv(), kSo101ArmNv);
  EXPECT_EQ(rgm.nq(), kSo101ArmNv);

  // nv() must equal the group's velocity-index count read from core (round-trip, not re-derived).
  const auto group = scene->getJointGroupInfo("arm");
  ASSERT_TRUE(group.has_value());
  EXPECT_EQ(rgm.vIndices().size(), group->v_indices.size());
  EXPECT_EQ(rgm.vIndices(), group->v_indices);

  // Reduction actually happened: the reduced model is strictly smaller than the full one.
  EXPECT_LT(rgm.reducedModel().nv, rgm.fullModel().nv);
}

TEST(ReducedGroupModelTest, LockedJointsAreTheNonGroupComplement) {
  auto scene = makeSo101Scene();
  const ReducedGroupModel rgm(*scene, "arm");

  const auto& locked = rgm.lockedJointNames();
  // The gripper joint is locked; the arm joints are not.
  EXPECT_FALSE(locked.empty());
  EXPECT_TRUE(contains(locked, kSo101LockedJoint));
  for (const char* arm_joint : kSo101ArmJoints) {
    EXPECT_FALSE(contains(locked, arm_joint));
  }
  // A locked joint is frozen: it is no longer a movable joint of the reduced model (it becomes a
  // FIXED_JOINT frame). Every locked name must be absent from the reduced model's joints.
  for (const auto& name : locked) {
    EXPECT_FALSE(rgm.reducedModel().existJointName(name)) << name << " should be locked/frozen";
    EXPECT_TRUE(rgm.fullModel().existJointName(name)) << name << " should exist in the full model";
  }

  // Structural invariant (holds for reduction and no-op alike): reduced movable joints + locked
  // joints == full movable joints. njoints counts the universe root, hence the -1 on each side.
  const int reduced_movable = rgm.reducedModel().njoints - 1;
  const int full_movable = rgm.fullModel().njoints - 1;
  EXPECT_EQ(reduced_movable + static_cast<int>(locked.size()), full_movable);
}

TEST(ReducedGroupModelTest, InitialStateMatchesSceneConfiguration) {
  auto scene = makeSo101Scene();
  const ReducedGroupModel rgm(*scene, "arm");

  ASSERT_EQ(rgm.q0().size(), rgm.nq());
  ASSERT_EQ(rgm.v0().size(), rgm.nv());

  // v0 defaults to zero (design §3.1).
  EXPECT_TRUE(rgm.v0().isZero());

  // q0 must reproduce the scene's current arm configuration. Copy the arm's q-values out of the
  // full reference configuration via the group's q_indices and compare.
  const Eigen::VectorXd& full_q = scene->getCurrentJointPositions();
  const auto group = scene->getJointGroupInfo("arm");
  ASSERT_TRUE(group.has_value());
  ASSERT_EQ(group->q_indices.size(), rgm.q0().size());
  for (int i = 0; i < group->q_indices.size(); ++i) {
    // Exact copy, no arithmetic: an equality check is appropriate (no tolerance needed).
    EXPECT_DOUBLE_EQ(rgm.q0()(i), full_q(group->q_indices(i)));
  }
}

TEST(ReducedGroupModelTest, FrameLookupResolvesAgainstReducedModel) {
  auto scene = makeSo101Scene();
  const ReducedGroupModel rgm(*scene, "arm");

  // The arm chain-tip link is present in the reduced model.
  const auto tip = rgm.frameId(kSo101TipFrame);
  ASSERT_TRUE(tip.has_value()) << (tip ? "" : tip.error());
  EXPECT_LT(*tip, rgm.reducedModel().frames.size());

  // A missing frame returns an error rather than the frames.size() sentinel getFrameId gives.
  const auto missing = rgm.frameId("no_such_frame");
  EXPECT_FALSE(missing.has_value());

  // frameNames() lists reduced-model frames, including the tip.
  EXPECT_TRUE(contains(rgm.frameNames(), kSo101TipFrame));
}

TEST(ReducedGroupModelTest, Deterministic) {
  auto scene = makeSo101Scene();
  const ReducedGroupModel a(*scene, "arm");
  const ReducedGroupModel b(*scene, "arm");
  EXPECT_EQ(a.nq(), b.nq());
  EXPECT_EQ(a.nv(), b.nv());
  EXPECT_EQ(a.q0(), b.q0());
  EXPECT_EQ(a.lockedJointNames(), b.lockedJointNames());
}

// --- No-op reduction: UR5 arm spans the whole movable model -----------------------------------

TEST(ReducedGroupModelTest, WholeModelGroupIsNoOp) {
  auto scene = makeUr5Scene();
  const ReducedGroupModel rgm(*scene, "arm");

  // The UR5 "arm" chain covers every movable joint, so nothing is locked and the reduced model
  // equals the full one dimensionally.
  EXPECT_TRUE(rgm.lockedJointNames().empty());
  EXPECT_EQ(rgm.reducedModel().nq, rgm.fullModel().nq);
  EXPECT_EQ(rgm.reducedModel().nv, rgm.fullModel().nv);
  EXPECT_EQ(rgm.nv(), rgm.fullModel().nv);
}

// --- Error signalling: setup violations throw (numerics rule) ---------------------------------

TEST(ReducedGroupModelTest, EmptyGroupNameThrows) {
  auto scene = makeSo101Scene();
  EXPECT_THROW(ReducedGroupModel(*scene, ""), std::invalid_argument);
}

TEST(ReducedGroupModelTest, UnknownGroupThrows) {
  auto scene = makeSo101Scene();
  EXPECT_THROW(ReducedGroupModel(*scene, "not_a_group"), std::invalid_argument);
}

// The two remaining constructor guards are defensive and intentionally NOT exercised here:
//   * the floating-base rejection (reduced_group_model.cpp) needs a group containing a
//     free-flyer/planar joint, and
//   * the "no movable joints" guard needs a joint-less group,
// neither of which any loadable example_models fixture produces (all are fixed-base robots with
// non-empty groups; FR3/dual_fr3, the only candidates with richer structure, currently fail to
// load in core with an unrelated IndexError). These branches stay covered by construction —
// documented here rather than silently untested, per .claude/rules/testing.md.

}  // namespace roboplan
