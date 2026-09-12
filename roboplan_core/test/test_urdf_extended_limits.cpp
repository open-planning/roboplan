#include <fstream>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <limits>
#include <memory>

#include <roboplan/core/scene.hpp>

namespace {
constexpr double kTolerance = 1e-6;
constexpr double kUnlimited = std::numeric_limits<double>::max();
}  // namespace

const std::string kSrdf = R"(
<robot name="robot">
  <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
</robot>
)";

// All three extended limit attributes explicitly set.
const std::string kUrdfAllLimits = R"(
<robot name="robot">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"
           acceleration="5.0" jerk="50.0"/>
  </joint>
</robot>
)";

// Only acceleration set; jerk should stay unlimited.
const std::string kUrdfAccelOnly = R"(
<robot name="robot">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"
           acceleration="5.0"/>
  </joint>
</robot>
)";

// No extended attributes — both should stay unlimited.
const std::string kUrdfNoExtendedLimits = R"(
<robot name="robot">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
</robot>
)";

// For YAML override test: URDF has 5.0/50.0, YAML overrides to 10.0/100.0.
const std::string kUrdfForYamlOverride = R"(
<robot name="robot">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"
           acceleration="5.0" jerk="50.0"/>
  </joint>
</robot>
)";

// Mimic joint test.
const std::string kUrdfWithMimic = R"(
<robot name="robot">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"
           acceleration="6.0" jerk="60.0"/>
  </joint>
  <joint name="mimic_joint" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    <mimic joint="joint1" multiplier="2.0" offset="0.0"/>
  </joint>
</robot>
)";

const std::string kSrdfWithMimic = R"(
<robot name="robot">
  <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
  <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
</robot>
)";

namespace roboplan {

// ──────────────────────────────────────────────────────────────
// All extended limits explicitly set
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, AllLimitsSet) {
  Scene scene("test", kUrdfAllLimits, kSrdf);

  const auto info = scene.getJointInfo("joint1").value();
  EXPECT_NEAR(info.limits.max_acceleration[0], 5.0, kTolerance);
  EXPECT_NEAR(info.limits.max_jerk[0], 50.0, kTolerance);
}

// ──────────────────────────────────────────────────────────────
// Only acceleration set; jerk should stay unlimited
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, AccelerationOnlyJerkUnlimited) {
  Scene scene("test", kUrdfAccelOnly, kSrdf);

  const auto info = scene.getJointInfo("joint1").value();
  EXPECT_NEAR(info.limits.max_acceleration[0], 5.0, kTolerance);
  EXPECT_DOUBLE_EQ(info.limits.max_jerk[0], kUnlimited);
}

// ──────────────────────────────────────────────────────────────
// No extended attributes — both should stay unlimited
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, NoExtendedLimitsStayUnlimited) {
  Scene scene("test", kUrdfNoExtendedLimits, kSrdf);

  const auto info = scene.getJointInfo("joint1").value();
  EXPECT_DOUBLE_EQ(info.limits.max_acceleration[0], kUnlimited);
  EXPECT_DOUBLE_EQ(info.limits.max_jerk[0], kUnlimited);
}

// ──────────────────────────────────────────────────────────────
// YAML overrides URDF values
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, YamlOverridesUrdf) {
  const auto tmp_yaml = std::filesystem::temp_directory_path() / "test_urdf_override.yaml";
  {
    std::ofstream f(tmp_yaml);
    f << "joint_limits:\n"
      << "  joint1:\n"
      << "    max_acceleration: [10.0]\n"
      << "    max_jerk: [100.0]\n";
  }

  Scene scene("test", kUrdfForYamlOverride, kSrdf, {}, tmp_yaml);

  const auto info = scene.getJointInfo("joint1").value();
  EXPECT_NEAR(info.limits.max_acceleration[0], 10.0, kTolerance);
  EXPECT_NEAR(info.limits.max_jerk[0], 100.0, kTolerance);

  std::filesystem::remove(tmp_yaml);
}

// ──────────────────────────────────────────────────────────────
// Mimic joint inherits scaled limits
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, MimicJointInheritsScaledLimits) {
  Scene scene("test", kUrdfWithMimic, kSrdfWithMimic);

  const auto joint1_info = scene.getJointInfo("joint1").value();
  EXPECT_NEAR(joint1_info.limits.max_acceleration[0], 6.0, kTolerance);
  EXPECT_NEAR(joint1_info.limits.max_jerk[0], 60.0, kTolerance);

  // mimic multiplier=2.0, so limits scale by |2.0|.
  const auto mimic_info = scene.getJointInfo("mimic_joint").value();
  EXPECT_NEAR(mimic_info.limits.max_acceleration[0], 12.0, kTolerance);
  EXPECT_NEAR(mimic_info.limits.max_jerk[0], 120.0, kTolerance);
}

}  // namespace roboplan
