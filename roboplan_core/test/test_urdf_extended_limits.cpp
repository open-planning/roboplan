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

// All three extended limit attributes explicitly set.
const std::string kUrdfAllLimits = R"(
<robot name="robot" version="1.2">
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
<robot name="robot" version="1.2">
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

// No extended attributes — both should stay unlimited. Deliberately left at URDF 1.0 so this case
// parses on every urdfdom, including the older ones that reject a 1.2 version outright.
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
<robot name="robot" version="1.2">
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
<robot name="robot" version="1.2">
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

namespace roboplan {

// Only urdfdom 3.0.0 and newer accept a URDF 1.2 document and parse its `acceleration` / `jerk`
// attributes. The older releases shipped by ROS 2 Jazzy and Kilted reject any version above 1.0
// outright, so the 1.2 fixtures below cannot even be parsed there. Probe the capability once
// instead of assuming it.
bool urdfExtendedLimitsSupported() {
  static const bool supported = [] {
    try {
      const Scene probe("probe", loadUrdfSceneDescriptionFromXml(kUrdfAccelOnly));
      return probe.getJointInfo("joint1").value().limits.max_acceleration[0] != kUnlimited;
    } catch (const std::exception&) {
      return false;
    }
  }();
  return supported;
}

#define SKIP_IF_NO_URDF_EXTENDED_LIMITS()                                                          \
  if (!urdfExtendedLimitsSupported()) {                                                            \
    GTEST_SKIP() << "urdfdom does not support URDF 1.2 extended joint limits.";                    \
  }

// ──────────────────────────────────────────────────────────────
// All extended limits explicitly set
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, AllLimitsSet) {
  SKIP_IF_NO_URDF_EXTENDED_LIMITS();
  Scene scene("test", loadUrdfSceneDescriptionFromXml(kUrdfAllLimits));

  const auto info = scene.getJointInfo("joint1").value();
  EXPECT_NEAR(info.limits.max_acceleration[0], 5.0, kTolerance);
  EXPECT_NEAR(info.limits.max_jerk[0], 50.0, kTolerance);
}

// ──────────────────────────────────────────────────────────────
// Only acceleration set; jerk should stay unlimited
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, AccelerationOnlyJerkUnlimited) {
  SKIP_IF_NO_URDF_EXTENDED_LIMITS();
  Scene scene("test", loadUrdfSceneDescriptionFromXml(kUrdfAccelOnly));

  const auto info = scene.getJointInfo("joint1").value();
  EXPECT_NEAR(info.limits.max_acceleration[0], 5.0, kTolerance);
  EXPECT_DOUBLE_EQ(info.limits.max_jerk[0], kUnlimited);
}

// ──────────────────────────────────────────────────────────────
// No extended attributes — both should stay unlimited
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, NoExtendedLimitsStayUnlimited) {
  Scene scene("test", loadUrdfSceneDescriptionFromXml(kUrdfNoExtendedLimits));

  const auto info = scene.getJointInfo("joint1").value();
  EXPECT_DOUBLE_EQ(info.limits.max_acceleration[0], kUnlimited);
  EXPECT_DOUBLE_EQ(info.limits.max_jerk[0], kUnlimited);
}

// ──────────────────────────────────────────────────────────────
// YAML overrides URDF values
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, YamlOverridesUrdf) {
  SKIP_IF_NO_URDF_EXTENDED_LIMITS();
  const auto tmp_yaml = std::filesystem::temp_directory_path() / "test_urdf_override.yaml";
  {
    std::ofstream f(tmp_yaml);
    f << "joint_limits:\n"
      << "  joint1:\n"
      << "    max_acceleration: [10.0]\n"
      << "    max_jerk: [100.0]\n";
  }

  Scene scene("test", loadUrdfSceneDescriptionFromXml(kUrdfForYamlOverride));
  scene.importJointLimitsFromConfig(loadJointLimitsConfig(tmp_yaml));

  const auto info = scene.getJointInfo("joint1").value();
  EXPECT_NEAR(info.limits.max_acceleration[0], 10.0, kTolerance);
  EXPECT_NEAR(info.limits.max_jerk[0], 100.0, kTolerance);

  std::filesystem::remove(tmp_yaml);
}

// ──────────────────────────────────────────────────────────────
// Mimic joint inherits scaled limits
// ──────────────────────────────────────────────────────────────
TEST(UrdfExtendedLimits, MimicJointInheritsScaledLimits) {
  SKIP_IF_NO_URDF_EXTENDED_LIMITS();
  Scene scene("test", loadUrdfSceneDescriptionFromXml(kUrdfWithMimic));

  const auto joint1_info = scene.getJointInfo("joint1").value();
  EXPECT_NEAR(joint1_info.limits.max_acceleration[0], 6.0, kTolerance);
  EXPECT_NEAR(joint1_info.limits.max_jerk[0], 60.0, kTolerance);

  // mimic multiplier=2.0, so limits scale by |2.0|.
  const auto mimic_info = scene.getJointInfo("mimic_joint").value();
  EXPECT_NEAR(mimic_info.limits.max_acceleration[0], 12.0, kTolerance);
  EXPECT_NEAR(mimic_info.limits.max_jerk[0], 120.0, kTolerance);
}

}  // namespace roboplan
