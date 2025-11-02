"""
Unit tests for scenes in RoboPlan.
"""

from pathlib import Path

import pytest
import numpy as np
import pinocchio as pin

from roboplan import (
    get_install_prefix,
    hasCollisionsAlongPath,
    Box,
    JointType,
    Scene,
    Sphere,
)


URDF = """
<robot name="robot">
  <link name="base_link"/>
  <link name="link1" />
  <link name="link2" />
  <link name="link3" />
  <joint name="continuous_joint" type="continuous">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="revolute_joint" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <joint name="mimic_joint" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    <mimic joint="revolute_joint" multiplier="1.0" offset="0.0"/>
  </joint>
</robot>
"""

SRDF = """
<robot name="robot">
  <group name="arm">
    <joint name="revolute_joint"/>
    <joint name="mimic_joint"/>
  </group>
  <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
  <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
  <disable_collisions link1="link2" link2="link3" reason="Adjacent"/>
</robot>
"""


@pytest.fixture
def test_scene() -> Scene:
    roboplan_examples_dir = Path(get_install_prefix()) / "share"
    roboplan_models_dir = roboplan_examples_dir / "roboplan_example_models" / "models"
    urdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [roboplan_examples_dir]
    yaml_config_path = roboplan_models_dir / "ur_robot_model" / "ur5_config.yaml"

    return Scene("test_scene", urdf_path, srdf_path, package_paths, yaml_config_path)


def test_scene_properties(test_scene: Scene) -> None:
    assert test_scene.getName() == "test_scene"
    assert test_scene.getJointNames() == [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    joint_info = test_scene.getJointInfo("shoulder_pan_joint")
    assert joint_info.type == JointType.REVOLUTE
    assert joint_info.num_position_dofs == 1
    assert joint_info.num_velocity_dofs == 1
    assert np.allclose(joint_info.limits.min_position, np.array([-np.pi]))
    assert np.allclose(joint_info.limits.max_position, np.array([np.pi]))
    assert np.allclose(joint_info.limits.max_velocity, np.array([3.15]))
    assert np.allclose(joint_info.limits.max_acceleration, np.array([2.0]))
    assert np.allclose(joint_info.limits.max_jerk, np.array([10.0]))

    print(test_scene)  # Test printing for good measure


def test_random_positions(test_scene: Scene) -> None:
    # Test subsequent pseudorandom values.
    orig_random_positions = test_scene.randomPositions()
    new_random_positions = test_scene.randomPositions()
    assert np.all(np.not_equal(orig_random_positions, new_random_positions))

    # Test seeded values.
    test_scene.setRngSeed(1234)
    orig_seeded_positions = test_scene.randomPositions()
    test_scene.setRngSeed(1234)  # reset seed
    new_seeded_positions = test_scene.randomPositions()
    assert np.all(np.equal(orig_seeded_positions, new_seeded_positions))


def test_collision_check(test_scene: Scene) -> None:
    # Collision free
    q_free = np.array([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])
    assert not test_scene.hasCollisions(q_free)

    # In collision
    q_coll = np.array([0.0, -1.57, 3.0, 0.0, 0.0, 0.0])
    assert test_scene.hasCollisions(q_coll)


def test_collision_check_along_path(test_scene: Scene) -> None:
    # Collision free
    q_start_free = np.array([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])
    q_end_free = np.array([1.0, -1.57, 1.57, 0.0, 0.0, 0.0])
    assert not hasCollisionsAlongPath(test_scene, q_start_free, q_end_free, 0.05)

    # In collision
    q_end_coll = np.array([0.0, -1.57, 3.0, 0.0, 0.0, 0.0])
    assert hasCollisionsAlongPath(test_scene, q_start_free, q_end_coll, 0.05)


def test_create_frame_map(test_scene: Scene) -> None:
    roboplan_examples_dir = Path(get_install_prefix()) / "share"
    roboplan_models_dir = roboplan_examples_dir / "roboplan_example_models" / "models"
    urdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    package_paths = [roboplan_examples_dir]
    model, _, _ = pin.buildModelsFromUrdf(urdf_path, package_dirs=package_paths)
    for frame in model.frames:
        if frame.name == "universe":
            continue
        assert test_scene.getFrameId(frame.name) == model.getFrameId(frame.name)


def test_collision_geometry(test_scene: Scene) -> None:
    # Nominally, this configuration is collision free
    q = np.zeros(6)
    assert not test_scene.hasCollisions(q)

    # Add some collision objects
    color = np.array([0.5, 0.5, 0.5, 0.5])
    box_tform = np.eye(4)
    box_tform[2, 3] = 1.0  # z position
    test_scene.addBoxGeometry(
        "test_box", "universe", Box(1.0, 1.0, 0.5), box_tform, color
    )

    sphere_tform = np.eye(4)
    sphere_tform[0, 3] = 3.0  # x position
    test_scene.addSphereGeometry(
        "test_sphere", "universe", Sphere(0.5), sphere_tform, color
    )

    assert not test_scene.hasCollisions(q)  # should still be collision free

    # Now move one of the collision objects to be in collision.
    sphere_tform[0, 3] = 0.0
    test_scene.updateGeometryPlacement("test_sphere", "universe", sphere_tform)
    assert test_scene.hasCollisions(q)

    # Remove the collision object and verify that the robot is no longer in collision.
    test_scene.removeGeometry("test_sphere")
    assert not test_scene.hasCollisions(q)


def test_set_collisions(test_scene: Scene) -> None:
    # Nominally, this configuration is collision free
    q = np.zeros(6)
    assert not test_scene.hasCollisions(q)

    # Add a collision object such that the configuration is in collision.
    sphere_tform = np.eye(4)
    sphere_tform[0, 3] = 0.6
    test_scene.addSphereGeometry(
        "test_sphere",
        "universe",
        Sphere(0.1),
        sphere_tform,
        np.array([0.5, 0.5, 0.5, 0.5]),
    )
    assert test_scene.hasCollisions(q)

    # Use the frame names, which should automatically look up the corresponding collision geometries.
    test_scene.setCollisions("forearm_link", "test_sphere", False)
    assert not test_scene.hasCollisions(q)

    # Now re-add the collision pair, which should re-enable collision.
    test_scene.setCollisions("test_sphere", "forearm_link", True)
    assert test_scene.hasCollisions(q)

    # Add an invalid collision pair for check for errors.
    with pytest.raises(RuntimeError) as exc_info:
        test_scene.setCollisions("nonexistent_link", "test_sphere", True)
    expected_error = (
        "Could not set collisions: Could not get collision geometry IDs: "
        "Frame name 'nonexistent_link' not found in frame_map_."
    )
    assert str(exc_info.value) == expected_error


def test_mimics() -> None:
    # Equivalent to the C++ test, but the updated joint state is returned as a new
    # object rather than updated in place.
    test_scene = Scene("test_scene", urdf=URDF, srdf=SRDF)
    q = np.zeros(4)
    q[2] = 1.0
    s = test_scene.applyMimics(q)
    assert s[3] == 1.0
