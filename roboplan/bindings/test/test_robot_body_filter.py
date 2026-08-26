"""
Unit tests for the robot body filter bindings in RoboPlan.
"""

import numpy as np
import pytest

from roboplan.core import (
    RobotBodyFilter,
    RobotBodyFilterMethod,
    RobotBodyFilterOptions,
    Scene,
)


# A robot made of a single sphere of radius 0.1 fixed at the origin, so the filter's
# classification can be checked against closed-form distances.
URDF = """
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
"""

SRDF = """<robot name="ball_bot"/>"""


@pytest.fixture
def ball_scene():
    return Scene("ball_scene", urdf=URDF, srdf=SRDF)


# The padded body is the ball of radius 0.1 + 0.05 = 0.15 around the origin; the padded
# OBB is the box of half extent 0.15. The corner point at distance ~0.17 tells them apart.
POINTS = np.array(
    [
        [0.14, 0.0, 0.0],
        [0.12, 0.12, 0.0],
        [0.2, 0.0, 0.0],
    ]
)


def test_narrowphase_mask(ball_scene):
    body_filter = RobotBodyFilter(
        ball_scene,
        RobotBodyFilterOptions(padding=0.05, method=RobotBodyFilterMethod.NARROWPHASE),
    )
    mask = body_filter.computeMask(np.empty(0), POINTS)
    assert mask.dtype == bool
    assert np.array_equal(mask, [True, False, False])


def test_padded_obb_mask_over_removes_corners(ball_scene):
    body_filter = RobotBodyFilter(
        ball_scene,
        RobotBodyFilterOptions(padding=0.05, method=RobotBodyFilterMethod.PADDED_OBB),
    )
    mask = body_filter.computeMask(np.empty(0), POINTS)
    assert np.array_equal(mask, [True, True, False])


def test_extra_padding(ball_scene):
    body_filter = RobotBodyFilter(ball_scene, RobotBodyFilterOptions(padding=0.05))
    mask = body_filter.computeMask(np.empty(0), POINTS, np.array([0.0, 0.0, 0.1]))
    assert np.array_equal(mask, [True, False, True])


def test_filter_points(ball_scene):
    body_filter = RobotBodyFilter(ball_scene, RobotBodyFilterOptions(padding=0.05))
    kept = body_filter.filterPoints(np.empty(0), POINTS)
    assert np.array_equal(kept, POINTS[1:])


def test_negative_padding_raises(ball_scene):
    with pytest.raises(ValueError):
        RobotBodyFilter(ball_scene, RobotBodyFilterOptions(padding=-0.01))
