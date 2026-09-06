"""
Unit tests for RoboPlan filter bindings.
"""

import numpy as np

from roboplan.filters import SE3LowPassFilter


def test_se3_low_pass_filter_first_update() -> None:
    target_pose = np.eye(4)
    target_pose[0, 3] = 1.0

    filt = SE3LowPassFilter()
    filtered_pose = filt.update(target_pose, 0.01)

    assert filt.isInitialized()
    np.testing.assert_allclose(filtered_pose, target_pose)


def test_se3_low_pass_filter_smooths_position() -> None:
    initial_pose = np.eye(4)
    target_pose = np.eye(4)
    target_pose[0, 3] = 1.0

    filt = SE3LowPassFilter(0.1)
    filt.reset(initial_pose)
    filtered_pose = filt.update(target_pose, 0.1)

    assert 0.0 < filtered_pose[0, 3] < 1.0
    np.testing.assert_allclose(filtered_pose[1:, 3], target_pose[1:, 3])
