from pathlib import Path

from roboplan import (
    get_package_share_dir,
    JointConfiguration,
    Scene,
    RRTOptions,
    RRT,
)


if __name__ == "__main__":

    roboplan_examples_dir = Path(get_package_share_dir())
    urdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [roboplan_examples_dir]

    scene = Scene("test_scene", urdf_path, srdf_path, package_paths)

    options = RRTOptions()
    options.max_connection_distance = 1.0
    options.collision_check_step_size = 0.05
    options.max_planning_time = 3.0
    options.rrt_connect = True
    rrt = RRT(scene, options)

    start = JointConfiguration()
    start.positions = scene.randomCollisionFreePositions()
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = scene.randomCollisionFreePositions()
    assert goal.positions is not None

    path = rrt.plan(start, goal)
    assert path is not None
    print(path)
