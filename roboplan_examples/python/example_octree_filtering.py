#!/usr/bin/env python3

"""
Demonstrates filtering the robot's own body out of a point cloud before turning it into
an octree collision object.

The example uses the same environment point cloud as the octree RRT demo, but additionally
scatters points on the Franka robot itself at its current pose, the way a depth camera
looking at the workspace would also see the robot. Without filtering, those points become
occupied voxels on the robot body and the current pose is immediately in collision, so no
planning is possible. The RobotBodyFilter removes them, and the example then repeatedly
plans RRT paths to random collision-free goals, refreshing the simulated point cloud and
the filtered octree at every pose the robot stops at.

The viewer shows the world as the point cloud itself: kept points in turquoise and the
points removed by the filter in red. The octree built from the kept points is added to the
scene only as the collision object the RRT plans against.

The filter method (exact NARROWPHASE or the faster, conservative PADDED_OBB) is selected
with --method, and each cycle prints its timing.
"""

import time

import numpy as np
import tyro
import xacro

try:
    import coal
except ModuleNotFoundError:
    import hppfcl as coal

import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

from common import (
    ROBOPLAN_MODELS_DIR,
    get_home_configuration,
    get_model_data,
    load_point_cloud,
    sample_points_on_robot,
)
from roboplan.core import (
    JointConfiguration,
    OcTree,
    RobotBodyFilter,
    RobotBodyFilterMethod,
    RobotBodyFilterOptions,
    Scene,
)
from roboplan.example_models import get_package_share_dir
from roboplan.rrt import RRTOptions, RRT
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions
from roboplan.visualization import visualizeJointTrajectory


OCTREE_NAME = "filtered_octree"


def main(
    method: RobotBodyFilterMethod = RobotBodyFilterMethod.NARROWPHASE,
    padding: float = 0.08,
    num_robot_points: int = 2000,
    robot_point_noise_std: float = 0.005,
    voxel_resolution: float = 0.04,
    max_planning_time: float = 5.0,
    rng_seed: int = 42,
    host: str = "localhost",
    port: str = "8000",
    open_browser: bool = True,
):
    """
    Run the octree filtering example with the provided parameters.

    Parameters:
        method: The filter method used to build the scene octree.
        padding: Distance around the robot's collision geometry, in meters, within which
            points are considered part of the robot body.
        num_robot_points: Number of synthetic sensor points scattered on the robot body.
        robot_point_noise_std: Standard deviation, in meters, of the noise on those points.
        voxel_resolution: The octree voxel resolution, in meters.
        max_planning_time: The maximum time (in seconds) for the RRT to search for a path.
        rng_seed: The seed used for point sampling, goal selection, and RRT.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
        open_browser: Whether to open the Viser page in a browser.
    """
    model_data = get_model_data()["franka"]
    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()
    scene = Scene(
        "octree_filtering_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )
    scene.setRngSeed(rng_seed)
    group_name = model_data.default_joint_group
    group_info = scene.getJointGroupInfo(group_name)

    # Redundant Pinocchio models for visualization and robot point sampling (see example_rrt.py).
    model = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=open_browser, loadModel=True, host=host, port=port)

    # The static environment cloud from the octree RRT demo. The robot's own sensor shadow is
    # re-simulated at every pose the robot stops at.
    env_points = load_point_cloud(
        ROBOPLAN_MODELS_DIR / "pointclouds" / "example_point_cloud.ply"
    )
    print(f"Environment point cloud: {len(env_points)} points")

    # The filter snapshots only the robot's own geometry, so the octree being swapped in and
    # out of the scene below does not invalidate it.
    body_filter = RobotBodyFilter(
        scene, RobotBodyFilterOptions(padding=padding, method=method)
    )

    toppra = PathParameterizerTOPPRA(scene, group_name)
    traj_dt = 0.01
    rrt_options = RRTOptions(
        group_name=group_name,
        max_planning_time=max_planning_time,
        rrt_connect=True,
    )

    q_current = get_home_configuration(scene, model_data)
    scene.setJointPositions(q_current)
    viz.display(q_current)

    rng = np.random.default_rng(rng_seed)
    plan_idx = 0
    while True:
        # Simulate a sensor snapshot at the current pose: the environment plus points on the
        # robot body, then remove the robot's sensor shadow with the selected filter method.
        robot_points = sample_points_on_robot(
            model,
            collision_model,
            q_current,
            num_robot_points,
            rng,
            robot_point_noise_std,
        )
        cloud = np.vstack([env_points, robot_points])

        t_start = time.perf_counter()
        mask = body_filter.computeMask(q_current, cloud)
        elapsed = time.perf_counter() - t_start
        print(
            f"\n[plan {plan_idx}] {method.name} removed {mask.sum()} / {len(cloud)} "
            f"points in {1000.0 * elapsed:.2f} ms"
        )

        # Rebuild the collision octree from the kept points and swap it into the scene. The
        # viewer instead shows the cloud itself: kept points in turquoise, removed in red.
        filtered_octree = coal.makeOctree(cloud[~mask], voxel_resolution)
        if plan_idx > 0:
            scene.removeGeometry(OCTREE_NAME)
        scene.addOcTreeGeometry(
            OCTREE_NAME,
            "universe",
            OcTree(filtered_octree.toBoxes(), voxel_resolution),
            np.eye(4),
            np.array([0.251, 0.878, 0.816, 1.0]),
        )
        viz.viewer.scene.add_point_cloud(
            "/environment_points",
            points=cloud[~mask],
            colors=(64, 224, 208),
            point_size=0.005,
        )
        viz.viewer.scene.add_point_cloud(
            "/removed_points",
            points=cloud[mask],
            colors=(255, 60, 60),
            point_size=0.005,
        )

        # Plan to a random collision-free goal through the fresh octree. The RRT is rebuilt
        # because swapping the octree changed the scene's collision geometry.
        rrt = RRT(scene, rrt_options)
        rrt.setRngSeed(rng_seed + plan_idx)

        start = JointConfiguration()
        start.positions = q_current[group_info.q_indices]
        goal = JointConfiguration()
        goal.positions = scene.randomCollisionFreePositions()[group_info.q_indices]

        t_start = time.time()
        try:
            path = rrt.plan(start, goal)
        except RuntimeError as e:
            print(f"Planning failed ({e}); retrying with a new goal.")
            continue
        print(
            f"Found a path with {len(path.positions)} waypoints "
            f"in {time.time() - t_start:.3f} s"
        )

        # Time-parameterize, visualize, and animate the trajectory.
        traj = toppra.generate(
            path, TOPPRAOptions(dt=traj_dt, mode=SplineFittingMode.Adaptive)
        )
        visualizeJointTrajectory(
            viz, scene, traj, model_data.ee_names, (100, 0, 0), "/rrt/path"
        )
        for q in traj.positions:
            viz.display(scene.toFullJointPositions(group_name, q))
            time.sleep(traj_dt)

        # The goal pose becomes the next sensing and planning pose.
        q_current = scene.toFullJointPositions(group_name, goal.positions)
        scene.setJointPositions(q_current)
        plan_idx += 1
        time.sleep(1.0)


if __name__ == "__main__":
    tyro.cli(main)
