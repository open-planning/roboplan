#!/usr/bin/env python3

import sys
import time

import pinocchio as pin
import tyro
import xacro
from common import get_model_data
from pinocchio.visualize import ViserVisualizer

from roboplan.core import (
    JointConfiguration,
    PathShortcutter,
    PathShortcuttingOptions,
    Scene,
    computePathLength,
    loadJointLimitsConfig,
    loadUrdfSceneDescriptionFromXml,
)
from roboplan.example_models import get_package_share_dir
from roboplan.ompl import OMPLPlanner, OMPLPlannerOptions, visualizeGraph
from roboplan.rrt import RRT, RRTOptions
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions
from roboplan.visualization import visualizeJointTrajectory

PATH_COLORS = [(200, 0, 0), (0, 0, 200), (0, 150, 0), (200, 120, 0), (150, 0, 150)]


def main(
    model: str = "ur5",
    planners: tuple[str, ...] = ("RRTConnect", "AORRTC"),
    compare_rrt: bool = True,
    max_planning_time: float = 2.0,
    collision_check_step_size: float = 0.05,
    simplify_path: bool = True,
    toppra_mode: SplineFittingMode = SplineFittingMode.Adaptive,
    include_obstacles: bool = True,
    visualize_graph: bool = True,
    rng_seed: int | None = None,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Plan with OMPL planners between random collision-free configurations and print timing.

    Every planner solves the same start/goal problem against the same scene. Optimizing planners
    such as AORRTC keep improving the path until the time budget runs out, so their planning time
    is the budget, while RRT-Connect returns on the first solution.

    Parameters:
        model: The name of the model to use.
        planners: The OMPL planners to run. See `OMPLPlanner.plannerNames()` for the options.
        compare_rrt: Whether to also run RoboPlan's own RRT-Connect planner (with path
            shortcutting when `simplify_path` is set) on the same problem.
        max_planning_time: The planning time budget (in seconds) for each planner.
        collision_check_step_size: Configuration-space step size for collision checking along edges.
        simplify_path: Whether to simplify (OMPL) or shortcut (RoboPlan RRT) the found paths.
        toppra_mode: The spline fitting mode for TOPP-RA trajectory generation.
        include_obstacles: Whether or not to include additional obstacles in the scene.
        visualize_graph: Whether to draw each OMPL planner's search graph (toggle them in the
            Viser scene tree). AORRTC clears its trees between iterations, so it draws nothing.
        rng_seed: The seed for the scene's and planners' random number generators.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
    """
    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        sys.exit(1)
    group_name = model_data.default_joint_group
    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()
    scene = Scene(
        "ompl_scene", loadUrdfSceneDescriptionFromXml(urdf_xml, package_paths)
    )
    scene.importJointLimitsFromConfig(
        loadJointLimitsConfig(model_data.yaml_config_path)
    )
    scene.importSrdf(srdf_xml)
    q_indices = scene.getJointGroupInfo(group_name).q_indices

    # Separate Pinocchio models for visualization, as in the other examples.
    pin_model = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        pin_model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        pin_model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )
    if include_obstacles:
        for obstacle in model_data.obstacles:
            obstacle.addToScene(scene)
            obstacle.addToPinocchioModels(pin_model, collision_model, visual_model)

    viz = ViserVisualizer(pin_model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    ompl_planners = {
        name: OMPLPlanner(
            scene,
            OMPLPlannerOptions(
                group_name=group_name,
                planner_name=name,
                max_planning_time=max_planning_time,
                collision_check_step_size=collision_check_step_size,
                simplify_path=simplify_path,
            ),
        )
        for name in planners
    }
    rrt = RRT(
        scene,
        RRTOptions(
            group_name=group_name,
            max_nodes=1_000_000,  # Let the time budget be the limit, as for the OMPL planners.
            collision_check_step_size=collision_check_step_size,
            max_planning_time=max_planning_time,
            rrt_connect=True,
        ),
    )
    shortcutter = PathShortcutter(
        scene,
        PathShortcuttingOptions(
            group_name=group_name, max_step_size=collision_check_step_size
        ),
    )
    toppra = PathParameterizerTOPPRA(scene, group_name)
    traj_dt = 0.01

    if rng_seed is not None:
        scene.setRngSeed(rng_seed)
        rrt.setRngSeed(rng_seed)
        OMPLPlanner.setRngSeed(rng_seed)

    q_full = scene.randomCollisionFreePositions()
    scene.setJointPositions(q_full)
    viz.display(q_full)

    trajectories = []
    animate = False

    def plan_all():
        nonlocal q_full
        trajectories.clear()
        start = JointConfiguration()
        start.positions = q_full[q_indices]
        goal = JointConfiguration()
        goal.positions = scene.randomCollisionFreePositions()[q_indices]

        header = (
            f"{'Planner':<16}{'plan [s]':>10}{'simplify [s]':>14}{'state checks':>14}"
            f"{'motion checks':>15}{'waypoints':>11}{'length [rad]':>14}{'TOPP-RA [s]':>13}"
            f"{'traj [s]':>10}"
        )
        print("\n" + header)
        print("-" * len(header))

        def report(name, path, plan_time, simplify_time, state_checks, motion_checks):
            t_start = time.perf_counter()
            traj = toppra.generate(path, TOPPRAOptions(dt=traj_dt, mode=toppra_mode))
            toppra_time = time.perf_counter() - t_start
            length = computePathLength(scene, group_name, path)
            print(
                f"{name:<16}{plan_time:>10.3f}{simplify_time:>14.3f}{state_checks:>14}"
                f"{motion_checks:>15}{len(path.positions):>11}{length:>14.3f}"
                f"{toppra_time:>13.3f}{traj.times[-1]:>10.2f}"
            )
            trajectories.append((name, traj))
            visualizeJointTrajectory(
                viz,
                scene,
                traj,
                model_data.ee_names,
                PATH_COLORS[len(trajectories) % len(PATH_COLORS)],
                f"/paths/{name}",
            )

        for name, planner in ompl_planners.items():
            try:
                path = planner.plan(start, goal)
            except RuntimeError as ex:
                print(f"{name:<16}{ex}")
                continue
            stats = planner.getLastStats()
            report(
                name,
                path,
                stats.planning_time,
                stats.simplification_time,
                stats.num_state_checks,
                stats.num_motion_checks,
            )
            if visualize_graph:
                visualizeGraph(
                    viz,
                    scene,
                    planner,
                    group_name,
                    model_data.ee_names,
                    0.1,
                    start_tree_name=f"/graphs/{name}/start_tree",
                    goal_tree_name=f"/graphs/{name}/goal_tree",
                )

        if compare_rrt:
            try:
                t_start = time.perf_counter()
                path = rrt.plan(start, goal)
                plan_time = time.perf_counter() - t_start
                simplify_time = 0.0
                if simplify_path:
                    t_start = time.perf_counter()
                    path = shortcutter.shortcut(path)
                    simplify_time = time.perf_counter() - t_start
                report("roboplan RRTC", path, plan_time, simplify_time, "-", "-")
            except RuntimeError as ex:
                print(f"{'roboplan RRTC':<16}{ex}")

        q_full = scene.toFullJointPositions(group_name, goal.positions)
        viz.display(scene.toFullJointPositions(group_name, start.positions))

    plan_button = viz.viewer.gui.add_button("Plan path")
    animate_button = viz.viewer.gui.add_button("Animate trajectories")

    @plan_button.on_click
    def _(_):
        plan_button.disabled = animate_button.disabled = True
        try:
            plan_all()
        finally:
            plan_button.disabled = animate_button.disabled = False

    @animate_button.on_click
    def _(_):
        nonlocal animate
        animate = True

    plan_all()
    while True:
        if animate and trajectories:
            plan_button.disabled = animate_button.disabled = True
            for name, traj in trajectories:
                print(f"Animating {name} trajectory...")
                for q in traj.positions:
                    viz.display(scene.toFullJointPositions(group_name, q))
                    time.sleep(traj_dt)
            animate = False
            plan_button.disabled = animate_button.disabled = False
        time.sleep(0.1)


if __name__ == "__main__":
    tyro.cli(main)
