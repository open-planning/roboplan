import pytest
import sys
import xacro

from roboplan import (
    JointConfiguration,
    Scene,
    RRTOptions,
    RRT,
)

# We don't build the bindings examples, so we just include the relative
# directory manually.
from pathlib import Path

examples_dir = Path(__file__).parent.parent / "bindings" / "examples"
sys.path.insert(0, str(examples_dir))

from common import MODELS, ROBOPLAN_EXAMPLES_DIR


def solve(scene: Scene, rrt: RRT, seed: int = 1234):
    scene.setRngSeed(seed)

    start = JointConfiguration()
    start.positions = scene.randomCollisionFreePositions()
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = scene.randomCollisionFreePositions()
    assert goal.positions is not None

    path = rrt.plan(start, goal)
    assert path is not None


def solve_many(scene: Scene, rrt: RRT, iterations: int = 10):
    for i in range(7):
        solve(scene, rrt, i + 1234)


def create_scene(model_name: str) -> Scene:
    model_data = MODELS[model_name]
    package_paths = [ROBOPLAN_EXAMPLES_DIR]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        f"{model_name}_benchmark_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )
    return scene


# The dual arm model needs planning groups or the benchmarks take forever.
@pytest.fixture(scope="session", params=["kinova", "ur5", "franka"])
def scene(request):
    return create_scene(request.param)


def test_benchmark_rrt(benchmark, scene):
    # Decreased connection distance to increase tree sizes.
    options = RRTOptions()
    options.max_nodes = 10000
    options.max_connection_distance = 0.1
    rrt = RRT(scene, options)

    benchmark(solve_many, scene, rrt, 10)


def test_benchmark_rrt_connect(benchmark, scene):
    options = RRTOptions()
    options.max_nodes = 10000
    options.max_connection_distance = 0.1
    options.rrt_connect = True
    rrt = RRT(scene, options)

    benchmark(solve_many, scene, rrt, 10)
