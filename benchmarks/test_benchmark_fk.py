import sys
from pathlib import Path

import pytest

from roboplan.core import Scene

# The examples are not an installed package, so add their directory to the path to import `common`.
examples_dir = Path(__file__).parent.parent / "roboplan_examples" / "python"
sys.path.insert(0, str(examples_dir))

from common import build_scene, get_model_data


def fk_many(scene: Scene, frame_name: str, qs) -> None:
    """Calls forwardKinematics() for every configuration in `qs`."""
    for q in qs:
        scene.forwardKinematics(q, frame_name)


@pytest.fixture(scope="session", params=["so101", "kinova", "ur5", "franka", "dual"])
def fk_benchmark_setup(request):
    model_name = request.param
    model_data = get_model_data()[model_name]
    scene = build_scene(model_name)
    scene.setRngSeed(1234)
    qs = [scene.randomPositions() for _ in range(100)]
    return {
        "scene": scene,
        "frame_name": model_data.ee_names[0],
        "qs": qs,
    }


def test_benchmark_forward_kinematics(benchmark, fk_benchmark_setup):
    s = fk_benchmark_setup
    benchmark(fk_many, s["scene"], s["frame_name"], s["qs"])
