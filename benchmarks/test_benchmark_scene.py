import sys
from pathlib import Path

import pytest

from roboplan.core import Scene

# The examples are not an installed package, so add their directory to the path to import `common`.
examples_dir = Path(__file__).parent.parent / "roboplan_examples" / "python"
sys.path.insert(0, str(examples_dir))

from common import build_scene


def has_collisions_many(scene: Scene, qs) -> int:
    """Calls hasCollisions() for every configuration in `qs` and return the collision count."""
    return sum(scene.hasCollisions(q) for q in qs)


@pytest.fixture(scope="session", params=["so101", "dual"])
def model_name(request):
    return request.param


@pytest.fixture(
    scope="session", params=[False, True], ids=["no_obstacles", "with_obstacles"]
)
def with_obstacles(request):
    return request.param


@pytest.fixture(scope="session")
def collision_benchmark_setup(model_name, with_obstacles):
    scene = build_scene(model_name, with_obstacles=with_obstacles)
    scene.setRngSeed(1234)
    qs = [scene.randomPositions() for _ in range(100)]
    return {"scene": scene, "qs": qs}


def test_benchmark_has_collisions(benchmark, collision_benchmark_setup):
    scene = collision_benchmark_setup["scene"]
    qs = collision_benchmark_setup["qs"]
    benchmark(has_collisions_many, scene, qs)
