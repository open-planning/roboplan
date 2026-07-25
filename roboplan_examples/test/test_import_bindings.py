"""
Smoke test that the Python bindings of all roboplan packages are importable.

This catches packaging issues for downstream packages that may use the bindings.
"""

import importlib

import pytest

BINDINGS_MODULES = [
    "roboplan.cartesian_planning",
    "roboplan.core",
    "roboplan.example_models",
    "roboplan.filters",
    "roboplan.optimal_ik",
    "roboplan.rrt",
    "roboplan.simple_ik",
    "roboplan.toppra",
]


@pytest.mark.parametrize("module", BINDINGS_MODULES)
def test_import_bindings(module: str) -> None:
    importlib.import_module(module)
