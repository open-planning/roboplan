"""
Basic import checks for the core roboplan package.
"""

import importlib
from importlib.metadata import PackageNotFoundError, version

import pytest
from packaging.version import parse


def test_import_roboplan_core() -> None:
    assert importlib.util.find_spec("roboplan.core")
    importlib.import_module("roboplan.core")


def test_roboplan_core_version_attr() -> None:
    import roboplan.core

    ver = roboplan.core.__version__
    assert ver == "0.3.0", "Incorrect roboplan-core version in module attribute"


def test_roboplan_core_version_metadata() -> None:
    # Distribution metadata only exists for pip/wheel installs. CMake/colcon
    # installs (e.g. pixi) place the package on the path without a .dist-info,
    # so skip rather than fail when no metadata is registered.
    try:
        meta_version = version("roboplan-core")
    except PackageNotFoundError:
        pytest.skip(
            "roboplan-core distribution metadata not available (non-wheel install)"
        )
    assert meta_version == "0.3.0", "Incorrect roboplan-core version in metadata"


def test_import_pinocchio() -> None:
    assert importlib.util.find_spec("pinocchio")
    pinocchio = importlib.import_module("pinocchio")
    assert parse(pinocchio.__version__) >= parse("3.9.0")
