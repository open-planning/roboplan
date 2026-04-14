"""
Basic sanity checks for RoboPlan module.
"""

import importlib
from importlib.metadata import version
from packaging.version import parse


def test_import_roboplan() -> None:
    assert importlib.util.find_spec("roboplan")
    importlib.import_module("roboplan")


def test_roboplan_version_attr() -> None:
    import roboplan

    ver = roboplan.__version__
    assert ver == "0.2.0", "Incorrect RoboPlan version in module attribute"


def test_roboplan_version_metadata() -> None:
    assert version("roboplan") == "0.2.0", "Incorrect RoboPlan version in metadata"


def test_import_pinocchio() -> None:
    assert importlib.util.find_spec("pinocchio")
    pinocchio = importlib.import_module("pinocchio")
    assert parse(pinocchio.__version__) >= parse("3.9.0")
