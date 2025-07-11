"""
Basic sanity checks for RoboPlan module.
"""

import importlib
from importlib.metadata import version


def test_import_roboplan() -> None:
    assert importlib.util.find_spec("roboplan")
    importlib.import_module("roboplan")


def test_roboplan_version() -> None:
    ver = version("roboplan")
    assert ver == "0.0.0", "Incorrect RoboPlan version"


def test_import_pinocchio() -> None:
    assert importlib.util.find_spec("pinocchio")
    importlib.import_module("pinocchio")


def test_import_viser() -> None:
    assert importlib.util.find_spec("viser")
    importlib.import_module("viser")
