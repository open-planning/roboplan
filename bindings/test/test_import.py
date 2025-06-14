"""
Basic sanity checks for RoboPlan module.
"""

import importlib
from importlib.metadata import version


def test_import() -> None:
    assert importlib.util.find_spec("roboplan")


def test_version() -> None:
    ver = version("roboplan")
    assert ver == "0.0.0", "Incorrect RoboPlan version"
