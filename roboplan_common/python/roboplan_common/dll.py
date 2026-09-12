"""Windows DLL search-path support for RoboPlan extension modules."""

import os
import sys
from functools import cache


@cache
def add_dll_directories() -> list[object]:
    """Register DLL directories from the active Python, Conda, and colcon prefixes."""
    if os.name != "nt":
        return []

    prefixes = [sys.prefix, os.environ.get("CONDA_PREFIX", "")]
    for variable in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        prefixes.extend(os.environ.get(variable, "").split(os.pathsep))

    handles = []
    seen_directories = set()
    for prefix in prefixes:
        for directory in (
            os.path.join(prefix, "bin"),
            os.path.join(prefix, "Library", "bin"),
        ):
            if (
                prefix
                and directory not in seen_directories
                and os.path.isdir(directory)
            ):
                seen_directories.add(directory)
                handles.append(os.add_dll_directory(directory))

    return handles
