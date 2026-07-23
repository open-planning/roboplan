# The `roboplan.cartesian_planning` bindings, backed by `_cartesian_ext`.
# Import core first to guarantee its types are registered before use.
import os

# Python >= 3.8 on Windows does not search PATH for the DLLs an extension
# module links against, and in a colcon workspace they live outside this
# package. Register the PATH entries explicitly before importing.
if os.name == "nt":
    for _entry in os.environ.get("PATH", "").split(os.pathsep):
        if _entry and os.path.isdir(_entry):
            os.add_dll_directory(_entry)

import roboplan.core  # noqa: F401

from ._cartesian_ext import *  # noqa: E402,F401,F403
from ._cartesian_ext import __version__  # noqa: E402,F401
