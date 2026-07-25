# The `roboplan.filters` bindings, backed by the compiled `_filters_ext` module.
import os

# Python >= 3.8 on Windows does not search PATH for the DLLs an extension
# module links against, and in a colcon workspace they live outside this
# package. Register the PATH entries explicitly before importing.
if os.name == "nt":
    for _entry in os.environ.get("PATH", "").split(os.pathsep):
        if _entry and os.path.isdir(_entry):
            os.add_dll_directory(_entry)

from ._filters_ext import *  # noqa: F401,F403
from ._filters_ext import __version__  # noqa: F401
