# The `roboplan.example_models` bindings, backed by `_example_models_ext`.
import os

# Python >= 3.8 on Windows does not search PATH for the DLLs an extension
# module links against, and in a colcon workspace they live outside this
# package. Register the PATH entries explicitly before importing.
if os.name == "nt":
    for _entry in os.environ.get("PATH", "").split(os.pathsep):
        if _entry and os.path.isdir(_entry):
            os.add_dll_directory(_entry)

from ._example_models_ext import *  # noqa: F401,F403
from ._example_models_ext import __version__  # noqa: F401
