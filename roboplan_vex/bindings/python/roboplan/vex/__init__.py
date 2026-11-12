# The `roboplan.vex` bindings, backed by `_vex_ext`.
# Import core first to guarantee its types are registered before use.
from roboplan_common import add_dll_directories

_dll_directories = add_dll_directories()

import roboplan.core  # noqa: F401

from ._vex_ext import *
from ._vex_ext import __version__  # noqa: F401
