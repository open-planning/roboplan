# The `roboplan.toppra` bindings, backed by `_toppra_ext`.
# Import core first to guarantee its types are registered before use.
import roboplan.core  # noqa: F401

from ._toppra_ext import *  # noqa: E402,F401,F403
from ._toppra_ext import __version__  # noqa: E402,F401
