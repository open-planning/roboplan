# The `roboplan.simple_ik` bindings, backed by `_simple_ik_ext`.
# Import core first to guarantee its types are registered before use.
import roboplan.core  # noqa: F401

from ._simple_ik_ext import *  # noqa: E402,F401,F403
from ._simple_ik_ext import __version__  # noqa: E402,F401
