# The `roboplan.aligator` bindings, backed by `_aligator_ext`.
# Import core first to guarantee its types are registered before use.
import roboplan.core  # noqa: F401

from ._aligator_ext import *
from ._aligator_ext import __version__  # noqa: F401
