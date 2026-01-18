# This permits the modules to be used as, e.g.,
#    from roboplan.core import Box
import sys
from .roboplan_ext import __version__
from .roboplan_ext import core as _core
from .roboplan_ext import example_models as _example_models
from .roboplan_ext import rrt as _rrt
from .roboplan_ext import simple_ik as _simple_ik
from .roboplan_ext import toppra as _toppra

# Make the submodules valid for import
sys.modules[__name__ + ".core"] = _core
sys.modules[__name__ + ".example_models"] = _example_models
sys.modules[__name__ + ".rrt"] = _rrt
sys.modules[__name__ + ".simple_ik"] = _simple_ik
sys.modules[__name__ + ".toppra"] = _toppra
