#!/usr/bin/env python3

import xacro

from common import MODELS
from roboplan.core import Scene
from roboplan.example_models import get_package_share_dir


if __name__ == "__main__":

    model = "dual"
    model_data = MODELS[model]

    urdf = xacro.process_file(model_data.urdf_path).toxml()
    srdf = xacro.process_file(model_data.srdf_path).toxml()
    package_paths = [get_package_share_dir()]

    # Specify argument names to distinguish overloaded Scene constructors from python.
    scene = Scene("dual_arm_scene", urdf=urdf, srdf=srdf, package_paths=package_paths)
    print(scene)
