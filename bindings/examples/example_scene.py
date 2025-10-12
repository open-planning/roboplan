import numpy as np

from common import MODELS, ROBOPLAN_EXAMPLES_DIR
import roboplan
from roboplan import get_package_share_dir


if __name__ == "__main__":

    jc = roboplan.JointConfiguration(
        ["joint_1", "joint_2", "joint_3"],
        np.array([0.1, 0.2, 0.3]),
    )
    print(f"Config names: {jc.joint_names}")
    print(f"Config positions: {jc.positions}")
    print("")

    model = "ur5"
    model_data = MODELS[model]
    package_paths = [ROBOPLAN_EXAMPLES_DIR]

    scene = roboplan.Scene(
        "test_scene", model_data.urdf_path, model_data.srdf_path, package_paths
    )
    print(scene)
