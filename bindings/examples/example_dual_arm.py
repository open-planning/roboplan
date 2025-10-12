import xacro

from common import MODELS, ROBOPLAN_EXAMPLES_DIR
import roboplan


if __name__ == "__main__":

    model = "franka"
    model_data = MODELS[model]

    urdf = xacro.process_file(model_data.urdf_path).toxml()
    srdf = xacro.process_file(model_data.srdf_path).toxml()
    package_paths = [ROBOPLAN_EXAMPLES_DIR]

    # Specify argument names to distinguish overloaded Scene constructors from python.
    scene = roboplan.Scene(
        "dual_arm_scene", urdf=urdf, srdf=srdf, package_paths=package_paths
    )
    print(scene)
