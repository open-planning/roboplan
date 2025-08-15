from pathlib import Path

import xacro

import roboplan
from roboplan import get_package_share_dir


if __name__ == "__main__":

    roboplan_examples_dir = Path(get_package_share_dir())
    urdf_path = roboplan_examples_dir / "franka_robot_model" / "dual_fr3.urdf"
    srdf_path = roboplan_examples_dir / "franka_robot_model" / "dual_fr3.srdf"
    package_paths = [roboplan_examples_dir]

    urdf = xacro.process_file(urdf_path).toxml()
    srdf = xacro.process_file(srdf_path).toxml()

    scene = roboplan.Scene.from_xml("dual_arm_scene", urdf, srdf, package_paths)
    print(scene)
