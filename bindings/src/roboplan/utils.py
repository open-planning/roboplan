from pathlib import Path

import roboplan


def get_package_path():
    roboplan_file = Path(roboplan.__file__)
    return Path(roboplan_file.parent)


def get_example_resources_directory():
    # TODO: Could replace this with the install location but for now it's
    #       assuming this exists in a compiled workspace.
    roboplan_path = get_package_path()
    roboplan_path = roboplan_path.parent.parent.parent.parent.parent
    return (
        roboplan_path
        / "install"
        / "roboplan_example_models"
        / "share"
        / "roboplan_example_models"
        / "models"
    )
