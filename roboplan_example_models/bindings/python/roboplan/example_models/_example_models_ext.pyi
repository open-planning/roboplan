import pathlib


def get_install_prefix() -> pathlib.Path:
    """
    Returns the install prefix, located at runtime from this shared library.
    """

def get_package_share_dir() -> pathlib.Path:
    """Returns the `share` directory under the install prefix."""

def get_package_models_dir() -> pathlib.Path:
    """
    Returns the example robot models directory (`share/roboplan_example_models/models`).
    """
