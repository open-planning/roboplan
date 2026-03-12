import pathlib


def get_install_prefix() -> pathlib.Path:
    """Provides compile time access to the resources install directory."""

def get_package_share_dir() -> pathlib.Path:
    """
    Provides compile time access to the resources shared directory for accessing robot models or other resource files.
    """

def get_package_models_dir() -> pathlib.Path:
    """
    Provides compile time access to the directory under the resources shared directory which contains all the example robot models.
    """
