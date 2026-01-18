import os
from pathlib import Path
import subprocess

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
import sys

sys.path.insert(0, os.path.abspath("../../bindings/src"))

# -- Project information -----------------------------------------------------

project = "roboplan"
copyright = "2025-2026, Open Planning"
author = "Sebastian Castro"

# The full version, including alpha/beta/rc tags
version = release = "0.1.0"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "autoapi.extension",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx_autodoc_typehints",
    "sphinx.ext.autosummary",
    "sphinx_rtd_theme",
    "sphinx_copybutton",
    "breathe",
]

# Add any paths that contain templates here, relative to this directory.
# templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns: list[str] = []

# Mock imports for external dependencies.
autodoc_mock_imports = ["roboplan.roboplan_ext"]
autoapi_type = "python"
autoapi_add_toctree_entry = False
autoapi_dirs = ["../../bindings/src"]
autodoc_typehints = "description"

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"
master_doc = "index"

# -- Options for breathe -----------------------------------------------------
breathe_default_project = "roboplan"
breathe_projects = {}
breathe_projects_source = {}

for package in [
    "roboplan",
    "roboplan_rrt",
    "roboplan_simple_ik",
    "roboplan_toppra",
    "roboplan_example_models",
]:
    # Generate Doxygen XML and add it to the breathe projects.
    subprocess.call(f"cd ../../{package}/docs; rm -rf html/ xml/; doxygen", shell=True)
    breathe_projects[package] = (os.path.abspath(f"../../{package}/docs/xml"),)

    # Add the package files to the breathe projects sources list.
    package_path = Path(os.path.abspath(f"../../{package}/include/{package}"))
    breathe_projects_source[package] = (
        package_path.as_posix(),
        [f for f in package_path.rglob("*.hpp")],
    )
