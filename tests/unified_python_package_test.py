from collections.abc import Iterable
from pathlib import Path
import re
import tomllib
from typing import cast

ROOT = Path(__file__).resolve().parents[1]
PYTHON_BINDING_PACKAGES = (
    "roboplan_example_models",
    "roboplan",
    "roboplan_simple_ik",
    "roboplan_oink",
    "roboplan_rrt",
    "roboplan_toppra",
    "roboplan_cartesian_planning",
)
DEPENDENT_PACKAGES = PYTHON_BINDING_PACKAGES[2:]
PACKAGES_WITH_EXAMPLE_MODEL_TEST_DEPENDENCY = (
    PYTHON_BINDING_PACKAGES[1],
    *PYTHON_BINDING_PACKAGES[3:],
)


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def _read_toml(path: str) -> dict[str, object]:
    return cast(dict[str, object], tomllib.loads(_read(path)))


def _assert_contains_all(source: str, expected: Iterable[str]) -> None:
    for value in expected:
        assert value in source, f"expected {value!r}"


def _assert_excludes_all(source: str, unexpected: Iterable[str]) -> None:
    for value in unexpected:
        assert value not in source, f"unexpected {value!r}"


def test_root_pyproject_defines_unified_roboplan_distribution() -> None:
    data = _read_toml("pyproject.toml")
    build_system = cast(dict[str, object], data["build-system"])
    build_requires = cast(list[str], build_system["requires"])
    project = cast(dict[str, object], data["project"])

    assert project["name"] == "roboplan"
    assert "version" not in project
    assert project["dynamic"] == ["version"]
    assert project["requires-python"] == ">=3.10,<3.15"
    assert "setuptools-scm >=8" in build_requires
    assert "cmeel-eigen[build]" in build_requires
    assert "cmeel-yaml-cpp[build]" in build_requires
    assert "libpinocchio[build] == 4.1.0" in build_requires
    assert "patchelf; platform_system == 'Linux'" in build_requires
    dependencies = cast(list[str], project["dependencies"])
    assert "roboplan-core" not in dependencies
    assert "numpy" in dependencies
    assert "pin >=4.1.0, <4.2" in dependencies
    assert "matplotlib" in dependencies

    tool = cast(dict[str, object], data["tool"])
    scikit_build = cast(dict[str, object], tool["scikit-build"])
    metadata = cast(dict[str, object], scikit_build["metadata"])
    version_metadata = cast(dict[str, str], metadata["version"])
    assert version_metadata["provider"] == "scikit_build_core.metadata.setuptools_scm"
    assert "setuptools_scm" in tool

    cmake = cast(dict[str, object], scikit_build["cmake"])
    assert cmake["source-dir"] == "packaging/python"
    defines = cast(dict[str, str], cmake["define"])
    assert defines["CMAKE_POLICY_VERSION_MINIMUM"] == "3.5"
    assert defines["CMAKE_CXX_STANDARD"] == "17"
    assert defines["CMAKE_CXX_STANDARD_REQUIRED"] == "ON"
    assert defines["CMAKE_INSTALL_LIBDIR"] == "lib"
    assert defines["CMAKE_INSTALL_RPATH"] == "$ORIGIN/../../lib"
    assert defines["CMAKE_INSTALL_RPATH_USE_LINK_PATH"] == "FALSE"

    sdist = cast(dict[str, list[str]], scikit_build["sdist"])
    assert ".omo/**" in sdist["exclude"]


def test_packaging_directory_is_ignored_by_colcon() -> None:
    assert (ROOT / "packaging/COLCON_IGNORE").exists()


def test_packaging_cmake_uses_scikit_build_dependency_prefix_only() -> None:
    packaging_cmake = _read("packaging/python/CMakeLists.txt")
    helper = _read("packaging/python/cmake/roboplan_python_packaging.cmake")

    _assert_contains_all(
        packaging_cmake,
        [
            "include(cmake/roboplan_python_packaging.cmake)",
            "roboplan_configure_scikit_build_prefix()",
        ],
    )
    _assert_excludes_all(packaging_cmake, ["roboplan_ensure_hpp_fcl_target"])
    _assert_excludes_all(helper, ["hpp-fcl", "/opt/ros"])


def test_root_cmake_superbuild_adds_all_python_binding_packages_in_order() -> None:
    source = _read("packaging/python/CMakeLists.txt")
    helper_source = _read("packaging/python/cmake/roboplan_python_packaging.cmake")
    package_order = re.findall(
        r'add_subdirectory\("\$\{ROBOPLAN_REPOSITORY_ROOT\}/(roboplan(?:_[a-z_]+)?)"\s+"[^/]+"\)',
        source,
    )

    repair_source = _read("packaging/python/cmake/repair_unified_rpaths.cmake.in")
    macos_repair_source = _read(
        "packaging/python/cmake/repair_unified_macos_rpaths.cmake.in"
    )

    _assert_contains_all(
        source,
        [
            "SKBUILD_PROJECT_VERSION",
            "roboplan_configure_scikit_build_prefix()",
            "roboplan_configure_unified_python_wheel()",
        ],
    )
    _assert_contains_all(
        helper_source,
        [
            "cmeel.prefix",
            "list(PREPEND CMAKE_PREFIX_PATH",
            "${ROBOPLAN_CMEEL_PREFIX}/lib",
            "${search_prefix}/lib/${pattern}",
            "boost_atomic",
            "roboplan_install_matching_libraries",
            "libboost_atomic.so.*",
            "libboost_filesystem.so.*",
            "liburdfdom_world.so.*",
            "liburdfdom_world.*.dylib",
            "liboctomap.*.dylib",
            "install(SCRIPT",
            "$ORIGIN/../../lib",
            "install_name_tool",
            "repair_unified_macos_rpaths.cmake",
            "@loader_path/../../lib",
        ],
    )
    _assert_excludes_all(helper_source, [".so.1.90.0", ".so.0.11.0", "install(CODE"])
    assert "--set-rpath" in repair_source
    _assert_contains_all(
        macos_repair_source,
        ["-add_rpath", "@loader_path/../../lib", "@loader_path"],
    )
    assert package_order == list(PYTHON_BINDING_PACKAGES)


def test_packaging_entrypoint_provides_build_tree_package_configs() -> None:
    helper = _read("packaging/python/cmake/roboplan_python_packaging.cmake")
    packaging_cmake = _read("packaging/python/CMakeLists.txt")

    assert "roboplan_register_build_tree_packages()" in packaging_cmake
    assert "roboplan_configure_cmeel_package" not in helper
    _assert_contains_all(
        helper,
        [
            "function(roboplan_register_build_tree_package package_name)",
            "${package_name}_DIR",
            "roboplan::roboplan=roboplan",
            "roboplan::filters=filters",
            "roboplan_example_models::roboplan_example_models=roboplan_example_models",
            "roboplan_cartesian_planning::roboplan_cartesian_planning=roboplan_cartesian_planning",
        ],
    )


def test_dependent_packages_keep_upstream_find_package_shape() -> None:
    for package in DEPENDENT_PACKAGES:
        path = f"{package}/CMakeLists.txt"
        source = _read(path)

        assert "find_package(roboplan REQUIRED)" in source, path
        assert "if(NOT TARGET roboplan::roboplan)" not in source, path


def test_binding_packages_keep_upstream_nanobind_discovery() -> None:
    for package in PYTHON_BINDING_PACKAGES:
        path = f"{package}/bindings/CMakeLists.txt"
        source = _read(path)

        assert "-m nanobind --cmake_dir" in source, path
        assert "find_package(nanobind CONFIG REQUIRED)" in source, path
        assert 'ROBOPLAN_VERSION="${PROJECT_VERSION}"' in source, path
        assert "ROBOPLAN_NANOBIND_PYTHON_RESULT" not in source, path


def test_dependent_package_tests_keep_upstream_example_model_dependency() -> None:
    for package in PACKAGES_WITH_EXAMPLE_MODEL_TEST_DEPENDENCY:
        path = f"{package}/test/CMakeLists.txt"
        source = _read(path)

        assert "find_package(roboplan_example_models REQUIRED)" in source, path
        assert (
            "if(NOT TARGET roboplan_example_models::roboplan_example_models)"
            not in source
        ), path


def test_python_binding_workflow_builds_repaired_wheels_for_pr_ci() -> None:
    workflow = _read(".github/workflows/build-pypi-wheels.yml")

    assert "pull_request:" in workflow
    assert "push:" in workflow
    assert "branches:" in workflow
    assert "- main" in workflow
    assert "workflow_call:" in workflow
    assert "name: Build PyPI wheels" in workflow
    assert "PACKAGE_NAME:" not in workflow
    assert 'project["name"] != "roboplan"' in workflow
    assert "fetch-depth: 0" in workflow
    assert workflow.count("fetch-depth: 0") == 3
    assert "setuptools-scm>=8" in workflow
    assert "get_version(root='.', local_scheme='no-local-version')" in workflow
    assert 'project["version"]' not in workflow
    assert "dynamic = ['version']" in workflow
    assert 'python-version: "3.12"' in workflow
    assert "RELEASE_TAG#v" not in workflow
    assert "^[0-9]+\\.[0-9]+\\.[0-9]+$" in workflow
    assert "pypa/cibuildwheel@" in workflow
    assert "fail-fast: false" in workflow
    assert "ubuntu-24.04-arm" in workflow
    assert "macos-13" not in workflow
    assert "macos-14" in workflow
    assert "CIBW_MANYLINUX_X86_64_IMAGE: manylinux_2_28" in workflow
    assert "CIBW_MANYLINUX_AARCH64_IMAGE: manylinux_2_28" in workflow
    assert "CIBW_ARCHS: ${{ matrix.archs }}" in workflow
    assert "archs: x86_64" in workflow
    assert "archs: aarch64" in workflow
    assert "archs: arm64" in workflow
    assert "artifact: linux-x86_64" in workflow
    assert "artifact: linux-aarch64" in workflow
    assert "artifact: macos-x86_64" not in workflow
    assert "artifact: macos-arm64" in workflow
    assert 'extra_environment: "MACOSX_DEPLOYMENT_TARGET=13.0"' not in workflow
    assert 'extra_environment: "MACOSX_DEPLOYMENT_TARGET=14.0"' in workflow
    assert "${{ matrix.extra_environment }}" in workflow
    assert 'CIBW_BUILD: "cp310-* cp311-* cp312-* cp313-* cp314-*"' in workflow
    assert "cp315" not in workflow
    assert "CIBW_ENABLE: cpython-prerelease" not in workflow
    assert 'CIBW_SKIP: "pp* *-musllinux*"' in workflow
    assert "CMAKE_BUILD_PARALLEL_LEVEL=2" in workflow
    assert 'MAKEFLAGS="-j2"' in workflow
    assert 'NINJAFLAGS="-j2"' in workflow
    assert "import roboplan; import roboplan.core" in workflow
    assert "roboplan.toppra" in workflow
    assert "roboplan.cartesian_planning" in workflow
    assert "dist/roboplan-*.tar.gz" in workflow
    assert "python-distributions-${{ matrix.artifact }}" in workflow
    assert "wheelhouse/roboplan-*.whl" in workflow


def test_release_workflow_reuses_python_binding_build_and_publishes() -> None:
    workflow = _read(".github/workflows/release.yml")

    assert '"v*"' not in workflow
    assert '"[0-9]*.[0-9]*.[0-9]*"' in workflow
    assert "uses: ./.github/workflows/build-pypi-wheels.yml" in workflow
    assert "pypa/cibuildwheel@" not in workflow
    assert "pypa/gh-action-pypi-publish@release/v1" in workflow
    assert workflow.count("pypa/gh-action-pypi-publish@release/v1") == 1
    assert "if: startsWith(github.ref, 'refs/tags/')" in workflow
    assert "id-token: write" in workflow
    assert "testpypi" not in workflow.lower()
    assert "https://pypi.org/project/roboplan/" in workflow
    assert "repository-url: https://test.pypi.org/legacy/" not in workflow
    assert "pattern: python-distributions-*" in workflow
    assert "TWINE_PASSWORD" not in workflow
    assert "__token__" not in workflow
