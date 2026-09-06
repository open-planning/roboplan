# RoboPlan superbuild and Python packaging

This directory is RoboPlan's shared CMake entry point. The PyPI backend is
`cmeel.build`, which configures this same superbuild and installs it into cmeel's
isolated prefix. cmeel exposes the prefix's Python site-packages directory through
its generated `.pth` file and discovers installed cmeel dependencies through
`CMAKE_PREFIX_PATH`.

Release wheels are built with `cibuildwheel` in the [`build-pypi-wheels.yml`](../.github/workflows/build-pypi-wheels.yml) CI workflow, and are smoke-tested by importing the `roboplan` namespace and every compiled submodule.
The [`build_and_test.yml`](../.github/workflows/build_and_test.yml) and [`release.yml`](../.github/workflows/release.yml) workflows invoke it, so wheel builds use the same triggers as the overall CI workflow and tagged releases can publish fresh artifacts through PyPI trusted publishing.


## Local checks

Run commands from the repository root. Keep native parallelism capped on small machines, matching CI:

```bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
export NINJAFLAGS=-j2
```

Source-build and import-test the unified wheel path:

```bash
uv venv --seed /tmp/roboplan-wheel-check
uv pip install -v --python /tmp/roboplan-wheel-check/bin/python --no-cache .
/tmp/roboplan-wheel-check/bin/python - <<'PY'
import roboplan
import roboplan.core
import roboplan.filters
import roboplan.example_models
import roboplan.simple_ik
import roboplan.optimal_ik
import roboplan.rrt
import roboplan.toppra
import roboplan.cartesian_planning
print("roboplan imports ok")
PY
```

Check the structural packaging contract:

```bash
uvx pytest tests/unified_python_package_test.py -q
```

## Run an example against the wheel

To prove the wheel is usable end-to-end, install it into a fresh environment and run one of the examples.
The examples are not part of the wheel, so they pull in a few extra runtime dependencies on top of `roboplan` itself.

```bash
# 1. Create an isolated environment and install the wheel we just built.
uv venv --seed /tmp/roboplan-wheel-check
uv pip install --python /tmp/roboplan-wheel-check/bin/python --no-cache .

# 2. Install the extra dependencies the examples need.
uv pip install --python /tmp/roboplan-wheel-check/bin/python pycollada tyro xacro viser

# 3. Run an example.
/tmp/roboplan-wheel-check/bin/python roboplan_examples/python/example_ik.py
```
