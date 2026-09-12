# RoboPlan superbuild and Python packaging

This directory is RoboPlan's shared CMake entry point for development: it configures every package in one build tree so `pixi run build`, `pixi run install`, and `pixi run test` run once instead of once per package.

PyPI wheels do **not** use the superbuild. Each package directory (`roboplan_core`, `roboplan_rrt`, ...) is its own `cmeel.build` project with its own `pyproject.toml`, and the `roboplan` directory holds a pure-Python metapackage that depends on all of them. Downstream packages pin their siblings to an exact version (for example `roboplan-core ==0.6.1`) as both build and runtime requirements, so a local build has to be done in dependency order against a local wheelhouse.

Release wheels are built with `cibuildwheel` in the [`build-pypi-wheels.yml`](../.github/workflows/build-pypi-wheels.yml) CI workflow and smoke-tested by importing every compiled submodule. The [`build_and_test.yml`](../.github/workflows/build_and_test.yml) and [`release.yml`](../.github/workflows/release.yml) workflows invoke it, so wheel builds use the same triggers as the overall CI workflow and tagged releases can publish fresh artifacts through PyPI trusted publishing.

## Build the wheels locally

Run commands from the repository root. The `wheelhouse/` directory is gitignored.
cmeel drives the native build with `-j$CMEEL_JOBS`, which defaults to 4. Raise it on a large machine, or cap it at 2 to match CI:

```bash
export CMEEL_JOBS=$(nproc)
```

Build the two pure-Python wheels first so the compiled packages can resolve them:

```bash
mkdir -p wheelhouse
uv build --wheel --out-dir wheelhouse roboplan_common
uv build --wheel --out-dir wheelhouse roboplan
```

Then build the compiled packages in dependency order.
`--find-links` lets each package's isolated build environment pick up the sibling wheels built so far instead of looking for them on PyPI:

```bash
for pkg in roboplan_core roboplan_example_models roboplan_oink roboplan_rrt \
           roboplan_simple_ik roboplan_toppra roboplan_cartesian_planning; do
  uv build --wheel --find-links wheelhouse --out-dir wheelhouse "$pkg"
done
```

To iterate on a single package, rebuild just that package and anything downstream of it with the same command.
Wheels are tagged with the interpreter `uv build` ran with, so keep one Python version per wheelhouse.

## Import-test the wheels

Install the metapackage from the local wheelhouse into a fresh environment and import every compiled submodule, mirroring the CI smoke test:

```bash
uv venv --seed /tmp/roboplan-wheel-check
uv pip install --python /tmp/roboplan-wheel-check/bin/python --find-links wheelhouse roboplan
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

## Run an example against the wheels

To prove the wheels are usable end-to-end, run one of the examples from the same environment.
The examples are not part of any wheel, so they pull in a few extra runtime dependencies on top of `roboplan` itself.

```bash
uv pip install --python /tmp/roboplan-wheel-check/bin/python pycollada tyro xacro viser
/tmp/roboplan-wheel-check/bin/python roboplan_examples/python/example_ik.py
```
