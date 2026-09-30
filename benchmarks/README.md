# Benchmarks

Benchmark suites for core RoboPlan functions built with `pytest-benchmark`.

Note: for CI we require a `BENCHMARK_TOKEN` secret.
This is a fine-grained access token with the **Contents: write** and **Pull requests: write** permissions.

## Running locally

```bash
# Run everything
pixi run -e default test_benchmarks

# Run a single suite (e.g. scene benchmarks)
pixi run -e default python -m pytest benchmarks/test_benchmark_scene.py

# Run a combo of model and params
pixi run -e default python -m pytest benchmarks/test_benchmark_scene.py -k "franka and 50"

# Compare against a previous run
pixi run -e default python -m pytest benchmarks/ --benchmark-autosave
pixi run -e default pytest-benchmark compare
```

`test_benchmarks` writes `pytest_benchmarks.json` (pytest-benchmark's `--benchmark-json` output) in the repo root.
