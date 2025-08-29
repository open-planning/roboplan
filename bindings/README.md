Python bindings to RoboPlan.

## Build instructions (colcon)

First, build the regular `roboplan` package.

```bash
colcon build
source install/setup.bash
```

As a one-time install, you should install the necessary Python packages to build the bindings.

```bash
pip install --break-system-packages nanobind scikit-build-core
```

Once you have built this, you can install the bindings.

```bash
cd bindings
pip install --break-system-packages --no-build-isolation -ve .
```

At this point, you should be able to use `roboplan` as a Python package!

```bash
python3
>>> import roboplan
```

... or run one of the examples.

```bash
python3 examples/example_scene.py
```
