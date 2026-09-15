# roboplan_mujoco

A native [MuJoCo](https://mujoco.org/) simulation backend for RoboPlan.

The package:

- imports RoboPlan URDF models into an editable MuJoCo `mjSpec`;
- attaches robots to MJCF scenes and compiles them into an owning simulation;
- maps RoboPlan/Pinocchio `q`, `v`, and `tau` vectors to MuJoCo state and actuator arrays;
- supports position servo control and direct kinematic configuration updates; and
- provides a lightweight interactive renderer with planning overlays.

Applications own the control loop and decide when to send commands, step the simulation, and
render frames.

## Build and install

[Pixi](https://pixi.sh/) is the recommended workflow. From the RoboPlan repository root:

```bash
pixi install
pixi run --environment default build
pixi run --environment default install
```

Pixi installs MuJoCo, GLFW, TinyXML2, and the Python dependencies used for model preparation.
When using CMake directly, MuJoCo 3.12.0 is fetched automatically if it is not already available;
the other native dependencies must be installed separately.

Run the complete test suite with:

```bash
pixi run --environment default test_all
```

## Prepare robot models

The models in `roboplan_example_models/models/` remain the canonical robot descriptions. The
preparation script derives MuJoCo compatible URDFs by expanding Xacro files, converting unsupported
meshes, preserving materials, and normalizing geometry names.

Prepare every registered model:

```bash
pixi run --environment default python roboplan_mujoco/scripts/prepare_urdf.py
```

Or prepare selected models:

```bash
pixi run --environment default python roboplan_mujoco/scripts/prepare_urdf.py franka
```

Generated files are written to `roboplan_example_models/models/mujoco/<robot>/`.

## Run the example

After preparing the Franka model and building the repository:

```bash
pixi run --environment default \
  ./build/roboplan_examples/cpp/mujoco_sim/example_pick_place
```