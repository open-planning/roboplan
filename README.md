# roboplan

Modern robot motion planning library based on Pinocchio.

> [!WARNING]
> This is an experimental, work-in-progress repository!

---

## Design philosophy

### Not a monolith

Several tools optimize their design for runtime configurability via YAML config files and plugins that rely on abstract interface classes for motion planners, IK solvers, etc.
This library shall instead establish _standard data types_ for things like joint states, paths, and trajectories.
It is strongly recommended that implementations use these data types in their interfaces as much as possible.

This does mean that switching to different planners (for example) requires changing your code. But, grounded in a decade of experience developing runtime-configurable systems, we consider this worthwhile to keep the codebase simple and flexible.

### Middleware is optional

The core library is standalone.
Middleware such as ROS, and all its specific tools (message definitions, pub/sub, parameters, etc.) shall be available as _optional_, lightweight wrappers around the core... in a separate repository.

Consequently, community contributors can leverage the core library in any project as-is and can consider middleware connections tailored to their use-cases.

### Bindings are first-class

Users should be able to `pip install` the Python bindings and get to hacking, debugging, and visualizing as quickly as possible.
New users can develop directly using Python, whereas intermediate/advanced users can directly use C++ for performance.
Contributors are expected to implement new features in C++ and provide working Python bindings.

---

## Packages list

This is all still very much work in progress!
Still debating whether this should be monorepo or multi-repo...

- `roboplan` : The core C++ library.
- `roboplan_simple_ik` : A simple inverse kinematics (IK) solver.
- `roboplan_rrt` : A Rapidly-exploring Random Tree (RRT) based motion planner.
- `roboplan_toppra` : A wrapper around the TOPP-RA algorithm for trajectory timing.
- `roboplan_example_models` : Contains robot models used for testing and examples.
- `roboplan_examples` : Basic examples with real robot models.

---

## Build instructions (colcon)

First, clone this repo (including submodules) to a valid ROS 2 workspace.

```bash
mkdir -p ~/roboplan_ws/src
cd ~/roboplan_ws/src
git clone --recursive https://github.com/open-planning/roboplan.git
```

Source your favorite ROS distro and compile the package.

```bash
source /opt/ros/rolling/setup.bash
cd ~/roboplan_ws
rosdep install --from-paths src -y --ignore-src
colcon build
```

**NOTE:** To compile tests, you may also need to install GTest and GMock:

```bash
sudo apt install libgtest-dev libgmock-dev
```

Now you should be able to run a basic example.

```bash
source install/setup.bash
ros2 run roboplan_examples example_scene
```

See the [bindings README](bindings/README.md) for instructions on building the Python bindings.

## Build instructions (pixi)

### Build instructions

Make sure to install [pixi](https://pixi.sh/latest/#installation).

```bash
git clone https://github.com/open-planning/roboplan.git
cd roboplan
# Build all packages
pixi run build_all
# Install all packages
pixi run install_all
# This will only build the package (You must have built the dependencies first)
pixi run build PACKAGE_NAME
# This will only install the package
pixi run install PACKAGE_NAME
```

### Run tests

```bash
pixi run test_all
# Test a specific package
pixi run test PACKAGE_NAME
```

### Linting

```bash
pixi run lint
```

### Build with AddressSanitizer (ASan)

```bash
pixi run build_asan PACKAGE_NAME
```

### Build with compilation time report

```bash
pixi run build_timetrace PACKAGE_NAME
```
