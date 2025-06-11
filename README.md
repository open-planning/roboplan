# roboplan
Modern and performant robot motion planning library based on Pinocchio.

> [!WARNING]
> This is an experimental, work-in-progress repository!

---

## Design philosophy

### Not a monolith
Several tools optimize their design for runtime configurability via YAML config files and plugins that rely on abstract interface classes for motion planners, IK solvers, etc.
This library shall instead establish _standard data types_ for things like joint states, paths, and trajectories.
It is strongly recommended that implementations use these data types in their interfaces as much as possible.

This does mean that switching to different planners (for example) requires recompiling your code, but we consider this worthwhile to keep the codebase simpler and more flexible.

### Middleware is optional
The core library is standalone.
Middleware such as ROS, and all its specific tools (message definitions, pub/sub, parameters, etc.) shall be available as _optional_, lightweight wrappers around the core... in a separate repository.

As a side benefit, this means that community contributors can provide their own connections to different middleware while leveraging the core library as-is.

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
- `roboplan_examples` : Basic examples with real robot models.

---

## Build instructions (colcon)

First, clone this repo to a valid ROS 2 workspace.

```bash
mkdir -p ~/roboplan_ws/src
cd ~/roboplan_ws/src
git clone https://github.com/sea-bass/roboplan.git
```

Source your favorite ROS distro and compile the package.

```bash
source /opt/ros/rolling/setup.bash
cd ~/roboplan_ws
colcon build
```

Now you should be able to run a basic example.

```bash
source install/setup.bash
ros2 run roboplan_examples test_scene
```

See the [bindings README](bindings/README.md) for instructions on building the Python bindings.
