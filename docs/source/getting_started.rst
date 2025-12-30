Getting Started
===============

First, clone this repo (including submodules) to a valid ROS 2 workspace.

::

    mkdir -p ~/roboplan_ws/src
    cd ~/roboplan_ws/src
    git clone --recursive https://github.com/open-planning/roboplan.git
    cd roboplan

There are currently 3 supported ways to work with RoboPlan.

---

Vanilla CMake
-------------

One of the design points of this library is that it should be portable, and therefore compiles with "vanilla" CMake.

**We do not recommend using this workflow;** it's more of an exercise in making sure the software can compile without any dependencies that lock it into a particular ecosystem.

If you do want to use regular CMake, you should take a look at the Dockerfile under ``.docker/ubuntu``.
Alternatively, you can try it for yourself.

::

    export UBUNTU_VERSION=24.04
    docker compose build ubuntu

Once the Docker image is built, you can try running the code in the image.

::

    docker compose run ubuntu bash

Inside the shell, you can try different commands, such as.

::

    ./build/roboplan_examples/cpp/example_scene
    python3 roboplan_examples/python/example_ik.py

To run the unit tests, you can do:

::

    scripts/run_tests.bash

---

Pixi
----

Another way to manage the environment is with the `Pixi <https://pixi.sh>`_ package management tool.

First, install Pixi using `these instructions <https://pixi.sh/latest/#installation>`_.

Once set up, you can use the ``pixi`` verbs as follows.

::

    # Build all packages, including Python bindings
    pixi run build_all

    # Install all packages
    pixi run install_all

    # This will only build the package (You must have built the dependencies first)
    pixi run build PACKAGE_NAME

    # This will only install the package
    pixi run install PACKAGE_NAME

Once built, you can use the Pixi shell to run specific examples.

::

    pixi shell
    ./build/roboplan_examples/cpp/example_scene
    python3 roboplan_examples/python/example_ik.py


To run the unit tests:

::

    # Test all packages
    pixi run test_all

    # Test a specific package
    pixi run test PACKAGE_NAME

To lint the code:

::

    pixi run lint


Build with AddressSanitizer (ASan)

::

    pixi run build_asan PACKAGE_NAME

Build with compilation time report

::

    pixi run build_timetrace PACKAGE_NAME

---


ROS 2 (colcon)
--------------

If you are using `ROS 2 <https://docs.ros.org/>`_, you can build RoboPlan with the ``colcon`` build system.

Source your favorite ROS distro and compile the package.

::

    source /opt/ros/rolling/setup.bash
    cd ~/roboplan_ws
    rosdep install --from-paths src -y --ignore-src
    colcon build

**NOTE:** To compile tests, you may also need to install GTest and GMock:

::

    sudo apt install libgtest-dev libgmock-dev

Now you should be able to run a basic example.

::

    source install/setup.bash
    ros2 run roboplan_examples example_scene
    ros2 run roboplan_examples example_ik.py

At this point, you should also be able to use `roboplan` as a Python package!

::

    python3
    >>> import roboplan

To run the unit tests, you can simply use ``colcon``:

::

    colcon test
    colcon test --packages-select roboplan --event-handlers console_direct+
