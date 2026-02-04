Getting Started
===============

First, clone this repo *including submodules*.

::

    git clone --recursive https://github.com/open-planning/roboplan.git
    cd roboplan

Minimally, this will give you access to the examples so you can run them regardless of how you installed RoboPlan.

The rest of this page shows various ways of getting started with RoboPlan.

---


Pre-built
---------

RoboPlan is available via `conda-forge <https://conda-forge.org/>`_ for easy installation.

To get started, first `install conda <https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html>`_.

We recommend creating your own environment for isolation.

::

    conda create -n roboplan -c conda-forge roboplan-python
    conda activate roboplan

In your new environment, you can import the ``roboplan`` Python bindings.

::

    python
    >>> import roboplan

From here, you can run the examples included in this repository.
For example, if you cloned the repo to a ``roboplan`` subfolder:

::

    python roboplan/roboplan_examples/python/example_ik.py

---


From Source
-----------

There are currently 3 supported ways to build RoboPlan from source.

Pixi
~~~~

Our recommended workflow is to use the `Pixi <https://pixi.sh>`_ package management tool.

First, install Pixi using `these instructions <https://pixi.sh/latest/#installation>`_.

Once set up, you can run the ``pixi`` tasks as follows.

::

    # Build all packages, including Python bindings
    pixi run build_all

    # Install all packages
    pixi run install_all

    # This will only build the package (You must have built the dependencies first)
    pixi run build PACKAGE_NAME

    # This will only install the package
    pixi run install PACKAGE_NAME

After building all the packages, you can use the Pixi shell to run specific examples.

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
~~~~~~~~~~~~~~

If you are using `ROS 2 <https://docs.ros.org/>`_, you can build RoboPlan with the ``colcon`` build system.

For this workflow, you should clone the repo (again, *including submodules*) to a valid ROS 2 workspace.

::

    mkdir -p ~/roboplan_ws/src
    cd ~/roboplan_ws/src
    git clone --recursive https://github.com/open-planning/roboplan.git

Source your favorite ROS distro and build the workspace.

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

At this point, you should also be able to use RoboPlan as a Python package!

::

    python3
    >>> import roboplan

To run the unit tests, you can simply use ``colcon``:

::

    colcon test
    colcon test --packages-select roboplan --event-handlers console_direct+

---


Vanilla CMake
~~~~~~~~~~~~~

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
