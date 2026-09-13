Installation
============

First, clone this repo.

::

    git clone https://github.com/open-planning/roboplan.git
    cd roboplan

Minimally, this will give you access to the examples so you can run them regardless of how you installed RoboPlan.

The rest of this page shows various ways of getting started with RoboPlan.

---


Pre-built
---------

RoboPlan is available via `PyPi <https://pypi.org/>`_ and `conda-forge <https://conda-forge.org/>`_ for easy installation.

Conda (recommended)
~~~~~~~~~~~~~~~~~~~

**Supported platforms:** Linux, macOS, Windows

To get started, first `install conda <https://docs.conda.io/projects/conda/en/latest/user-guide/install/index.html>`_.

We recommend creating your own environment for isolation, installing all the libraries with Python bindings.

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

For each package in this repository, you can use conda to install either a C++ only library (e.g., ``libroboplan-core``, ``libroboplan-simple-ik``) or a library with Python bindings (e.g., ``roboplan-core-python``, ``roboplan-simple-ik-python``).
We also provide convenient metapackages (``libroboplan`` and ``roboplan-python``) containing all the libraries.

---

PyPi (Experimental)
~~~~~~~~~~~~~~~~~~~

**Supported platforms:** Linux (x86_64 and aarch64), macOS (Apple Silicon)

You can also ``pip install roboplan`` to get all the Python bindings as one package.

We recommend creating a Python virtual environment for isolation.

::

    python3 -m venv roboplan
    source roboplan/bin/activate
    pip3 install roboplan

The ``roboplan`` package on PyPi is a pure-Python metapackage that depends on one wheel per package in this repository (``roboplan-core``, ``roboplan-rrt``, etc.).
These wheels are built with ``cibuildwheel`` by an automated CI job that runs on every new tagged version of RoboPlan.
Each package directory is its own `cmeel <https://github.com/cmake-wheel/cmeel>`_ project with its own ``pyproject.toml``;
refer to the `superbuild README <https://github.com/open-planning/roboplan/blob/main/superbuild/README.md>`_ for how to build and test the wheels locally.


---


From Source
-----------

There are currently 3 supported ways to build RoboPlan from source.

Pixi (Recommended)
~~~~~~~~~~~~~~~~~~

**Supported platforms:** Linux, macOS, Windows

Our recommended workflow is to use the `Pixi <https://pixi.sh>`_ package management tool.

First, install Pixi using `these instructions <https://pixi.sh/latest/#installation>`_.

Once set up, you can run the ``pixi`` tasks as follows.

::

    # Build all packages, including Python bindings
    pixi run -e default build

    # Install all packages
    pixi run -e default install

    # This will only build one package (and its dependencies)
    pixi run -e default build PACKAGE_NAME

.. note::

   The ``-e default`` (``--environment default``) flag is required for the ``build``, ``install``, and ``test`` tasks.
   The :ref:`ROS 2 Pixi environments <ros2-with-pixi>` define tasks with the same names, so Pixi refuses to guess which environment you mean.

All packages share a single build tree (``build/``), configured in one shot, so there is no separate
"install one package" operation -- ``pixi run -e default install`` always installs everything.

After building all the packages, you can use the Pixi shell to run specific examples.

::

    pixi shell
    ./build/roboplan_examples/cpp/example_scene
    python3 roboplan_examples/python/example_ik.py


To run the unit tests:

::

    # Test all packages (C++ and Python)
    pixi run -e default test_all

    # Test a specific package (C++ tests only)
    pixi run -e default test PACKAGE_NAME

    # Run only the Python tests
    pixi run test_py

To lint the code:

::

    pixi run lint

Build with AddressSanitizer (ASan)

::

    pixi run -e default build_asan PACKAGE_NAME

Build with compilation time report

::

    pixi run -e default build_timetrace PACKAGE_NAME

``PACKAGE_NAME`` is optional for ``build``/``build_asan``/``build_timetrace``/``test`` above; omit it to build or test
every package.

---


ROS 2 (colcon)
~~~~~~~~~~~~~~

**Supported platforms:** `Supported platforms <https://reps.openrobotics.org/rep-2000/#support-tiers>`_ for your ROS distro.

If you are using `ROS 2 <https://docs.ros.org/>`_, you can build RoboPlan with the ``colcon`` build system.

.. _ros2-with-pixi:

With Pixi
^^^^^^^^^

**Supported platforms:** Linux, macOS, Windows

The Pixi workflow also provides ROS dependencies through `RoboStack <https://robostack.github.io/>`_, with no system install required.
Each supported distro is its own Pixi environment: ``rolling``, ``lyrical``, ``kilted``, ``jazzy``.

Use the following to build and execute with ``colcon``.

::

    # One-time colcon mixin setup
    pixi run -e kilted setup

    # Build and test
    pixi run -e kilted build
    pixi run -e kilted test
    pixi run -e kilted test-result

    # Or run an example from an interactive shell
    pixi shell -e kilted
    source install/setup.bash
    ros2 run roboplan_examples example_scene

Substitute ``kilted`` for whichever distro you want to target.

Directly
^^^^^^^^

For this workflow, you should clone the repo to a valid ROS 2 workspace.

::

    mkdir -p ~/roboplan_ws/src
    cd ~/roboplan_ws/src
    git clone https://github.com/open-planning/roboplan.git

Source your favorite ROS distro and build the workspace.

::

    source /opt/ros/rolling/setup.bash
    cd ~/roboplan_ws
    rosdep install --from-paths src -y --ignore-src
    colcon build

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
    colcon test --packages-select roboplan_core --event-handlers console_direct+

---


Vanilla CMake
~~~~~~~~~~~~~

**Supported platforms:** Linux (macOS and Windows possible but requires effort)

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
