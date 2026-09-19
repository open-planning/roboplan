^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_cartesian_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2026-09-19)
------------------
* Pre-0.7.0 docs and consistency passthrough (`#324 <https://github.com/open-planning/roboplan/issues/324>`_)
* Move YAML loading out of constructor (`#348 <https://github.com/open-planning/roboplan/issues/348>`_)
* Add group helpers, exercise them via new `importSrdf` (`#344 <https://github.com/open-planning/roboplan/issues/344>`_)
* Move file I/O out of Scene constructor, support loading MJCF (`#300 <https://github.com/open-planning/roboplan/issues/300>`_)
* Add ruff check (`#336 <https://github.com/open-planning/roboplan/issues/336>`_)
* Fix dylib resolution for bindings on macOS with colcon (`#306 <https://github.com/open-planning/roboplan/issues/306>`_)
* Per package PyPi publishing and runtime path setting (`#326 <https://github.com/open-planning/roboplan/issues/326>`_)
* Rename roboplan-core's export target (`#329 <https://github.com/open-planning/roboplan/issues/329>`_)
* Add toFullJointVelocities (`#297 <https://github.com/open-planning/roboplan/issues/297>`_)
* General CMake cleanup (`#319 <https://github.com/open-planning/roboplan/issues/319>`_)
* Use pixi-build to declare Conda dependencies for each package (`#301 <https://github.com/open-planning/roboplan/issues/301>`_)
* Remove cmake build-time package config hack (`#316 <https://github.com/open-planning/roboplan/issues/316>`_)
* Put the Python install logic back into its original CMake location (`#315 <https://github.com/open-planning/roboplan/issues/315>`_)
* Add Pixi ROS targets and CI (`#308 <https://github.com/open-planning/roboplan/issues/308>`_)
* Add `cmake-format` to pre-commit, and run it (`#317 <https://github.com/open-planning/roboplan/issues/317>`_)
* Make cmeel packages independently buildable (`#312 <https://github.com/open-planning/roboplan/issues/312>`_)
* Drop `scikit-build` in favor of direct usage of `cmeel` (`#304 <https://github.com/open-planning/roboplan/issues/304>`_)
* Constrain scope of DLL loader manipulation (`#302 <https://github.com/open-planning/roboplan/issues/302>`_)
* Create CMake superbuild to consolidate build config (`#295 <https://github.com/open-planning/roboplan/issues/295>`_)
* Migrate to SceneContext class for thread-safe operation (`#286 <https://github.com/open-planning/roboplan/issues/286>`_)
* Contributors: Erik Holum, Ezra Brooks, Sebastian Castro, Sebastian Jahr

0.6.1 (2026-08-20)
------------------
* Depend on typing_extensions via rosdep (`#283 <https://github.com/open-planning/roboplan/issues/283>`_)
* Contributors: Sebastian Castro

0.6.0 (2026-07-31)
------------------
* Improve Cartesian path planner (`#277 <https://github.com/open-planning/roboplan/issues/277>`_)
* Fix MacOS rpath issues (`#275 <https://github.com/open-planning/roboplan/issues/275>`_)
* Fix Python bindings installs for ROS + Windows (`#272 <https://github.com/open-planning/roboplan/issues/272>`_)
* Windows support through Pixi (`#271 <https://github.com/open-planning/roboplan/issues/271>`_)
* Switch OInK backend from OSQP to ProxQP (`#259 <https://github.com/open-planning/roboplan/issues/259>`_)
  Co-authored-by: Sebastian Castro <sebas.a.castro@gmail.com>
* Contributors: Erik Holum, Sebastian Castro, Sebastian Jahr

0.5.1 (2026-07-13)
------------------
* Make nanobind-dev and python3-dev build dependencies in package.xml (`#264 <https://github.com/open-planning/roboplan/issues/264>`_)
* Contributors: Sebastian Castro

0.5.0 (2026-07-07)
------------------
* Add pixi Support for osx-arm64 and missing docs pages (`#248 <https://github.com/open-planning/roboplan/issues/248>`_)
* Cartesian path planner (`#240 <https://github.com/open-planning/roboplan/issues/240>`_)
* Contributors: Erik Holum, Sebastian Castro
