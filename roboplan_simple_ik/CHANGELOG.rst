^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_simple_ik
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Pre-0.7.0 docs and consistency passthrough (`#324 <https://github.com/open-planning/roboplan/issues/324>`_)
* Add group helpers, exercise them via new `importSrdf` (`#344 <https://github.com/open-planning/roboplan/issues/344>`_)
* Move file I/O out of Scene constructor, support loading MJCF (`#300 <https://github.com/open-planning/roboplan/issues/300>`_)
* Add ruff check (`#336 <https://github.com/open-planning/roboplan/issues/336>`_)
* Fix dylib resolution for bindings on macOS with colcon (`#306 <https://github.com/open-planning/roboplan/issues/306>`_)
* Per package PyPi publishing and runtime path setting (`#326 <https://github.com/open-planning/roboplan/issues/326>`_)
* Rename roboplan-core's export target (`#329 <https://github.com/open-planning/roboplan/issues/329>`_)
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
* Contributors: Erik Holum, Ezra Brooks, Sebastian Castro

0.6.1 (2026-08-20)
------------------
* Depend on typing_extensions via rosdep (`#283 <https://github.com/open-planning/roboplan/issues/283>`_)
* Contributors: Sebastian Castro

0.6.0 (2026-07-31)
------------------
* Pose constraints in RRT (`#278 <https://github.com/open-planning/roboplan/issues/278>`_)
* Improve joint limit saturation in SimpleIk (`#276 <https://github.com/open-planning/roboplan/issues/276>`_)
* Fix MacOS rpath issues (`#275 <https://github.com/open-planning/roboplan/issues/275>`_)
* Fix Python bindings installs for ROS + Windows (`#272 <https://github.com/open-planning/roboplan/issues/272>`_)
* Windows support through Pixi (`#271 <https://github.com/open-planning/roboplan/issues/271>`_)
* Contributors: Erik Holum, Sebastian Castro

0.5.1 (2026-07-13)
------------------
* Make nanobind-dev and python3-dev build dependencies in package.xml (`#264 <https://github.com/open-planning/roboplan/issues/264>`_)
* Fix SimpleIk return logic on timeout (`#261 <https://github.com/open-planning/roboplan/issues/261>`_)
* Contributors: Sebastian Castro

0.5.0 (2026-07-07)
------------------
* Add unit tests for roboplan_simple_ik package (`#257 <https://github.com/open-planning/roboplan/issues/257>`_)
* Store list of link names in group info (`#253 <https://github.com/open-planning/roboplan/issues/253>`_)
* Speed up collision checking and RRT (`#232 <https://github.com/open-planning/roboplan/issues/232>`_)
* Fix the example IK python script (`#233 <https://github.com/open-planning/roboplan/issues/233>`_)
* Minor improvements to SimpleIK and forward kinematics (`#231 <https://github.com/open-planning/roboplan/issues/231>`_)
* Contributors: Erik Holum, Sebastian Castro

0.4.0 (2026-06-02)
------------------
* Modularize Python bindings (`#221 <https://github.com/open-planning/roboplan/issues/221>`_)
* Use native mimic joint functionality in Pinocchio (`#214 <https://github.com/open-planning/roboplan/issues/214>`_)
* Support planar joints (`#209 <https://github.com/open-planning/roboplan/issues/209>`_)
* Add Stretch4 model (`#208 <https://github.com/open-planning/roboplan/issues/208>`_)
* Contributors: Ola Ghattas, Sebastian Castro

0.3.0 (2026-04-18)
------------------
* return true only when a valid solution found (`#160 <https://github.com/open-planning/roboplan/issues/160>`_)
* Contributors: Matteo Villani

0.2.0 (2026-02-16)
------------------
* Separates 6D IK error tolerance into linear (meters) and angular (radians) components (`#128 <https://github.com/open-planning/roboplan/issues/128>`_)
* Support multiple tip frames in simple IK (`#125 <https://github.com/open-planning/roboplan/issues/125>`_)
* Contributors: Sanjeev, Sebastian Castro

0.1.0 (2026-01-19)
------------------
* Add argument names and basic docstrings to Python bindings (`#114 <https://github.com/open-planning/roboplan/issues/114>`_)
* Add initial ReadTheDocs setup (`#90 <https://github.com/open-planning/roboplan/issues/90>`_)
* Add collision checking, random restarts, and max time to simple IK solver (`#86 <https://github.com/open-planning/roboplan/issues/86>`_)
* Support joint groups (`#64 <https://github.com/open-planning/roboplan/issues/64>`_)
* Add Kinova + Robotiq model, initial limited support for continuous and mimic joints (`#59 <https://github.com/open-planning/roboplan/issues/59>`_)
* Reorder ament_cmake include in CMakeLists to resolve test and symlink install issues (`#63 <https://github.com/open-planning/roboplan/issues/63>`_)
* Create map of frame names to IDs in Scene (`#58 <https://github.com/open-planning/roboplan/issues/58>`_)
* Fix IK example (`#49 <https://github.com/open-planning/roboplan/issues/49>`_)
* Interactive IK example (`#29 <https://github.com/open-planning/roboplan/issues/29>`_)
* First vanilla RRT implementation with dynotree (`#16 <https://github.com/open-planning/roboplan/issues/16>`_)
* Test all active ROS distros (`#11 <https://github.com/open-planning/roboplan/issues/11>`_)
* Collision checking functionality (`#10 <https://github.com/open-planning/roboplan/issues/10>`_)
* Move models to `roboplan_example_models` package (`#7 <https://github.com/open-planning/roboplan/issues/7>`_)
* Add basic unit testing pipeline (`#5 <https://github.com/open-planning/roboplan/issues/5>`_)
* Add simple IK solver (`#3 <https://github.com/open-planning/roboplan/issues/3>`_)
* Contributors: Catarina Pires, Erik Holum, Sebastian Castro
