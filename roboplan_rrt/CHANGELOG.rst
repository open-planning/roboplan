^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_rrt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Pre-0.7.0 docs and consistency passthrough (`#324 <https://github.com/open-planning/roboplan/issues/324>`_)
* Add group helpers, exercise them via new `importSrdf` (`#344 <https://github.com/open-planning/roboplan/issues/344>`_)
* Move file I/O out of Scene constructor, support loading MJCF (`#300 <https://github.com/open-planning/roboplan/issues/300>`_)
* Add ruff check (`#336 <https://github.com/open-planning/roboplan/issues/336>`_)
* Always initialize the state space of the RRT planner (`#339 <https://github.com/open-planning/roboplan/issues/339>`_)
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
* Fix MacOS rpath issues (`#275 <https://github.com/open-planning/roboplan/issues/275>`_)
* Fix Python bindings installs for ROS + Windows (`#272 <https://github.com/open-planning/roboplan/issues/272>`_)
* Windows support through Pixi (`#271 <https://github.com/open-planning/roboplan/issues/271>`_)
* Contributors: Erik Holum, Sebastian Castro

0.5.1 (2026-07-13)
------------------
* Make nanobind-dev and python3-dev build dependencies in package.xml (`#264 <https://github.com/open-planning/roboplan/issues/264>`_)
* Contributors: Sebastian Castro

0.5.0 (2026-07-07)
------------------
* Add unit tests for roboplan_simple_ik package (`#257 <https://github.com/open-planning/roboplan/issues/257>`_)
* Cartesian path planner (`#240 <https://github.com/open-planning/roboplan/issues/240>`_)
* Add clang-tidy (`#182 <https://github.com/open-planning/roboplan/issues/182>`_)
* Store list of link names in group info (`#253 <https://github.com/open-planning/roboplan/issues/253>`_)
* Fix RRT connect tree joining for RRT* (`#252 <https://github.com/open-planning/roboplan/issues/252>`_)
* Add RRT* and fast-return mode (`#246 <https://github.com/open-planning/roboplan/issues/246>`_)
* Speed up collision checking and RRT (`#232 <https://github.com/open-planning/roboplan/issues/232>`_)
* Scene parsing and joint configuration clamping improvements (`#228 <https://github.com/open-planning/roboplan/issues/228>`_)
* Contributors: Erik Holum, Sebastian Castro, Sebastian Jahr

0.4.0 (2026-06-02)
------------------
* Add missing gtest and gmock deps in package.xmls (`#223 <https://github.com/open-planning/roboplan/issues/223>`_)
* Modularize Python bindings (`#221 <https://github.com/open-planning/roboplan/issues/221>`_)
* Use native mimic joint functionality in Pinocchio (`#214 <https://github.com/open-planning/roboplan/issues/214>`_)
* Support planar joints (`#209 <https://github.com/open-planning/roboplan/issues/209>`_)
* Add Stretch4 model (`#208 <https://github.com/open-planning/roboplan/issues/208>`_)
* Contributors: Ola Ghattas, Sebastian Castro

0.3.0 (2026-04-18)
------------------
* Get toppra from binaries or FetchContent (`#174 <https://github.com/open-planning/roboplan/issues/174>`_)
* Add bisection option when checking collisions along path and optimize RRT visualization (`#164 <https://github.com/open-planning/roboplan/issues/164>`_)
* Add viser buttons to RRT example (`#163 <https://github.com/open-planning/roboplan/issues/163>`_)
* Add scene methods to get joint limit vectors (`#162 <https://github.com/open-planning/roboplan/issues/162>`_)
* Contributors: Sebastian Castro

0.2.0 (2026-02-16)
------------------
* Fix usage of tinyxml2 and tl_expected dependencies (`#124 <https://github.com/open-planning/roboplan/issues/124>`_)
* Contributors: Sebastian Castro

0.1.0 (2026-01-19)
------------------
* Add argument names and basic docstrings to Python bindings (`#114 <https://github.com/open-planning/roboplan/issues/114>`_)
* Organize examples (`#95 <https://github.com/open-planning/roboplan/issues/95>`_)
* Add initial ReadTheDocs setup (`#90 <https://github.com/open-planning/roboplan/issues/90>`_)
* Make example models locatable in Rviz (`#82 <https://github.com/open-planning/roboplan/issues/82>`_)
* Support building bindings with colcon (`#76 <https://github.com/open-planning/roboplan/issues/76>`_)
* Support joint groups (`#64 <https://github.com/open-planning/roboplan/issues/64>`_)
* Add Kinova + Robotiq model, initial limited support for continuous and mimic joints (`#59 <https://github.com/open-planning/roboplan/issues/59>`_)
* Reorder ament_cmake include in CMakeLists to resolve test and symlink install issues (`#63 <https://github.com/open-planning/roboplan/issues/63>`_)
* Organize Python bindings (`#51 <https://github.com/open-planning/roboplan/issues/51>`_)
* Add `tl::expected` wrapping (`#36 <https://github.com/open-planning/roboplan/issues/36>`_)
* Visualize RRTs with Viser (`#25 <https://github.com/open-planning/roboplan/issues/25>`_)
* Add RRT Connect (`#24 <https://github.com/open-planning/roboplan/issues/24>`_)
* Respect Joint Limits for Start and Goal Poses and Add Planning Timeout for RRT (`#23 <https://github.com/open-planning/roboplan/issues/23>`_)
* First vanilla RRT implementation with dynotree (`#16 <https://github.com/open-planning/roboplan/issues/16>`_)
* Contributors: Erik Holum, Sebastian Castro
