^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_toppra
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-07-07)
------------------
* Add pixi Support for osx-arm64 and missing docs pages (`#248 <https://github.com/open-planning/roboplan/issues/248>`_)
* Cartesian path planner (`#240 <https://github.com/open-planning/roboplan/issues/240>`_)
* Add clang-tidy (`#182 <https://github.com/open-planning/roboplan/issues/182>`_)
* Speed up collision checking and RRT (`#232 <https://github.com/open-planning/roboplan/issues/232>`_)
* Contributors: Erik Holum, Sebastian Castro, Sebastian Jahr

0.4.0 (2026-06-02)
------------------
* Add missing gtest and gmock deps in package.xmls (`#223 <https://github.com/open-planning/roboplan/issues/223>`_)
* Modularize Python bindings (`#221 <https://github.com/open-planning/roboplan/issues/221>`_)
* Add toppra and nanobind-dev rosdep keys (`#219 <https://github.com/open-planning/roboplan/issues/219>`_)
* Fix TOPPRA issues when using planar joints (`#216 <https://github.com/open-planning/roboplan/issues/216>`_)
* Support Pinocchio 4.0 (`#205 <https://github.com/open-planning/roboplan/issues/205>`_)
* Contributors: Sebastian Castro

0.3.0 (2026-04-18)
------------------
* Get toppra from binaries or FetchContent (`#174 <https://github.com/open-planning/roboplan/issues/174>`_)
* Adaptive TOPP-RA trajectory generation (`#166 <https://github.com/open-planning/roboplan/issues/166>`_)
* Add spline fitting options to TOPP-RA (`#165 <https://github.com/open-planning/roboplan/issues/165>`_)
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
* Support continuous joints in RRT and TOPP-RA (`#73 <https://github.com/open-planning/roboplan/issues/73>`_)
* Support joint groups (`#64 <https://github.com/open-planning/roboplan/issues/64>`_)
* Add Kinova + Robotiq model, initial limited support for continuous and mimic joints (`#59 <https://github.com/open-planning/roboplan/issues/59>`_)
* Reorder ament_cmake include in CMakeLists to resolve test and symlink install issues (`#63 <https://github.com/open-planning/roboplan/issues/63>`_)
* Organize Python bindings (`#51 <https://github.com/open-planning/roboplan/issues/51>`_)
* Specify acceleration and jerk limits through YAML config file (`#45 <https://github.com/open-planning/roboplan/issues/45>`_)
* TOPP-RA path parameterization (`#42 <https://github.com/open-planning/roboplan/issues/42>`_)
* Contributors: Erik Holum, Sebastian Castro
