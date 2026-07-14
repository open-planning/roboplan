^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_rrt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
