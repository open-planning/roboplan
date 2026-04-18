^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_simple_ik
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
