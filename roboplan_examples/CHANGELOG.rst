^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-04-18)
------------------
* Incorporate scene and joint groups into OInK (`#177 <https://github.com/open-planning/roboplan/issues/177>`_)
* Add octree support (`#139 <https://github.com/open-planning/roboplan/issues/139>`_)
* Upgrade to Pinocchio 3.9 (`#97 <https://github.com/open-planning/roboplan/issues/97>`_)
* [oink] Control Barrier Functions (`#122 <https://github.com/open-planning/roboplan/issues/122>`_)
* Adaptive TOPP-RA trajectory generation (`#166 <https://github.com/open-planning/roboplan/issues/166>`_)
* Add spline fitting options to TOPP-RA (`#165 <https://github.com/open-planning/roboplan/issues/165>`_)
* Add bisection option when checking collisions along path and optimize RRT visualization (`#164 <https://github.com/open-planning/roboplan/issues/164>`_)
* Add viser buttons to RRT example (`#163 <https://github.com/open-planning/roboplan/issues/163>`_)
* Simplify oink frame task and use model joint limits for velocity constraints (`#143 <https://github.com/open-planning/roboplan/issues/143>`_)
* Contributors: Cihat Kurtuluş Altıparmak, Sebastian Castro, Sebastian Jahr

0.2.0 (2026-02-16)
------------------
* Expose oink solver regularization as argument (`#136 <https://github.com/open-planning/roboplan/issues/136>`_)
* Simplify FrameTask interface and allow modifying target transforms at runtime (`#131 <https://github.com/open-planning/roboplan/issues/131>`_)
* Separates 6D IK error tolerance into linear (meters) and angular (radians) components (`#128 <https://github.com/open-planning/roboplan/issues/128>`_)
* Support multiple tip frames in simple IK (`#125 <https://github.com/open-planning/roboplan/issues/125>`_)
* Remove lambdas from oink python bindings and use Eigen::Ref (`#119 <https://github.com/open-planning/roboplan/issues/119>`_)
* Aggregated Init SimpleIkOptions and RRTOptions constructor bindings (`#120 <https://github.com/open-planning/roboplan/issues/120>`_)
* Optimal differential IK solver (`#110 <https://github.com/open-planning/roboplan/issues/110>`_)
* Contributors: Sanjeev, Sebastian Castro, Sebastian Jahr

0.1.0 (2026-01-19)
------------------
* Add SO-101 arm model (`#106 <https://github.com/open-planning/roboplan/issues/106>`_)
* Extract visualize_joint_trajectory from example_rrt.py (`#98 <https://github.com/open-planning/roboplan/issues/98>`_)
* Organize examples (`#95 <https://github.com/open-planning/roboplan/issues/95>`_)
* Add collision checking, random restarts, and max time to simple IK solver (`#86 <https://github.com/open-planning/roboplan/issues/86>`_)
* Add xacro as a rosdep for the examples (`#78 <https://github.com/open-planning/roboplan/issues/78>`_)
* Support joint groups (`#64 <https://github.com/open-planning/roboplan/issues/64>`_)
* Reorder ament_cmake include in CMakeLists to resolve test and symlink install issues (`#63 <https://github.com/open-planning/roboplan/issues/63>`_)
* Organize Python bindings (`#51 <https://github.com/open-planning/roboplan/issues/51>`_)
* Visualize RRTs with Viser (`#25 <https://github.com/open-planning/roboplan/issues/25>`_)
* First vanilla RRT implementation with dynotree (`#16 <https://github.com/open-planning/roboplan/issues/16>`_)
* Collision checking functionality (`#10 <https://github.com/open-planning/roboplan/issues/10>`_)
* Generate random positions from scene (`#8 <https://github.com/open-planning/roboplan/issues/8>`_)
* Move models to `roboplan_example_models` package (`#7 <https://github.com/open-planning/roboplan/issues/7>`_)
* Update the example target in the README (`#4 <https://github.com/open-planning/roboplan/issues/4>`_)
* Add simple IK solver (`#3 <https://github.com/open-planning/roboplan/issues/3>`_)
* Reorganize packages (`#2 <https://github.com/open-planning/roboplan/issues/2>`_)
* Contributors: Erik Holum, Sebastian Castro, Sebastian Jahr
