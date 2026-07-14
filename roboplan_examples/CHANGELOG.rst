^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2026-07-13)
------------------

0.5.0 (2026-07-07)
------------------
* Fix OInK instability with failures to converge (`#256 <https://github.com/open-planning/roboplan/issues/256>`_)
* Cartesian path planner (`#240 <https://github.com/open-planning/roboplan/issues/240>`_)
* Make `ConfigurationTask` runtime tunable and add `AccelerationLimit` (`#250 <https://github.com/open-planning/roboplan/issues/250>`_)
* Add broadphase culling to collision checking and self-collision barrier (`#254 <https://github.com/open-planning/roboplan/issues/254>`_)
* Add RRT* and fast-return mode (`#246 <https://github.com/open-planning/roboplan/issues/246>`_)
* add base frame support to Jacobian (`#238 <https://github.com/open-planning/roboplan/issues/238>`_)
* Speed up path shortcutting (`#236 <https://github.com/open-planning/roboplan/issues/236>`_)
* Fix the example IK python script (`#233 <https://github.com/open-planning/roboplan/issues/233>`_)
* Add Reachback mobile manipulator model (`#227 <https://github.com/open-planning/roboplan/issues/227>`_)
* Contributors: Erik Holum, Muslim Alaran, Sebastian Castro, Stephanie Eng

0.4.0 (2026-06-02)
------------------
* Modularize Python bindings (`#221 <https://github.com/open-planning/roboplan/issues/221>`_)
* OInK self-collision barrier (`#207 <https://github.com/open-planning/roboplan/issues/207>`_)
* Support priorities and nullspace projection in OInK tasks (`#206 <https://github.com/open-planning/roboplan/issues/206>`_)
* Keyboard teleoperation example (`#213 <https://github.com/open-planning/roboplan/issues/213>`_)
* Fix TOPPRA issues when using planar joints (`#216 <https://github.com/open-planning/roboplan/issues/216>`_)
* Use native mimic joint functionality in Pinocchio (`#214 <https://github.com/open-planning/roboplan/issues/214>`_)
* Support planar joints (`#209 <https://github.com/open-planning/roboplan/issues/209>`_)
* Add Stretch4 model (`#208 <https://github.com/open-planning/roboplan/issues/208>`_)
* Lazy import plyfile to not require it in all examples (`#194 <https://github.com/open-planning/roboplan/issues/194>`_)
* Support Pinocchio 4.0 (`#205 <https://github.com/open-planning/roboplan/issues/205>`_)
* Support dual end effectors in action chunk example (`#204 <https://github.com/open-planning/roboplan/issues/204>`_)
* Support cylinder and mesh scene collision objects (`#197 <https://github.com/open-planning/roboplan/issues/197>`_)
* Add CartesianTrajectory type (`#203 <https://github.com/open-planning/roboplan/issues/203>`_)
* Add OInK action chunk tracking example (`#196 <https://github.com/open-planning/roboplan/issues/196>`_)
* Add SE3 low-pass filter C++ library and Python bindings (`#180 <https://github.com/open-planning/roboplan/issues/180>`_)
* Contributors: Muslim Alaran, Ola Ghattas, Prajwal, Sebastian Castro

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
