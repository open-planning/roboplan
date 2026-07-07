^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_oink
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-07-07)
------------------
* Fix OInK instability with failures to converge (`#256 <https://github.com/open-planning/roboplan/issues/256>`_)
* Cartesian path planner (`#240 <https://github.com/open-planning/roboplan/issues/240>`_)
* Make `ConfigurationTask` runtime tunable and add `AccelerationLimit` (`#250 <https://github.com/open-planning/roboplan/issues/250>`_)
* Add broadphase culling to collision checking and self-collision barrier (`#254 <https://github.com/open-planning/roboplan/issues/254>`_)
* Add clang-tidy (`#182 <https://github.com/open-planning/roboplan/issues/182>`_)
* add base frame support to Jacobian (`#238 <https://github.com/open-planning/roboplan/issues/238>`_)
* Contributors: Muslim Alaran, Sebastian Castro, Sebastian Jahr

0.4.0 (2026-06-02)
------------------
* Fix failing OInK test due to dimension mismatch (`#225 <https://github.com/open-planning/roboplan/issues/225>`_)
* Add Ubuntu 26.04 and ROS 2 Lyrical to CI (`#215 <https://github.com/open-planning/roboplan/issues/215>`_)
* Add missing gtest and gmock deps in package.xmls (`#223 <https://github.com/open-planning/roboplan/issues/223>`_)
* Modularize Python bindings (`#221 <https://github.com/open-planning/roboplan/issues/221>`_)
* OInK self-collision barrier (`#207 <https://github.com/open-planning/roboplan/issues/207>`_)
* Support priorities and nullspace projection in OInK tasks (`#206 <https://github.com/open-planning/roboplan/issues/206>`_)
* Support planar joints (`#209 <https://github.com/open-planning/roboplan/issues/209>`_)
* Contributors: Ola Ghattas, Sebastian Castro

0.3.0 (2026-04-18)
------------------
* Incorporate scene and joint groups into OInK (`#177 <https://github.com/open-planning/roboplan/issues/177>`_)
* Switch to OSQP v0.6.3 for OInK (`#176 <https://github.com/open-planning/roboplan/issues/176>`_)
* Get toppra from binaries or FetchContent (`#174 <https://github.com/open-planning/roboplan/issues/174>`_)
* [oink] Control Barrier Functions (`#122 <https://github.com/open-planning/roboplan/issues/122>`_)
* Add scene methods to get joint limit vectors (`#162 <https://github.com/open-planning/roboplan/issues/162>`_)
* Fix position limits oink constraint for models with continuous joints (`#147 <https://github.com/open-planning/roboplan/issues/147>`_)
* Ensure oink tests run in all workflows (`#145 <https://github.com/open-planning/roboplan/issues/145>`_)
* Simplify oink frame task and use model joint limits for velocity constraints (`#143 <https://github.com/open-planning/roboplan/issues/143>`_)
* Fix roboplan_oink OSQP dependency resolution (`#141 <https://github.com/open-planning/roboplan/issues/141>`_)
* Contributors: Sebastian Castro, Sebastian Jahr, Zhengyang Kris Weng

0.2.0 (2026-02-16)
------------------
* Expose oink solver regularization as argument (`#136 <https://github.com/open-planning/roboplan/issues/136>`_)
* Simplify FrameTask interface and allow modifying target transforms at runtime (`#131 <https://github.com/open-planning/roboplan/issues/131>`_)
* Remove lambdas from oink python bindings and use Eigen::Ref (`#119 <https://github.com/open-planning/roboplan/issues/119>`_)
* Optimal differential IK solver (`#110 <https://github.com/open-planning/roboplan/issues/110>`_)
* Contributors: Sebastian Castro, Sebastian Jahr
