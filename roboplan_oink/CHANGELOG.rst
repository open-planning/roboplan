^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_oink
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
