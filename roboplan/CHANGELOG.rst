^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-04-18)
------------------
* Incorporate scene and joint groups into OInK (`#177 <https://github.com/open-planning/roboplan/issues/177>`_)
* Add octree support (`#139 <https://github.com/open-planning/roboplan/issues/139>`_)
* Add console_bridge to cmake (`#172 <https://github.com/open-planning/roboplan/issues/172>`_)
* Add bisection option when checking collisions along path and optimize RRT visualization (`#164 <https://github.com/open-planning/roboplan/issues/164>`_)
* Add scene methods to get joint limit vectors (`#162 <https://github.com/open-planning/roboplan/issues/162>`_)
* Validate subgroup parsing order issues in SRDF (`#157 <https://github.com/open-planning/roboplan/issues/157>`_)
* Fix position limits oink constraint for models with continuous joints (`#147 <https://github.com/open-planning/roboplan/issues/147>`_)
* Contributors: Cihat Kurtuluş Altıparmak, Rafael A. Rojas, Sebastian Castro

0.2.0 (2026-02-16)
------------------
* Fix usage of tinyxml2 and tl_expected dependencies (`#124 <https://github.com/open-planning/roboplan/issues/124>`_)
* Optimal differential IK solver (`#110 <https://github.com/open-planning/roboplan/issues/110>`_)
* Contributors: Sebastian Castro, Sebastian Jahr

0.1.0 (2026-01-19)
------------------
* Add argument names and basic docstrings to Python bindings (`#114 <https://github.com/open-planning/roboplan/issues/114>`_)
* Improve error reporting in scene_utils map access (`#113 <https://github.com/open-planning/roboplan/issues/113>`_)
* Organize examples (`#95 <https://github.com/open-planning/roboplan/issues/95>`_)
* More consts in the Scene member accessors and python tests (`#94 <https://github.com/open-planning/roboplan/issues/94>`_)
* Add initial ReadTheDocs setup (`#90 <https://github.com/open-planning/roboplan/issues/90>`_)
* Fix applyMimics binding by copying and returning a value (`#88 <https://github.com/open-planning/roboplan/issues/88>`_)
* Allow setting collision pairs (`#85 <https://github.com/open-planning/roboplan/issues/85>`_)
* Support collision objects in scene (`#80 <https://github.com/open-planning/roboplan/issues/80>`_)
* Make example models locatable in Rviz (`#82 <https://github.com/open-planning/roboplan/issues/82>`_)
* Add basic unit test for continuous and mimic joints (`#75 <https://github.com/open-planning/roboplan/issues/75>`_)
* Support continuous joints in RRT and TOPP-RA (`#73 <https://github.com/open-planning/roboplan/issues/73>`_)
* Ensure the sampled points are actually connectable in path shortcutting (`#71 <https://github.com/open-planning/roboplan/issues/71>`_)
* Support joint groups (`#64 <https://github.com/open-planning/roboplan/issues/64>`_)
* Add Kinova + Robotiq model, initial limited support for continuous and mimic joints (`#59 <https://github.com/open-planning/roboplan/issues/59>`_)
* Reorder ament_cmake include in CMakeLists to resolve test and symlink install issues (`#63 <https://github.com/open-planning/roboplan/issues/63>`_)
* Add an example model with two fr3s (`#50 <https://github.com/open-planning/roboplan/issues/50>`_)
* Fix forward kinematics calc (`#61 <https://github.com/open-planning/roboplan/issues/61>`_)
* Create map of frame names to IDs in Scene (`#58 <https://github.com/open-planning/roboplan/issues/58>`_)
* Organize Python bindings (`#51 <https://github.com/open-planning/roboplan/issues/51>`_)
* Fix path shortcutting logic (`#46 <https://github.com/open-planning/roboplan/issues/46>`_)
* Specify acceleration and jerk limits through YAML config file (`#45 <https://github.com/open-planning/roboplan/issues/45>`_)
* TOPP-RA path parameterization (`#42 <https://github.com/open-planning/roboplan/issues/42>`_)
* Add `tl::expected` wrapping (`#36 <https://github.com/open-planning/roboplan/issues/36>`_)
* Add path shortening utils and consolidate examples (`#33 <https://github.com/open-planning/roboplan/issues/33>`_)
* Visualize RRTs with Viser (`#25 <https://github.com/open-planning/roboplan/issues/25>`_)
* Respect Joint Limits for Start and Goal Poses and Add Planning Timeout for RRT (`#23 <https://github.com/open-planning/roboplan/issues/23>`_)
* First vanilla RRT implementation with dynotree (`#16 <https://github.com/open-planning/roboplan/issues/16>`_)
* Add vanilla CMake workflow (`#12 <https://github.com/open-planning/roboplan/issues/12>`_)
* Test all active ROS distros (`#11 <https://github.com/open-planning/roboplan/issues/11>`_)
* Collision checking functionality (`#10 <https://github.com/open-planning/roboplan/issues/10>`_)
* Generate random positions from scene (`#8 <https://github.com/open-planning/roboplan/issues/8>`_)
* Move models to `roboplan_example_models` package (`#7 <https://github.com/open-planning/roboplan/issues/7>`_)
* Add basic unit testing pipeline (`#5 <https://github.com/open-planning/roboplan/issues/5>`_)
* Add simple IK solver (`#3 <https://github.com/open-planning/roboplan/issues/3>`_)
* Reorganize packages (`#2 <https://github.com/open-planning/roboplan/issues/2>`_)
* Contributors: Catarina Pires, Erik Holum, Ola Ghattas, Sebastian Castro
