^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_bindings
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
* CI check for bindings stub files (`#161 <https://github.com/open-planning/roboplan/issues/161>`_)
* Add scene methods to get joint limit vectors (`#162 <https://github.com/open-planning/roboplan/issues/162>`_)
* Fix stub install folder (`#144 <https://github.com/open-planning/roboplan/issues/144>`_)
* Contributors: Cihat Kurtuluş Altıparmak, Sebastian Castro, Sebastian Jahr

0.2.0 (2026-02-16)
------------------
* Use nanobind from pip install for ament workflows (`#138 <https://github.com/open-planning/roboplan/issues/138>`_)
* Add cmake option to generate Python stubs (`#133 <https://github.com/open-planning/roboplan/issues/133>`_)
* Fix bindings (`#137 <https://github.com/open-planning/roboplan/issues/137>`_)
* Expose oink solver regularization as argument (`#136 <https://github.com/open-planning/roboplan/issues/136>`_)
* Simplify FrameTask interface and allow modifying target transforms at runtime (`#131 <https://github.com/open-planning/roboplan/issues/131>`_)
* Separates 6D IK error tolerance into linear (meters) and angular (radians) components (`#128 <https://github.com/open-planning/roboplan/issues/128>`_)
* Support multiple tip frames in simple IK (`#125 <https://github.com/open-planning/roboplan/issues/125>`_)
* Update .pyi files after oink addition (`#127 <https://github.com/open-planning/roboplan/issues/127>`_)
* Fix usage of tinyxml2 and tl_expected dependencies (`#124 <https://github.com/open-planning/roboplan/issues/124>`_)
* Remove lambdas from oink python bindings and use Eigen::Ref (`#119 <https://github.com/open-planning/roboplan/issues/119>`_)
* Aggregated Init SimpleIkOptions and RRTOptions constructor bindings (`#120 <https://github.com/open-planning/roboplan/issues/120>`_)
* Optimal differential IK solver (`#110 <https://github.com/open-planning/roboplan/issues/110>`_)
* Contributors: Erik Holum, Sanjeev, Sebastian Castro, Sebastian Jahr

0.1.0 (2026-01-19)
------------------
* Add argument names and basic docstrings to Python bindings (`#114 <https://github.com/open-planning/roboplan/issues/114>`_)
* CMake and example model path fixes for conda-forge building (`#112 <https://github.com/open-planning/roboplan/issues/112>`_)
* Another hotfix to Viser visualizer
* Patch Viser visualizer to load STL mesh colors properly (`#108 <https://github.com/open-planning/roboplan/issues/108>`_)
* Export bindings utils as an interface library (`#103 <https://github.com/open-planning/roboplan/issues/103>`_)
* Extract visualize_joint_trajectory from example_rrt.py (`#98 <https://github.com/open-planning/roboplan/issues/98>`_)
* Organize examples (`#95 <https://github.com/open-planning/roboplan/issues/95>`_)
* More consts in the Scene member accessors and python tests (`#94 <https://github.com/open-planning/roboplan/issues/94>`_)
* Add initial ReadTheDocs setup (`#90 <https://github.com/open-planning/roboplan/issues/90>`_)
* Fix applyMimics binding by copying and returning a value (`#88 <https://github.com/open-planning/roboplan/issues/88>`_)
* Allow setting collision pairs (`#85 <https://github.com/open-planning/roboplan/issues/85>`_)
* Add collision checking, random restarts, and max time to simple IK solver (`#86 <https://github.com/open-planning/roboplan/issues/86>`_)
* Support collision objects in scene (`#80 <https://github.com/open-planning/roboplan/issues/80>`_)
* Make example models locatable in Rviz (`#82 <https://github.com/open-planning/roboplan/issues/82>`_)
* Support building bindings with colcon (`#76 <https://github.com/open-planning/roboplan/issues/76>`_)
* Support continuous joints in RRT and TOPP-RA (`#73 <https://github.com/open-planning/roboplan/issues/73>`_)
* Ensure the sampled points are actually connectable in path shortcutting (`#71 <https://github.com/open-planning/roboplan/issues/71>`_)
* Support joint groups (`#64 <https://github.com/open-planning/roboplan/issues/64>`_)
* Add Kinova + Robotiq model, initial limited support for continuous and mimic joints (`#59 <https://github.com/open-planning/roboplan/issues/59>`_)
* Add an example model with two fr3s (`#50 <https://github.com/open-planning/roboplan/issues/50>`_)
* Create map of frame names to IDs in Scene (`#58 <https://github.com/open-planning/roboplan/issues/58>`_)
* Update READMEs from setting up new computer
* Organize Python bindings (`#51 <https://github.com/open-planning/roboplan/issues/51>`_)
* Fix IK example (`#49 <https://github.com/open-planning/roboplan/issues/49>`_)
* Specify acceleration and jerk limits through YAML config file (`#45 <https://github.com/open-planning/roboplan/issues/45>`_)
* TOPP-RA path parameterization (`#42 <https://github.com/open-planning/roboplan/issues/42>`_)
* Add `tl::expected` wrapping (`#36 <https://github.com/open-planning/roboplan/issues/36>`_)
* Add path shortening utils and consolidate examples (`#33 <https://github.com/open-planning/roboplan/issues/33>`_)
* Add Franka model and examples (`#34 <https://github.com/open-planning/roboplan/issues/34>`_)
* Hotfix to IK example
* Interactive IK example (`#29 <https://github.com/open-planning/roboplan/issues/29>`_)
* Visualize RRTs with Viser (`#25 <https://github.com/open-planning/roboplan/issues/25>`_)
* Add RRT Connect (`#24 <https://github.com/open-planning/roboplan/issues/24>`_)
* Respect Joint Limits for Start and Goal Poses and Add Planning Timeout for RRT (`#23 <https://github.com/open-planning/roboplan/issues/23>`_)
* First vanilla RRT implementation with dynotree (`#16 <https://github.com/open-planning/roboplan/issues/16>`_)
* Add vanilla CMake workflow (`#12 <https://github.com/open-planning/roboplan/issues/12>`_)
* Test all active ROS distros (`#11 <https://github.com/open-planning/roboplan/issues/11>`_)
* Collision checking functionality (`#10 <https://github.com/open-planning/roboplan/issues/10>`_)
* Generate random positions from scene (`#8 <https://github.com/open-planning/roboplan/issues/8>`_)
* Move models to `roboplan_example_models` package (`#7 <https://github.com/open-planning/roboplan/issues/7>`_)
* Update the example target in the README (`#4 <https://github.com/open-planning/roboplan/issues/4>`_)
* Add basic unit testing pipeline (`#5 <https://github.com/open-planning/roboplan/issues/5>`_)
* Add simple IK solver (`#3 <https://github.com/open-planning/roboplan/issues/3>`_)
* Reorganize packages (`#2 <https://github.com/open-planning/roboplan/issues/2>`_)
* Add basic CI (`#1 <https://github.com/open-planning/roboplan/issues/1>`_)
* More playing around with types and bindings
* Exploration in class binding
* Initial build instructions for ROS 2 + Python bindings
* Actually load a model, put find_package in the config file
* Add dummy Pinocchio function to bind
* Extremely hacky nanobind experiment
* Contributors: Catarina Pires, Erik Holum, Sebastian Castro, Sebastian Jahr
