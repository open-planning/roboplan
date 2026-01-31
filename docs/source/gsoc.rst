Google Summer of Code
=====================

We are happy to participate in the `Google Summer of Code <https://summerofcode.withgoogle.com/>`_ (GSoC) program.

This is a great way to get hands-on experience with open-source software.
If you are interested, consider applying as a GSoC contributor to the Open Planning organization!

Below are some project ideas for GSoC.
You should feel free to propose projects directly under the following ideas, but you are also welcome to bring your own proposal!

For help putting together a proposal, refer to `this useful guide <https://google.github.io/gsocguides/student/writing-a-proposal>`_.

---

2026 Project Ideas
------------------

Releasing RoboPlan to PyPi
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Mentors:** Sebastian Castro

**Project size:** Medium (175 hours)

**Description:**

Recently, RoboPlan was added to `conda-forge <https://conda-forge.org/>`_ to make installation easy.
Right now this only supports Linux, so a contributor could also consider expanding support to other platforms as part of this effort.

However, a next step for ease of use is to be able to ``pip install roboplan``.

With libraries like RoboPlan, however, which rely on several upstream libraries, distribution to PyPi can be challenging.
Wheels have to be assembled using tools such as `cibuildwheel <https://cibuildwheel.pypa.io/en/stable/>`_, for which there are several reference open-source projects.

This project involves exploring the feasibility of building wheels from the RoboPlan stack in CI, testing on various platforms, and automating/documenting the release generation process.


ROS Interface and Examples
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Mentors:** Erik Holum, Sebastian Castro, Sebastian Jahr

**Project size:** Large (350 hours)

**Description:**

While RoboPlan itself is a middleware-free core library by design, integrating with common robotics ecosystems is desirable.

As such, we also maintain an experimental `roboplan-ros <https://github.com/open-planning/roboplan-ros>`_ repository,
which contains a RoboPlan ROS 2 wrapper.

This project involves directly working on the ``roboplan-ros`` repository, helping expand the capabilities and better integrate into the ROS ecosystem.
Some representative tasks in this project are:

* Helping finalize the ROS wrappers to RoboPlan for motion planning, inverse kinematics, modifying scene obstacles, etc.
* Creating ROS 2 controllers that utilize inverse kinematics, online planning, and/or collision checking capabilities in RoboPlan.
* Putting together more complete examples that use simulators such as MuJoCo/Gazebo with ``ros2_control`` and demonstrate full motion planning pipelines (`here <https://github.com/NASA-JSC-Robotics/clr_ws>_` is a motivating example).
* Work on releasing the ROS wrappers to the ROS Buildfarm, so the packages can be installed via binaries.

You are welcome to propose a subset of the above ideas for this project, or even come up with new ideas!
The ROS ecosystem has a large space to explore, and many types of contributions are welcome.


Robot Learning Application Examples
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Mentors:** Sebastian Castro, Jafar Uruç, Sebastian Jahr

**Project size:** Large (350 hours)

**Description:**

Many open-source tools like `LeRobot <https://github.com/huggingface/lerobot>`_ and `Neuracore <https://github.com/NeuracoreAI/neuracore>`_
are emerging as ways to make robot learning (particularly imitation learning) workflows accessible to newcomers and experts alike.

In RoboPlan, we believe it is important to build a motion planning tool that can help with more modern robot learning pipelines,
rather than only traditional "sense-plan-act" pipelines built for industrial manipulators that must avoid contact.

Projects in this area would attempt to integrate RoboPlan with one of these other imitation learning tools.
RoboPlan could be beneficial in several ways:

* Providing a way to teleoperate a (simulated) robot using the keyboard, devices such as gamepads or 3D mice, or leader-follower systems.
* Demonstrating data collection workflows by integrating with tools such as LeRobot to log training data.
* Using RoboPlan's motion planning capabilities to smoothly track the output of trained policies (VLAs, Diffusion Policy, Action Chunking Transformers (ACT), etc.)

A successful project would involve working with the mentors to agree on an application example to build up to.
If you need to purchase any hardware as part of your proposal, please include a proposed bill of materials and we may consider sponsoring that as feasible.
However, note that these efforts should target simulation, or low-cost hobbyist platforms if there is a good motivation.


Expanding Available Algorithms in RoboPlan
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Mentors:** Sebastian Jahr, Sebastian Castro, Erik Holum

**Project size:** Medium (175 hours) - Large (350 hours)

**Description:**

Currently, RoboPlan offers basic algorithms for collision-free motion planning (RRT),
inverse kinematics (simple IK and incoming optimization-based IK), and trajectory timing (TOPP-RA).

There are many other algorithms in the space of motion planning that could be implemented here.
If you have any ideas for new algorithms to implement or integrate into RoboPlan, and you find this exciting,
feel free to propose your own.

The mentors are happy to help you integrate these algorithms into RoboPlan.
Expected tasks will involve fitting your code into the design of the tool, creating Python bindings, testing,
documentation, and examples for you and the users to gain confidence that this algorithm works and is useful.

This is a great opportunity for contributors with strong mathematical/algorithmic/domain knowledge to try out a
more "professional" direction of software development that you may encounter in contributing to other open-source projects.
