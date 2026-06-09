ARG ROS_DISTRO=jazzy
ARG GENERATE_PYTHON_STUBS=ON

FROM ros:${ROS_DISTRO} AS roboplan_ros
ARG ROS_DISTRO

ENV PIP_BREAK_SYSTEM_PACKAGES=1
# See https://github.com/tier4/osqp_vendor/issues/26
ENV CMAKE_POLICY_VERSION_MINIMUM=3.5
SHELL ["/bin/bash", "-c"]

# Install core dependencies.
RUN apt update -y \
    && apt upgrade -y \
    && apt install -y python3-pip

# Install Python dependencies needed for bindings.
# Note that since Pinocchio in ROS is built with Numpy 1.x, we must preserve this by
# pinning a bunch of other versions. These pins are ROS specific, and at odds with
# non-ROS workflows, so we cannot put these as requirements in the `pyproject.toml`.
# As/if this package matures, a possible solution is to apply patches for ROS release.
COPY .docker/ros/ros_python_requirements_${ROS_DISTRO}.txt /tmp/ros_python_requirements.txt
RUN pip3 install -r /tmp/ros_python_requirements.txt

# Create a ROS 2 workspace and copy in the source code.
RUN mkdir -p /workspace/roboplan_ws/src/roboplan
WORKDIR /workspace/roboplan_ws
COPY . src/roboplan

# Install dependencies and build RoboPlan.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --cmake-args -DBUILD_TESTING=ON -DGENERATE_PYTHON_STUBS=${GENERATE_PYTHON_STUBS}

# Set up entrypoint and working folder.
WORKDIR /workspace/roboplan_ws
COPY .docker/ros/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
