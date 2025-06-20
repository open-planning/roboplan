ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO} AS roboplan_ros

ENV PIP_BREAK_SYSTEM_PACKAGES=1
SHELL ["/bin/bash", "-c"]

# Install core dependencies.
RUN apt-get update -y \
    && apt install -y libgmock-dev python3-pip
RUN pip3 install nanobind scikit-build-core

# Create a ROS 2 workspace and copy in the source code.
RUN mkdir -p /workspace/roboplan_ws/src/roboplan
WORKDIR /workspace/roboplan_ws
COPY . src/roboplan

# Install dependencies and build RoboPlan.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --cmake-args -DBUILD_TESTING=ON

# Install the Python bindings.
WORKDIR /workspace/roboplan_ws/src/roboplan/bindings
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source /workspace/roboplan_ws/install/setup.bash \
    && pip3 install --no-build-isolation -ve .

# Set up entrypoint and working folder.
WORKDIR /workspace/roboplan_ws
COPY .docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
