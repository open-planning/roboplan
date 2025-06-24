ARG UBUNTU_VERSION=24.04

FROM ubuntu:${UBUNTU_VERSION} AS roboplan_ubuntu

ENV PIP_BREAK_SYSTEM_PACKAGES=1
SHELL ["/bin/bash", "-c"]

# Install core dependencies.
RUN apt-get update -y \
    && apt install -y \
        cmake curl libeigen3-dev libgmock-dev lsb-release python3-pip
RUN pip3 install nanobind pytest scikit-build-core

# Install Pinocchio binaries.
# https://stack-of-tasks.github.io/pinocchio/download.html
RUN mkdir -p /etc/apt/keyrings && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
     | tee /etc/apt/keyrings/robotpkg.asc > /dev/null && \
     echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
     | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update -y && \
    apt install -qqy robotpkg-py3*-pinocchio
ENV CMAKE_PREFIX_PATH="/opt/openrobots/:${CMAKE_PREFIX_PATH}"
ENV LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH}"

# Create a ROS 2 workspace and copy in the source code.
RUN mkdir -p /roboplan_ws
WORKDIR /roboplan_ws
COPY . /roboplan_ws

# Build RoboPlan from source.
RUN ./scripts/build_cmake.bash

# Install the Python bindings.
ENV CMAKE_PREFIX_PATH="/roboplan_ws/install/:${CMAKE_PREFIX_PATH}"
RUN cd bindings && \
    pip3 install --no-build-isolation -ve .

# Set up entrypoint.
COPY .docker/ubuntu/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
