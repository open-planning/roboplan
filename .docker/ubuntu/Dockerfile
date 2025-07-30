ARG UBUNTU_VERSION=24.04

FROM ubuntu:${UBUNTU_VERSION} AS roboplan_ubuntu

ARG PINOCCHIO_VERSION=3.7.0
ENV PIP_BREAK_SYSTEM_PACKAGES=1
SHELL ["/bin/bash", "-c"]

# Install core dependencies.
RUN apt-get update -y \
    && apt install -y \
        cmake libeigen3-dev libgmock-dev liburdfdom-dev python3-pip
RUN pip3 install nanobind pytest scikit-build-core tyro

# Install pinocchio with pip as it supports additional CPU architectures.
# Also install additional dependencies for examples.
RUN pip3 install pin==${PINOCCHIO_VERSION} viser opencv-contrib-python-headless

# Find the location of the relevant pip installed packages and ensure they are available to
# cmake and LD. This will cause build warnings with system installed packages.
RUN PIP_PATH="$(pip3 show pin | grep "Location:" | cut -d' ' -f2)/cmeel.prefix" && \
    echo "export LD_LIBRARY_PATH=${PIP_PATH}/lib:\${LD_LIBRARY_PATH}" >> /roboplan_env.sh \
    echo "export CMAKE_PREFIX_PATH=${PIP_PATH}:\${CMAKE_PREFIX_PATH}" >> /roboplan_env.sh

# Create a ROS 2 workspace and copy in the source code.
RUN mkdir -p /roboplan_ws
WORKDIR /roboplan_ws
COPY . /roboplan_ws

# Build RoboPlan from source and add it to the cmake path
RUN . /roboplan_env.sh && ./scripts/build_cmake.bash
RUN echo "export CMAKE_PREFIX_PATH=/roboplan_ws/install/:\${CMAKE_PREFIX_PATH}" >> /roboplan_env.sh

# Install the Python bindings.
RUN . /roboplan_env.sh && \
    cd bindings && \
    pip3 install --no-build-isolation -ve .

RUN for PACKAGE in /roboplan_ws/install/*/lib; do \
        echo "export LD_LIBRARY_PATH=${PACKAGE}:\${LD_LIBRARY_PATH}" >> /roboplan_env.sh; \
    done

# Set up entrypoint.
COPY .docker/ubuntu/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
