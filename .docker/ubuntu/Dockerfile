ARG UBUNTU_VERSION=24.04

FROM ubuntu:${UBUNTU_VERSION} AS roboplan_ubuntu

ENV PIP_BREAK_SYSTEM_PACKAGES=1
SHELL ["/bin/bash", "-c"]

# Install core dependencies.
RUN apt-get update -y \
    && apt install -y \
        cmake libeigen3-dev libgmock-dev liburdfdom-dev lsb-release python3-pip wget
RUN pip3 install nanobind pytest scikit-build-core

# Install pinocchio with pip as it support additional CPU architectures
RUN pip3 install pin

# Set paths for cmeel.prefix installation, this will cause warnings with system package versions
ENV CMAKE_PREFIX_PATH="/usr/local/lib/python3.12/dist-packages/cmeel.prefix:${CMAKE_PREFIX_PATH}"
ENV LD_LIBRARY_PATH="/usr/local/lib/python3.12/dist-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH}"

# Create a ROS 2 workspace and copy in the source code.
RUN mkdir -p /roboplan_ws
WORKDIR /roboplan_ws
COPY . /roboplan_ws

# # Build RoboPlan from source and add it to the cmake path
RUN ./scripts/build_cmake.bash
ENV CMAKE_PREFIX_PATH="/roboplan_ws/install/:${CMAKE_PREFIX_PATH}"

# Install the Python bindings.
RUN cd bindings && \
    pip3 install --no-build-isolation -ve .

# Add all installed packages' lib dirs to ldconfig
RUN for PACKAGE in /roboplan_ws/install/*/lib; do \
        echo ${PACKAGE} >> /etc/ld.so.conf.d/roboplan.conf; \
    done && \
    ldconfig

# Set up entrypoint.
COPY .docker/ubuntu/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
