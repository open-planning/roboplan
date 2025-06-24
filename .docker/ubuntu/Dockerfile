ARG UBUNTU_VERSION=24.04

FROM ubuntu:${UBUNTU_VERSION} AS roboplan_ubuntu

ENV PIP_BREAK_SYSTEM_PACKAGES=1
SHELL ["/bin/bash", "-c"]

# Install core dependencies.
RUN apt-get update -y \
    && apt install -y \
        cmake curl libeigen3-dev libgmock-dev lsb-release python3-pip wget

# Use conda rather than binaries to install Pinocchio to support both amd64 and aarch64 cpus.
# https://stack-of-tasks.github.io/pinocchio/download.html
RUN arch=$(uname -m) && \
    if [ "$arch" = "x86_64" ]; \
        then \
            MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh"; \
        elif [ "$arch" = "aarch64" ]; \
        then \
            MINICONDA_URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh"; \
        else \
            echo "Unsupported architecture: $arch"; \
        exit 1; \
    fi && \
    wget $MINICONDA_URL -O miniconda.sh && \
    mkdir -p /root/.conda && \
    bash miniconda.sh -b -p /root/miniconda3 && \
    rm -f miniconda.sh
ENV PATH="/root/miniconda3/bin:${PATH}"

RUN conda init bash
RUN conda install pinocchio=3.7.0 coal eigenpy -c conda-forge

# Install remaining python deps
RUN pip3 install nanobind pytest scikit-build-core

ENV CMAKE_PREFIX_PATH="/root/miniconda3:${CMAKE_PREFIX_PATH}"
ENV LD_LIBRARY_PATH="/root/miniconda3/lib:${LD_LIBRARY_PATH}"

# Create a ROS 2 workspace and copy in the source code.
RUN mkdir -p /roboplan_ws
WORKDIR /roboplan_ws
COPY . /roboplan_ws

# Build RoboPlan from source and add it to the cmake path
RUN ./scripts/build_cmake.bash
ENV CMAKE_PREFIX_PATH="/roboplan_ws/install/:${CMAKE_PREFIX_PATH}"

# Install the Python bindings.
RUN cd bindings && \
    pip3 install --no-build-isolation -ve .

# Add all installed packages' lib dirs to ldconfig
RUN find /roboplan_ws/install -name "lib" -type d | while read libdir;\
    do \
        echo "$libdir" >> /etc/ld.so.conf.d/roboplan.conf; \
    done && \
    ldconfig

# Set up entrypoint.
COPY .docker/ubuntu/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
