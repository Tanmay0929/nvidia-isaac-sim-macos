# ROS2 Humble Development Environment for Isaac Sim Integration
FROM ros:humble-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    curl \
    vim \
    nano \
    htop \
    tree \
    build-essential \
    cmake \
    pkg-config \
    libeigen3-dev \
    libopencv-dev \
    python3-opencv \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-yaml \
    python3-empy \
    python3-lxml \
    python3-defusedxml \
    python3-netifaces \
    python3-psutil \
    python3-pydot \
    python3-pygraphviz \
    python3-setuptools \
    python3-wheel \
    python3-dev \
    libboost-all-dev \
    libtinyxml2-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libconsole-bridge-dev \
    liblog4cxx-dev \
    liblz4-dev \
    libbz2-dev \
    libzstd-dev \
    libssl-dev \
    libxml2-dev \
    libxslt1-dev \
    libyaml-dev \
    libgtest-dev \
    libgmock-dev \
    libbenchmark-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    scipy \
    matplotlib \
    opencv-python \
    pyyaml \
    empy \
    lxml \
    defusedxml \
    netifaces \
    psutil \
    pydot \
    pygraphviz \
    setuptools \
    wheel \
    pytest \
    pytest-cov \
    flake8 \
    black \
    isort

# Create workspace directory
WORKDIR /ros2_ws

# Initialize rosdep
RUN rosdep init && rosdep update

# Copy workspace files
COPY ros2-workspace/ /ros2_ws/

# Install workspace dependencies
RUN cd /ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN cd /ros2_ws && \
    colcon build --symlink-install

# Source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Create entrypoint script
COPY config/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Expose ports for ROS2 communication
EXPOSE 11311 11312

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
