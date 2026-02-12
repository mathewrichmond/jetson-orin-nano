# Perception Container - Jetson Orin Nano
# Runs RealSense cameras, nvblox TSDF, and publishes to Zenoh

# Use NVIDIA L4T base (includes CUDA, TensorRT for Jetson ARM64)
FROM nvcr.io/nvidia/l4t-base:r35.2.1

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install RealSense SDK
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    && rm -rf /var/lib/apt/lists/*

# Install Isaac ROS nvblox (or build from source if not available)
# Note: This may require building from source for ARM64
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# TODO: Install/build nvblox for Jetson
# For now, placeholder - will need to build isaac_ros_nvblox from source

WORKDIR /app

# Install robotics framework dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Install robotics_sdk from framework repo
# For development, mount local repo. For production, install from git
ARG ROBOTICS_SDK_INSTALL=git
RUN if [ "$ROBOTICS_SDK_INSTALL" = "git" ]; then \
    pip3 install git+https://github.com/user/robotics.git#subdirectory=common/robotics_sdk || \
    echo "WARNING: robotics_sdk not installed - mount locally for development"; \
    fi

# Copy perception source
COPY src/ /app/src/
COPY config/ /app/config/

# Source ROS2 in bashrc for interactive debugging
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENV PYTHONPATH=/app:$PYTHONPATH
ENV ROS_DOMAIN_ID=0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

CMD ["python3", "src/perception_main.py"]
