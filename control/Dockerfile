# Control Container - Jetson Orin Nano
# Runs VLA model, subscribes to missions, sends subsystem commands

# Use NVIDIA L4T PyTorch base (includes CUDA, cuDNN, PyTorch for Jetson ARM64)
FROM nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

# Install ROS2 Humble (lighter install for control)
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
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install requirements
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Install robotics_sdk from framework repo
ARG ROBOTICS_SDK_INSTALL=git
RUN if [ "$ROBOTICS_SDK_INSTALL" = "git" ]; then \
    pip3 install git+https://github.com/user/robotics.git#subdirectory=common/robotics_sdk || \
    echo "WARNING: robotics_sdk not installed - mount locally for development"; \
    fi

# Copy control source
COPY src/ /app/src/
COPY config/ /app/config/

# Mount point for VLA models
VOLUME ["/data/models"]

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENV PYTHONPATH=/app:$PYTHONPATH
ENV ROS_DOMAIN_ID=0
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility

CMD ["python3", "src/control_main.py"]
