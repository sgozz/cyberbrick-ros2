# CyberBrick ROS2 Simulation Environment
# Optimized for Mac M1/M2 (ARM64)

FROM arm64v8/ros:humble

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Add Gazebo packages repo (Fortress for ARM64)
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget \
    lsb-release \
    gnupg \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install ROS2 packages (without classic Gazebo - use Ignition/Fortress instead)
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-vision-opencv \
    ros-humble-teleop-twist-keyboard \
    ros-humble-ros-gz \
    ros-humble-ros-ign-bridge \
    python3-pip \
    python3-opencv \
    git \
    vim \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages (without pyaudio - causes issues, will use alternative)
RUN pip3 install \
    paho-mqtt \
    numpy \
    opencv-python

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Setup entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
