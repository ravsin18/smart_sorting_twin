# 1. THE FOUNDATION
# We start with the official ROS 2 "Humble" image.
# "Desktop Full" includes Gazebo (simulator) and RViz (visualizer).
FROM osrf/ros:humble-desktop-full

# 2. NVIDIA GPU SUPPORT
# These lines allow the container to use your laptop's GTX 1650.
# Without this, Gazebo will run very slowly (low FPS).
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:-all}
# Fixes a common graphical glitch in Gazebo/RViz
ENV QT_X11_NO_MITSHM=1

# 3. INSTALL ROBOTICS TOOLS
# We update the package list and install the specific tools for this project.
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros-gz-bridge \
    ros-humble-joint-state-publisher-gui \
    ros-humble-moveit-resources-panda-moveit-config \
    python3-pip \
    mosquitto-clients \
    nano \
    && rm -rf /var/lib/apt/lists/*

# 4. INSTALL PYTHON LIBRARIES
# paho-mqtt: The library we use to send data to the "Digital Twin".
# streamlit: The library for the web dashboard (we use this later).
RUN pip3 install paho-mqtt streamlit

# 5. SETUP WORKSPACE
# We create the folder structure inside the Linux container.
WORKDIR /root/ws_moveit

# 6. COPY SOURCE CODE
# We copy your 'src' folder into the container.
COPY ./src ./src

# 7. AUTOMATIC SOURCING
# This line saves you from typing "source /opt/ros/humble/setup.bash" every time.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc