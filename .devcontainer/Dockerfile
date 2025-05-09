FROM ros:humble

# Setup locale & update
RUN apt update && apt install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && apt install -y \
    build-essential \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-gazebo-ros \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Create a ROS workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Source ROS setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
