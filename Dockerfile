FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Cài tiện ích cơ bản và gói ROS cần thiết
RUN apt-get update && apt-get install -y \
    nano \
    x11-apps \   
    ros-jazzy-xacro  \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control   \ 
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-jazzy-ros-gz 
    
   

# Copy workspace và config vào container
COPY config/ /site_config/
COPY src/ /robot-ws/src/

# Tạo user không phải root
ARG USER_NAME=rosjazzy
ARG USER_UID=1001
ARG USER_GID=${USER_UID}

RUN groupadd --gid $USER_GID $USER_NAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USER_NAME \
  && mkdir /home/$USER_NAME/.config \
  && chown $USER_UID:$USER_GID /home/$USER_NAME/.config

# Cấu hình quyền sudo cho user
RUN echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER_NAME \
    && chmod 0440 /etc/sudoers.d/$USER_NAME

# Source ROS 2 Jazzy cho cả user root và user thường
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USER_NAME/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USER_NAME
WORKDIR /robot-ws

ENTRYPOINT [ "bash", "/entrypoint.sh" ]
CMD ["bash"]
