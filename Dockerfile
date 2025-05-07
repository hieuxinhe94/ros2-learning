FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*

COPY config/ /site_config/

ARG USER_NAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Create a non-root user
RUN groupadd --gid $USER_GID $USER_NAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USER_NAME \
  && mkdir /home/$USER_NAME/.config && chown $USER_UID:$USER_GID /home/$USER_NAME/.config

RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USER_NAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER_NAME\
    && chmod 0440 /etc/sudoers.d/$USER_NAME \ 
    && rm -rf /var/lib/apt/lists/*



