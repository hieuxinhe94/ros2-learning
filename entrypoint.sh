#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash

echo "Provided argument $@"

echo "=========go to root"

exec sudo su

exec source /opt/ros/jazzy/setup.bash

exec cd /robot-ws/src

exec colcon build

exec source install/setup.bash

exec cd /robot-ws/src && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash

# docker image build -t my_ros .

# docker run -it --user ros --env="DISPLAY=$DISPLAY"  --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged  --network=host --ipc=host -v $PWD/source:/source my_ros:latest bash


exec $@



