#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

echo "Provided argument $@"

echo "=========go to root"

exec sudo su

exec source /opt/ros/humble/setup.bash

exec cd /robot-ws/src

exec colcon build

exec source install/setup.bash

exec ros2 launch first_robot rsp.launch.py use_sim_time:=true

exec $@

