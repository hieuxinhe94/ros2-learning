#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash

echo "Provided argument $@"

echo "=========go to root"

exec sudo su

exec source /opt/ros/jazzy/setup.bash

exec cd /robot-ws/src

exec rm -rf  build/ install/ log/ && colcon build

exec source install/setup.bash


exec $@



