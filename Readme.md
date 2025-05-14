<code>
# docker image build -t my_ros .

# docker run -it --user ros --env="DISPLAY=$DISPLAY"  --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged  --network=host --ipc=host -v $PWD/source:/source my_ros:latest bash

exec ros2 launch first_robot rsp.launch.py use_sim_time:=true
exec ros2 launch first_robot launch_sim.launch.py use_sim_time:=true
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run first_robot random_mover.py


exec ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 0.7
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 1.0"

Tiến tới (an toàn)
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "twist:
  linear:
    x: 0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"



🔽 Lùi lại
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "twist:
  linear:
    x: -0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"





 </code>
