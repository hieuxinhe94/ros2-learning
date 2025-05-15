## Mục tiêu
- Xây dựng robot có khả năng di chuyển tự động, sử dụng các tool gazebo (mô phỏng vật lý) + rviz (mô phỏng dữ liệu).
- Chạy thử trên 2 loại môi trường Linux Ubuntu 24.04  (sau này deploy vào Rapper PI) và Docker (môi trường ảo hóa giúp làm việc với team/ triển khai nhanh chóng với số lượng lớn).


## Linux Ubuntu 24.04
> Trong ví dụ này tôi đang sử dụng wsl (ảo hóa Ubuntu trên windows) nhưng nếu được bạn cũng có thể cài hẳn Ubuntu lên máy tính lập trình, hoặc cài thẳng Ubuntu vào Raspberry Pi (bộ não robot của bạn). 

> **Lưu ý: Nếu bạn chưa cài ros2 jazzy và gazebo humble, hãy cài đặt theo các bước ở hướng dẫn này....** 

Source lại ros2 và truy cập vào thư mục source code. 

    source  /opt/ros/jazzy/setup.bash
Truy cập vào thư mục src 

    cd  src
Build lại code (xác nhận lại thư mục hiện tại là src/)

    colcon build --packages-select first_robot
Sau khi build hoàn thành, cần source thư mục code vừa build

    source  install/setup.bash

Bắt đầu chạy code

    ros2 launch first_robot launch_sim_gazebo.launch.py


### Docker
  
> Trong ví dụ này tôi đang sử dụng wsl (ảo hóa Ubuntu trên windows) nhưng nếu được bạn cũng có thể cài hẳn Ubuntu lên máy tính lập trình, hoặc cài thẳng Ubuntu vào Raspberry Pi (bộ não robot của bạn). 

> **Lưu ý: Bạn hãy cài docker** 

Source lại ros2 và truy cập vào thư mục source code. 

    docker image build -t my_first_robot .
Run container của image vừa build và truy cập vào bin/bash của container đó (tôi đã tích hợp tự động truy cập bash của container sau khi run)

    docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged --network=host --ipc=host -v $PWD/source:/source my_first_robot:latest bash
Build lại code (xác nhận lại thư mục hiện tại là /robot-ws/src)

    cd  /robot-ws/src && source  /opt/ros/jazzy/setup.bash && colcon  build
Sau khi build hoàn thành, cần source thư mục code vừa build

    source  install/setup.bash

Bắt đầu chạy code

    ros2 launch first_robot launch_sim_gazebo.launch.py

### Kết quả
![Gazebo demo](https://github.com/hieuxinhe94/ros2-learning/blob/main/first_robot_two_wheel_gazebo_rviz.gif?raw=true)

video demo:

![Rviz](https://github.com/hieuxinhe94/ros2-learning/blob/main/first_robot_two_wheel_gazebo_rviz.gif?raw=true)
