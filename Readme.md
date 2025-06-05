## Mục tiêu
> Dựa trên cơ sở:
- Xây dựng robot có khả năng di chuyển tự động, sử dụng các tool gazebo (mô phỏng vật lý) + rviz (mô phỏng dữ liệu).
- Chạy thử trên 2 loại môi trường Linux Ubuntu 24.04  (sau này deploy vào Raspberry PI) và Docker (môi trường ảo hóa giúp làm việc với team/ triển khai nhanh chóng với số lượng lớn).
> Bổ sung:
- Sử dụng laser dò quét địa hình, chướng ngại vật
- Sử dụng AI model dùng để nhớ map và tìm đường đến với địa điểm được yêu cầu.

Toàn bộ khóa học được cung cấp ở đây: https://hieuxinhe94.github.io/general/overview/ 

## Linux Ubuntu 24.04
> Trong ví dụ này tôi đang sử dụng wsl (ảo hóa Ubuntu trên windows) nhưng nếu được bạn cũng có thể cài hẳn Ubuntu lên máy tính lập trình, hoặc cài thẳng Ubuntu vào Raspberry Pi (bộ não robot của bạn). 

> **Lưu ý: Nếu bạn chưa cài ros2 jazzy và gazebo humble, hãy cài đặt theo các bước ở hướng dẫn này https://hieuxinhe94.github.io/blog/2025/04/01/h%C6%B0%E1%BB%9Bng-d%E1%BA%ABn-c%C3%A0i-%C4%91%E1%BA%B7t-ros2/  .** 

Source lại ros2 và truy cập vào thư mục source code. 

    source  /opt/ros/jazzy/setup.bash

>> nano ~/.bashrc
>> source  /opt/ros/jazzy/setup.bash


Truy cập vào thư mục src 

    cd  src
Build lại code (xác nhận lại thư mục hiện tại là src/)

    colcon build --packages-select first_robot
Sau khi build hoàn thành, cần source thư mục code vừa build

    source  install/setup.bash
Cài đặt các gói cần thiết:
    
    sudo apt install ros-jazzy-gz-ros2-control


Bắt đầu chạy code

    ros2 launch first_robot launch_grade2_gazebo.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel

### Docker

> **Lưu ý: Bạn hãy cài docker đúng với OS của bạn** 

Build image từ source code ứng dụng. 

    docker image build -t my_first_robot .
Run container của image vừa build và truy cập vào bin/bash của container đó (tôi đã tích hợp tự động truy cập bash của container sau khi run) 
> Lưu ý quan trọng: $PWD/src:/robot-ws/src là dùng để Bind mount trực tiếp thư mục code ở máy local hiện tại vào container, nên sau này không cần build lại nhiều lần

    docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --privileged --network=host --ipc=host -v $PWD/src:/robot-ws/src my_first_robot:latest bash
Build lại code (xác nhận lại thư mục hiện tại là /robot-ws/src)

    cd  /robot-ws/src && source  /opt/ros/jazzy/setup.bash && colcon  build
Sau khi build hoàn thành, cần source thư mục code vừa build

    source  install/setup.bash

Bắt đầu chạy code

    ros2 launch first_robot launch_grade2_gazebo.launch.py

### Kết quả
![Gazebo demo](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/first_robot_two_wheel_gazebo_rviz.gif?raw=true)

video demo:

![Rviz](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/simple_robot_running_2.gif?raw=true)


### 1. Chi tiết cấu hình
#### **Cấu trúc luồng khớp**:

*   **Chassis** (khung thân robot, định nghĩa trong robot\_core.xacro):
    *   Đây là liên kết gốc (root link) mà tất cả các chân được gắn vào.
*   **Khớp: ${prefix}\_hip\_motor\_mount (fixed)**:
    *   **Loại**: Fixed (khớp cố định, không chuyển động).
    *   **Parent**: chassis.
    *   **Child**: ${prefix}\_hip\_motor (động cơ hông).
    *   **Vị trí**: xyz="${x} ${y} -0.005", với x, y phụ thuộc vào chân (ví dụ: x=0.15, y=0.10 cho left\_front).
    *   **Mục đích**: Gắn động cơ hông vào chassis, không cho phép xoay.
*   **Liên kết: ${prefix}\_hip\_motor**:
    *   Hình trụ nhỏ (bán kính 0.012m, chiều dài 0.02m), đại diện cho động cơ hông.
    *   Khối lượng: 0.03kg.
*   **Khớp: ${prefix}\_hip\_joint (revolute)**:
    *   **Loại**: Revolute (xoay quanh một trục).
    *   **Parent**: ${prefix}\_hip\_motor.
    *   **Child**: ${prefix}\_thigh (đùi).
    *   **Trục xoay**: axis xyz="0 1 0" (xoay quanh trục y, trong mặt phẳng xz).
    *   **Giới hạn góc**: lower="-1.57" upper="0" (-90° đến 0°, cho phép đùi xoay về phía trước).
    *   **Mục đích**: Điều khiển góc đùi so với chassis, hỗ trợ chuyển động chân về phía trước.
*   **Liên kết: ${prefix}\_thigh**:
    *   Hình hộp (0.03 x 0.03 x 0.18m), đại diện cho đoạn đùi.
    *   Khối lượng: 0.15kg.
    *   Tâm khối lượng: xyz="0 0 -0.09" (giữa đùi).
*   **Khớp: ${prefix}\_knee\_motor\_mount (fixed)**:
    *   **Loại**: Fixed.
    *   **Parent**: ${prefix}\_thigh.
    *   **Child**: ${prefix}\_knee\_motor (động cơ đầu gối).
    *   **Vị trí**: xyz="0 0 -0.18" rpy="0 -0.3 0" (đầu gối ở cuối đùi, nghiêng -17.2° để shank hướng về phía trước).
    *   **Mục đích**: Gắn động cơ đầu gối vào đùi.
*   **Liên kết: ${prefix}\_knee\_motor**:
    *   Hình trụ nhỏ (bán kính 0.012m, chiều dài 0.02m).
    *   Khối lượng: 0.03kg.
*   **Khớp: ${prefix}\_knee\_joint (revolute)**:
    *   **Loại**: Revolute.
    *   **Parent**: ${prefix}\_knee\_motor.
    *   **Child**: ${prefix}\_shank (cẳng chân).
    *   **Trục xoay**: axis xyz="0 1 0".
    *   **Giới hạn góc**: lower="-2.5" upper="0" (-143.2° đến 0°, cho phép shank gập về phía trước).
    *   **Mục đích**: Điều khiển góc cẳng chân so với đùi, hỗ trợ gập chân.
*   **Liên kết: ${prefix}\_shank**:
    *   Hình hộp (0.02 x 0.02 x 0.18m), đại diện cho đoạn cẳng chân.
    *   Khối lượng: 0.15kg.
    *   Tâm khối lượng: xyz="0 0 -0.09".
*   **Khớp: ${prefix}\_ankle\_motor\_mount (fixed)**:
    *   **Loại**: Fixed.
    *   **Parent**: ${prefix}\_shank.
    *   **Child**: ${prefix}\_ankle\_motor (động cơ mắt cá).
    *   **Vị trí**: xyz="0 0 -0.18" rpy="0 -0.3 0" (mắt cá ở cuối shank, nghiêng -17.2°).
    *   **Mục đích**: Gắn động cơ mắt cá vào cẳng chân.
*   **Liên kết: ${prefix}\_ankle\_motor**:
    *   Hình trụ nhỏ (bán kính 0.012m, chiều dài 0.02m).
    *   Khối lượng: 0.03kg.
*   **Khớp: ${prefix}\_ankle\_joint (revolute)**:
    *   **Loại**: Revolute.
    *   **Parent**: ${prefix}\_ankle\_motor.
    *   **Child**: ${prefix}\_foot (bàn chân).
    *   **Trục xoay**: axis xyz="0 1 0".
    *   **Giới hạn góc**: lower="-2.0" upper="2.0" (-114.6° đến +114.6°, cho phép điều chỉnh linh hoạt bàn chân).
    *   **Mục đích**: Điều khiển góc bàn chân để tiếp xúc tốt với mặt đất.
*   **Liên kết: ${prefix}\_foot**:
    *   Hình cầu (bán kính 0.03m).
    *   Khối lượng: 0.1kg.
    *   Ma sát: mu1=1.2, mu2=1.2 (cho Gazebo).

#### **Tóm tắt luồng khớp**:

*   **Thứ tự**: chassis → hip\_motor\_mount (fixed) → hip\_motor → hip\_joint (revolute) → thigh → knee\_motor\_mount (fixed) → knee\_motor → knee\_joint (revolute) → shank → ankle\_motor\_mount (fixed) → ankle\_motor → ankle\_joint (revolute) → foot.
*   **Khớp tự do (revolute)**: 3 khớp (hip\_joint, knee\_joint, ankle\_joint), đều xoay quanh trục y, cho phép chuyển động trong mặt phẳng xz.
*   **Đặc điểm**: Chân gập về phía trước (hình ">") nhờ giới hạn góc âm (hip\_joint: -1.57 đến 0, knee\_joint: -2.5 đến 0) và rpy="-0.3" ở khớp knee và ankle.

* * *

## Lỗi 

Update Dependencies:
Ensure all ROS 2 Jazzy packages, Gazebo, and gz_ros2_control are up to date:
bash

Copy
sudo apt update
sudo apt upgrade
chmod +x move/dog_gait_cycle.py
    <!-- need to compose all to single file  run  xacro first_robot/description/robot.urdf.xacro > robot.urdf -->
  <!-- Tâm link nằm giữa  origin là tại tâm hình học  nên Đầu A nằm ở  +L/2 theo trục Z Đầu B nằm ở  -L/2 theo trục Z -->
  Ví dụ: link_parent là hình trụ dài 0.12 dọc trục Z
→ Tâm: tại 0 0 0,
→ Đầu B: tại 0 0 -0.06
