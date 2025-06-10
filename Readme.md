### Mục lục
Lớp 1: Robot 2 bánh xe tự di chuyển 
![Design]()
![Gazebo demo](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/first_robot_two_wheel_gazebo_rviz.gif?raw=true)
Link: 

Lớp 2: Robot 4 bánh xe tự di chuyển + SLAM + NAV2 (camera + laser ) + AI (Mobile SSD)
![Design]()
[Rviz](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/simple_robot_running_2.gif?raw=true)
Link: 

Lớp 3: Robot chó 4 chân với khung cơ bản  + SLAM + NAV2  + CHAMP (camera + laser ) + AI (Mobile SSD)
![Design](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/dog_v2_preview_design.gif?raw=true)
[Rviz](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/simple_robot_running_2.gif?raw=true)
Link: 

Lớp 4: Robot chó 4 chân: Sản xuất vật lý và ghép nối các thiết bị + SLAM + NAV2  + CHAMP (camera + laser)  + AI (Mobile SSD)
![Design]()
[Rviz]()
Link: 


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
 
 
Dưới đây là bản mô tả tổng hợp 4 chân robot chó, ngắn gọn và súc tích:
![Preview-Design](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/dog_v2_preview_design.gif?raw=true)




* * *

### 1\. Chi tiết cấu hình

#### **Cấu trúc luồng khớp (áp dụng cho cả 4 chân: FL, FR, RL, RR)**

*   **Chassis** (`base_link`): liên kết gốc, gắn 4 chân.
    
*   **Khớp: `${prefix}_hip_joint` (revolute)**
    
    *   Parent: `base_link`, Child: `${prefix}_hip_link`
        
    *   Trục: z (0 0 1), Giới hạn: ±0.785 rad
        
    *   Vị trí: xác định bởi `x_pos`, `y_pos` theo từng chân
        
*   **Khớp: `${prefix}_upper_leg_joint` (revolute)**
    
    *   Parent: `${prefix}_hip_link`, Child: `${prefix}_upper_leg_link`
        
    *   Trục: y (0 1 0), Giới hạn: -1.57 → 0 rad
        
*   **Khớp: `${prefix}_lower_leg_joint` (revolute)**
    
    *   Parent: `${prefix}_upper_leg_link`, Child: `${prefix}_lower_leg_link`
        
    *   Trục: y (0 1 0), Giới hạn: -2.5 → 0 rad, RPY="-0.3 0 0"
        
*   **Liên kết foot** (nếu có): Gắn cuối chân dưới, không có khớp riêng.
    

* * *

#### **Tóm tắt luồng khớp**

scss

CopyEdit

`base_link  → hip_joint (z)    → hip_link      → upper_leg_joint (y)        → upper_leg_link          → lower_leg_joint (y, rpy -0.3)            → lower_leg_link              → foot`

*   Tổng: 3 khớp chủ động (revolute) mỗi chân × 4 chân = **12 DOF**
    
*   Tất cả các khớp đều hoạt động trong mặt phẳng xz để tạo dáng di chuyển kiểu ">"
    
*   Thiết kế modular, cấu trúc giống nhau, chỉ khác `prefix`, `x/y position`.
    

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
