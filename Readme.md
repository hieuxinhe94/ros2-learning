### Mục lục
Lớp 1: Robot 2 bánh xe tự di chuyển 
![Design]()
![Gazebo demo](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/first_robot_two_wheel_gazebo_rviz.gif?raw=true)
Source code: branch grade_1_two_wheel_robot_auto_run_gazebo_rviz
Link: 

Lớp 2: Robot 4 bánh xe tự di chuyển + SLAM + NAV2 (camera + laser ) + AI (Mobile SSD)
![Design]()
[Rviz](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/simple_robot_running_2.gif?raw=true)
Source code: branch grade_2_two_wheel_with_AI_robot_laser_map_gazebo_rviz
Link: 

Lớp 3: Robot chó 4 chân với khung cơ bản  + SLAM + NAV2  + CHAMP (camera + laser ) + AI (Mobile SSD)
![Prototype](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/dog_v2_preview_design.gif?raw=true)
![Gazebo](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/dog_v3_preview_level_1.gif?raw=true)
[Rviz](https://github.com/hieuxinhe94/ros2-learning/blob/main/docs/simple_robot_running_2.gif?raw=true)
Source code: branch grade_3_snipdog_with_slam_nav2_3dlaser_gazebo

Link: 

Lớp 4: Robot chó 4 chân: Sản xuất vật lý và ghép nối các thiết bị + SLAM + NAV2  + CHAMP (camera + laser)  + AI (Mobile SSD)
![Design](TODO)
[Rviz](TODO)
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

    ros2 launch first_robot launch_grade3_gazebo.launch.py
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

    ros2 launch first_robot launch_grade3_gazebo.launch.py
    
    
* * *
Robot sử dụng cấu trúc 4 chân (quadruped), mỗi chân gồm 3 khớp chủ động (revolute), tổng cộng 12 bậc tự do (DOF). Thiết kế này tương thích với bộ điều khiển CHAMP và mô phỏng vật lý trong Gazebo.

#### **Bảng thông số các khớp chân robot**

| Tên khớp      | Loại khớp   | Trục quay      | Giới hạn (rad)           | Vị trí gắn | Ghi chú                |
|---------------|-------------|----------------|--------------------------|------------|------------------------|
| Hip           | Revolute    | X (1 0 0)      | -0.686 ~ 0.863           | Thân robot | Khớp háng, quay ngang  |
| Knee          | Revolute    | Y (0 1 0)      | -1.5708 ~ 3.4907         | Đùi        | Khớp gối, quay dọc     |
| Ankle         | Revolute    | Y (0 1 0)      | -2.818 ~ -0.888          | Cẳng chân  | Khớp cổ chân           |
| Foot          | Fixed       | -              | -                        | Bàn chân   | Không chủ động         |

- **Mỗi chân gồm 3 khớp chủ động (hip, knee, ankle) và 1 khớp cố định (foot).**
- **Tổng cộng 12 bậc tự do (DOF) cho 4 chân.**
- **Các giới hạn và trục quay phải đồng bộ giữa URDF/xacro và file cấu hình CHAMP.**

---

3. Thông số hình học và vị trí lắp đặt
- Hip Joint:
Gắn vào trunk tại vị trí xyz riêng cho từng chân (ví dụ: 0.1934 0.0465 0 cho chân trước trái).
Trục quay X (1 0 0).
- Knee Joint:
Gắn vào hip_link, offset theo trục Y (ví dụ: 0 0.0955 0).
Trục quay Y (0 1 0).
- Ankle Joint:
Gắn vào knee_link, offset theo trục Z (ví dụ: 0 0 -0.213).
Trục quay Y (0 1 0).
- Foot Link:
Gắn cố định vào ankle_link, offset theo trục Z.

* * *
 base_link
   └── hip_joint (revolute, trục X)
         └── hip_link
               └── knee_joint (revolute, trục Y)
                     └── knee_link
                           └── ankle_joint (revolute, trục Y)
                                 └── ankle_link
                                       └── foot_link (fixed)

... continue update 
 
## Lỗi 

Update Dependencies:
Ensure all ROS 2 Jazzy packages, Gazebo, and gz_ros2_control are up to date:
bash

Copy
sudo apt update
sudo apt upgrade
<!-- need to compose all to single file  run  xacro first_robot/description/robot.urdf.xacro > robot.urdf -->

