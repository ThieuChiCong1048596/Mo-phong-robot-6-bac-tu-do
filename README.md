🤖 Mô phỏng Robot 6 Bậc Tự Do (6-DOF)

🚀 Tổng Quan
Dự án cung cấp một môi trường mô phỏng robot công nghiệp 6 bậc tự do (6-DOF) sử dụng ROS Noetic, MoveIt, Gazebo, và giao diện điều khiển (GUI) phát triển bằng MATLAB. Hệ thống tích hợp mô hình URDF từ SolidWorks, cho phép:

* Lập kế hoạch chuyển động (trajectory planning).
* Điều khiển robot (động học thuận/nghịch).
* Tương tác môi trường (phát hiện và thao tác đối tượng).

**Ý tưởng chính:** Xây dựng hệ thống mô phỏng robot AR2 hoàn chỉnh, kết nối ROS ↔ MATLAB, xử lý hình ảnh từ cảm biến, và điều khiển linh hoạt qua GUI hoặc script Python.
Phù hợp cho nghiên cứu, phát triển robot, và học tập.

🎬 Demo
📹 Xem video mô phỏng robot 6-DOF trong Gazebo, MoveIt, và GUI MATLAB:
\[Link YouTube hoặc Google Drive: [https://youtube.com/](https://youtube.com/)...]

✨ Tính Năng

* 🤖 Mô phỏng robot AR2: Tích hợp URDF và Gazebo cho chuyển động khớp chính xác.
* 🛠️ Lập kế hoạch chuyển động: Sử dụng MoveIt với script Python (IK\_solver.py, Cartesian\_path.py).
* 🎮 GUI MATLAB: Điều khiển và giám sát robot qua giao diện đồ họa (Robot\_6DOF\_controller\_GUI).
* 🔍 Phát hiện đối tượng: Xử lý hình ảnh để nhận diện vật thể trong Gazebo (Detectobject.py).
* 🧩 Tương tác môi trường: Thêm vật thể động (node\_spawn\_box\_models\_in\_gazebo.py).
* 📍 Điều khiển đầu cuối: Theo dõi và đặt tư thế định sẵn (EE\_tracker.py, node\_set\_predefined\_pose.py).
* 📐 Thiết kế 3D: Mô hình SolidWorks chi tiết (Solidwork 6DOF Assembly file.rar).

📦 Yêu Cầu Hệ Thống

| Yêu cầu          | Chi tiết                                                      |
| ---------------- | ------------------------------------------------------------- |
| Hệ điều hành     | Ubuntu 20.04 (Focal Fossa)                                    |
| ROS              | Noetic Ninjemys                                               |
| MoveIt           | 1.1.9                                                         |
| Gazebo           | 11.x                                                          |
| MATLAB           | R2024a+ với ROS Toolbox                                       |
| Phần mềm bổ sung | Git, Python 3.8+, Catkin tools                                |
| Phần cứng        | RAM 8GB+ (khuyến nghị 16GB), CPU quad-core, GPU hỗ trợ OpenGL |

⚙️ Cài Đặt

1. Cài ROS Noetic

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

👉 Chi tiết: Hướng dẫn chính thức ROS

2. Cài MoveIt

```bash
sudo apt install ros-noetic-moveit
```

3. Tạo ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

4. Clone Repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/ThieuChiCong1048596/Mo-phong-robot-6-bac-tu-do.git
cd Mo-phong-robot-6-bac-tu-do
catkin_make
source ~/catkin_ws/devel/setup.bash
```

5. Cài MATLAB

* Cài MATLAB R2024a trở lên
* Thêm ROS Toolbox qua Add-Ons
* Kiểm tra kết nối ROS: `rosinit('http://<ubuntu-ip>:11311')`

🚀 Sử Dụng

1. Khởi động Mô phỏng

```bash
roslaunch moveit_ar2_sim full_ar2_sim.launch
```

* Gazebo: Hiển thị robot AR2 và môi trường.
* RViz: Theo dõi trạng thái và lập kế hoạch chuyển động.

2. Chạy Node ROS
   \| Chức năng | Lệnh |
   \|-----------|------|
   \| Theo dõi đầu cuối (EE) | `rosrun moveit_ar2_sim EE_tracker.py` |
   \| Lập kế hoạch chuyển động | `rosrun moveit_ar2_sim IK_solver.py` |
   \| Đường đi Cartesian | `rosrun moveit_ar2_sim Cartesian_path.py` |
   \| Phát hiện đối tượng | `rosrun moveit_ar2_sim Detectobject.py` |
   \| Hiệu chuẩn vị trí | `rosrun moveit_ar2_sim Calibrate.py` |
   \| Tạo vật thể trong Gazebo | `rosrun moveit_ar2_sim node_spawn_box_models_in_gazebo.py` |
   \| Đặt tư thế định sẵn | `rosrun moveit_ar2_sim node_set_predefined_pose.py` |

3. Chạy GUI MATLAB

* Mở MATLAB, kết nối ROS: `rosinit('http://<ubuntu-ip>:11311')`
* Chạy GUI: `run('Robot_6DOF_controller_GUI/gui.mlapp')`
* Hoặc dùng file thực thi: `./Robot_6DOF_controller_GUI/for_redistribution/MyAppInstaller_web.exe`

📁 Cấu Trúc Dự Án

```
Mo-phong-robot-6-bac-tu-do/
├── moveit_ws/
│   └── src/
│       ├── moveit_ar2_sim/
│       │   ├── scripts/
│       │   │   ├── IK_solver.py
│       │   │   ├── Cartesian_path.py
│       │   │   ├── Detectobject.py
│       │   │   ├── Calibrate.py
│       │   │   ├── EE_tracker.py
│       │   │   ├── node_set_predefined_pose.py
│       │   │   ├── node_spawn_box_models_in_gazebo.py
│       │   ├── launch/
│       │   │   ├── full_ar2_sim.launch
│       │   ├── config/
│       │   │   ├── joint_limits.yaml
│       ├── ar2_robot/
│       │   ├── urdf/
│       │   │   ├── ar2_robot.urdf
│       │   ├── meshes/
│       │   │   ├── Base.STL
│       │   ├── launch/
│       │   │   ├── ar2_urdf.launch
│       │   ├── config/
│       │   │   ├── joint_trajectory_controller.yaml
├── Robot_6DOF_controller_GUI/
│   ├── gui.mlapp
│   ├── for_redistribution/
│   │   ├── MyAppInstaller_web.exe
│   ├── for_testing/
│   │   ├── Robot_6DOF_controller.exe
├── .gitignore
├── .gitattributes
├── README.md
├── LICENSE
```

📐 Tài Liệu & Thiết Kế

* 📦 SolidWorks 3D: [Tải tại đây](https://drive.google.com/file/d/<YOUR_SOLIDWORKS_FILE_ID>)
* 💻 File GUI thực thi (.exe): [Tải tại đây](https://drive.google.com/file/d/<YOUR_EXE_FILE_ID>)

🖼️ Ảnh Chụp Màn Hình

* Gazebo
* RViz
* GUI MATLAB
  (Thay link ảnh bằng link thực tế sau khi tải lên GitHub Issues.)

🧠 Thuật Toán

| Tính năng            | Mô tả                                                  |
| -------------------- | ------------------------------------------------------ |
| IK Solver            | Giải bài toán nghịch (inverse kinematics) bằng MoveIt. |
| Cartesian Path       | Nội suy tuyến tính trong không gian Cartesian.         |
| Object Detection     | Phát hiện vật thể bằng xử lý hình ảnh (OpenCV).        |
| End-Effector Tracker | Theo dõi và hiệu chỉnh vị trí đầu cuối.                |

⚠️ Lưu Ý

* Mạng: MATLAB và ROS master phải chạy trên cùng mạng (LAN hoặc localhost).
* Thứ tự khởi động: Khởi động Gazebo trước, sau đó chạy node ROS và GUI MATLAB.


👤 Tác Giả

* Tên: Thiều Chí Công
* Email: [thieuchicong1048596@gmail.com](mailto:thieuchicong1048596@gmail.com)
* GitHub: [ThieuChiCong1048596](https://github.com/ThieuChiCong1048596)

📄 License
Dự án được phát hành theo MIT License.
