# Mô phỏng Robot 6 Bậc Tự Do (6-DOF)

## Tổng Quan

Dự án cung cấp một môi trường mô phỏng robot công nghiệp 6 bậc tự do (6-DOF) sử dụng **ROS Noetic**, **MoveIt**, **Gazebo** và **giao diện MATLAB GUI**. Hệ thống tích hợp mô hình URDF từ SolidWorks, hỗ trợ:

* Lập kế hoạch chuyển động (trajectory planning)
* Điều khiển robot (động học thuận/nghịch)
* Tương tác môi trường (phát hiện và thao tác đối tượng)

**Ý tưởng chính:** Xây dựng hệ thống mô phỏng robot AR2 hoàn chỉnh, kết nối ROS ↔ MATLAB, xử lý hình ảnh từ cảm biến, và điều khiển linh hoạt qua GUI hoặc script Python.
![Demo GUI](https://drive.google.com/file/d/1YwCPNcOnOscLRBTUyOEucLvSFvD39UGx/view?usp=drive_link)
Phù hợp cho: Nghiên cứu học thuật, phát triển ứng dụng robot, và giảng dạy.

## Demo

**Video mô phỏng** (Gazebo, MoveIt, GUI MATLAB):
[https://youtube.com/](https://youtube.com/)...

## Tính Năng

* Mô phỏng robot 6 bậc tự do: URDF + Gazebo với mô phỏng khớp chính xác
* Lập kế hoạch chuyển động: MoveIt, điều khiển qua Python (`IK_solver.py`, `Cartesian_path.py`)
* Giao diện MATLAB: Điều khiển trực quan qua GUI (`gui.mlapp`)
* Phát hiện đối tượng: Xử lý ảnh với OpenCV (`Detectobject.py`)
* Tương tác môi trường: Tạo vật thể trong Gazebo (`node_spawn_box_models_in_gazebo.py`)
* Điều khiển đầu cuối: Theo dõi và điều chỉnh End-Effector (`EE_tracker.py`, `node_set_predefined_pose.py`)
* Thiết kế 3D: Mô hình SolidWorks đầy đủ

## Yêu Cầu Hệ Thống

| Thành phần            | Phiên bản hoặc mô tả                                        |
| --------------------- | ----------------------------------------------------------  |
| Hệ điều hành          | Window 10/11 cho Matlab, Ubuntu 20.04 (Focal Fossa) cho ROS |
| ROS                   | Noetic Ninjemys                                             |
| MoveIt                | 1.1.9                                                       |
| Gazebo                | 11.x                                                        |
| MATLAB                | R2024a+ với ROS Toolbox                                     |
| Phần mềm bổ sung      | Git, Python 3.8+, Catkin tools                              |
| Phần cứng khuyến nghị | RAM ≥ 8GB (tốt nhất 16GB), CPU ≥ 4 nhân, GPU hỗ trợ OpenGL  |

## Cài Đặt

1. **Cài Đặt ROS Noetic**

Vui lòng tham khảo hướng dẫn chính thức tại:👉 https://wiki.ros.org/noetic/Installation/Ubuntu

Hoặc xem hướng dẫn chi tiết bằng tiếng Việt tại:👉 https://robodev.blog/cai-dat-ros-noetic

2. **Cài MoveIt**

```bash
sudo apt install ros-noetic-moveit
```

3. **Tạo ROS Workspace**

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

4. **Clone Repository**

```bash
cd ~/catkin_ws/src
git clone https://github.com/ThieuChiCong1048596/Mo-phong-robot-6-bac-tu-do.git
cd ../
catkin_make
source devel/setup.bash
```

5. **Cài MATLAB**

* Cài phiên bản R2024a hoặc mới hơn
* Cài ROS Toolbox qua Add-Ons
* Kiểm tra kết nối:

```matlab
rosinit('http://<ubuntu-ip>:11311')
```

## Hướng Dẫn Sử Dụng

### 1. Khởi động mô phỏng

```bash
roslaunch moveit_ar2_sim full_ar2_sim.launch
```

### 2. Chạy các node ROS

| Chức năng                  | Lệnh thực thi                                              |
| -------------------------- | ---------------------------------------------------------- |
| Theo dõi End-Effector      | `rosrun moveit_ar2_sim EE_tracker.py`                      |
| Lập kế hoạch IK            | `rosrun moveit_ar2_sim IK_solver.py`                       |
| Nội suy đường đi Cartesian | `rosrun moveit_ar2_sim Cartesian_path.py`                  |
| Phát hiện đối tượng        | `rosrun moveit_ar2_sim Detectobject.py`                    |
| Hiệu chuẩn vị trí          | `rosrun moveit_ar2_sim Calibrate.py`                       |
| Sinh vật thể trong Gazebo  | `rosrun moveit_ar2_sim node_spawn_box_models_in_gazebo.py` |
| Đặt tư thế định sẵn        | `rosrun moveit_ar2_sim node_set_predefined_pose.py`        |

### 3. Giao diện GUI MATLAB

```matlab
rosinit('http://<ubuntu-ip>:11311')
run('Robot_6DOF_controller_GUI/gui.mlapp')
```

Hoặc chạy file thực thi:

```bash
./Robot_6DOF_controller_GUI/for_redistribution/MyAppInstaller_web.exe
```

## Cấu Trúc Dự Án

```text
Mo-phong-robot-6-bac-tu-do/
├── moveit_ws/
│   └── src/
│       ├── moveit_ar2_sim/
│       │   ├── scripts/
│       │   ├── launch/
│       │   ├── config/
│       ├── ar2_robot/
│       │   ├── urdf/
│       │   ├── meshes/
│       │   ├── launch/
│       │   ├── config/
├── Robot_6DOF_controller_GUI/
│   ├── gui.mlapp
│   ├── for_redistribution/
│   ├── for_testing/
├── .gitignore
├── .gitattributes
├── README.md
├── LICENSE
```

## Tài Liệu & Thiết Kế

* File SolidWorks 3D (Giải nén trên Desktop, yêu cầu bản SolidWorks từ 2022 trở lên): (https://drive.google.com/drive/folders/10BUfy0PVBtMiwYveavlsReMv1TZBFfdv?usp=drive_link)
* GUI Executable (.exe): có trong .zip

## Thuật Toán

| Tính năng          | Mô tả                               |
| ------------------ | ----------------------------------- |
| IK Solver          | Giải inverse kinematics bằng MoveIt |
| Cartesian Path     | Nội suy tuyến tính trong không gian |
| Object Detection   | Phát hiện vật thể qua xử lý ảnh     |
| End-Effector Track | Theo dõi vị trí và hiệu chỉnh       |

## Lưu ý

* MATLAB và ROS master cần cùng mạng LAN hoặc chạy `localhost`
* Khởi động mô phỏng theo thứ tự: Gazebo → ROS nodes → MATLAB GUI
* Một số lỗi thường gặp:



## Tác Giả

* Tên: Thiều Chí Công
* Email: [thieuchicong1048596@gmail.com](mailto:thieuchicong1048596@gmail.com)
* GitHub: [ThieuChiCong1048596](https://github.com/ThieuChiCong1048596)

## License

Dự án được phát hành theo giấy phép [MIT License](./LICENSE)
