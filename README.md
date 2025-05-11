# 🤖 MÔ PHỎNG ROBOT 6 BẬC TỰ DO (6-DOF)

## 🧭 Tổng Quan

Dự án này cung cấp một môi trường mô phỏng robot công nghiệp 6 bậc tự do sử dụng **ROS Noetic**, **MoveIt**, **Gazebo**, và **GUI MATLAB**. Hệ thống tích hợp mô hình URDF từ SolidWorks, hỗ trợ:

- Lập kế hoạch chuyển động (trajectory planning)
- Điều khiển robot (Động học thuận/nghịch)
- Tương tác với môi trường mô phỏng

Phù hợp cho nghiên cứu, phát triển robot và học tập.

---

## 💡 Ý Tưởng Chính

Xây dựng hệ thống mô phỏng robot AR2 6-DOF hoàn chỉnh có khả năng:
- Kết nối ROS ↔ MATLAB
- Giao tiếp cảm biến → xử lý hình ảnh
- Điều khiển robot linh hoạt bằng GUI hoặc script

---

## 🎬 Demo

📹 Xem video mô phỏng: [Demo Robot 6-DOF](https://drive.google.com/file/d/<YOUR_DEMO_FILE_ID>/view)  
Hoặc YouTube: [https://youtube.com/...](https://youtube.com/...)

---

## ✨ Tính Năng

- ✅ **Mô phỏng robot AR2** với URDF + Gazebo
- ✅ **Lập kế hoạch chuyển động** bằng MoveIt + Python script
- ✅ **Giao diện MATLAB GUI** điều khiển và giám sát robot
- ✅ **Phát hiện đối tượng**, thêm vật thể vào môi trường mô phỏng
- ✅ **Điều khiển đầu cuối**, đặt tư thế định sẵn
- ✅ **Thiết kế 3D bằng SolidWorks**

---

## 📦 Yêu Cầu Hệ Thống

### 🐧 Ubuntu
- **Ubuntu:** 20.04 (Focal)
- **ROS:** Noetic Ninjemys
- **MoveIt:** 1.1.9
- **Gazebo:** 11.x

### 🪟 Windows
- **Windows:** 10/11
- **MATLAB:** R2024a+ với ROS Toolbox

### 🔧 Phần mềm bổ sung
- Git, Python 3.8+, Catkin tools

---

## ⚙️ Cài Đặt

### 1. Cài ROS Noetic  
👉 [Hướng dẫn chính thức](http://wiki.ros.org/noetic/Installation)

### 2. Cài MoveIt
```bash
sudo apt install ros-noetic-moveit
3. Tạo ROS Workspace
bash
Sao chép
Chỉnh sửa
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
4. MATLAB
Cài bản MATLAB R2024a trở lên, kèm ROS Toolbox

🚀 Sử Dụng
1. Khởi động mô phỏng
bash
Sao chép
Chỉnh sửa
roslaunch moveit_ar2_sim full_ar2_sim.launch
Gazebo hiển thị robot + môi trường

RViz theo dõi trạng thái & lập kế hoạch

2. Chạy các node chính
Chức năng	Lệnh
Theo dõi đầu cuối (EE)	rosrun moveit_ar2_sim EE_tracker.py
Lập kế hoạch theo waypoint	rosrun moveit_ar2_sim IK_solver.py
Quét & hành động	rosrun moveit_ar2_sim Cartesian_path.py
Phát hiện vật thể	rosrun moveit_ar2_sim Detectobject.py
Hiệu chuẩn vị trí	rosrun moveit_ar2_sim Calibrate.py
Tạo vật thể trong Gazebo	rosrun moveit_ar2_sim node_spawn_box_models_in_gazebo.py
Đặt tư thế định sẵn	rosrun moveit_ar2_sim node_set_predefined_pose.py

3. Chạy GUI MATLAB
Mở Robot_6DOF_controller_GUI trong MATLAB để điều khiển robot bằng giao diện đồ họa

📁 Cấu Trúc Dự Án
arduino
Sao chép
Chỉnh sửa
Mo-phong-robot-6-bac-tu-do/
├── moveit_ws/
│   └── src/
│       ├── moveit_ar2_sim/
│       │   ├── scripts/                  # Python điều khiển
│       │   ├── launch/
│       │   ├── config/
│       ├── ar2_robot/
│           ├── urdf/, meshes/, launch/
│           ├── config/
├── Robot_6DOF_controller_GUI/           # GUI MATLAB
│   ├── for_redistribution/
│   ├── for_testing/
├── .gitignore, .gitattributes
├── README.md
├── LICENSE
📐 Tài Liệu & Thiết Kế
📦 SolidWorks 3D: Tải tại đây

💻 File GUI thực thi (.exe): Tải tại đây

🧠 Thuật Toán
Tính năng	Mô tả
IK Solver	Giải bài toán nghịch bằng MoveIt
Cartesian Path	Nội suy tuyến tính trong không gian
Object Detection	Phát hiện vật bằng xử lý hình ảnh
End-Effector Tracker	Theo dõi và hiệu chỉnh vị trí đầu cuối

🖼️ Ảnh Chụp Màn Hình
Gazebo	RViz	GUI MATLAB

⚠️ Lưu Ý
MATLAB và ROS nên chạy cùng mạng nội bộ (LAN hoặc localhost).

Các node cần được khởi động theo đúng trình tự để tránh lỗi.

👤 Tác Giả
Thiều Chí Công
📧 Email: thieuchicong1048596@gmail.com
🌐 GitHub: ThieuChiCong1048596

📄 License
Dự án phát hành theo giấy phép MIT License.
