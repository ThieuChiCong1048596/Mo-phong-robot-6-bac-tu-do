 Mô phỏng Robot 6 Bậc Tự Do (6-DOF)
 Tổng Quan
Dự án cung cấp một môi trường mô phỏng robot công nghiệp 6 bậc tự do (6-DOF) sử dụng ROS Noetic, MoveIt, Gazebo, và giao diện điều khiển phát triển bằng MATLAB. Hệ thống tích hợp mô hình URDF từ SolidWorks, cho phép:

✅ Lập kế hoạch chuyển động (trajectory planning)

✅ Điều khiển robot (động học thuận/nghịch)

✅ Tương tác môi trường (phát hiện và thao tác đối tượng)

Ý tưởng chính:
Xây dựng hệ thống mô phỏng robot AR2 hoàn chỉnh, kết nối ROS ↔ MATLAB, xử lý hình ảnh từ cảm biến, và điều khiển linh hoạt qua GUI hoặc script Python.
👉 Phù hợp cho nghiên cứu, phát triển robot, và học tập.

🎬 Demo
📹 Xem video mô phỏng robot 6-DOF trong Gazebo, MoveIt, và GUI MATLAB:
🔗 Link YouTube hoặc Google Drive

Tính Năng
Tính năng	Mô tả
 Mô phỏng robot AR2	Tích hợp URDF + Gazebo, mô phỏng chính xác chuyển động khớp
 Lập kế hoạch chuyển động	Sử dụng MoveIt qua script Python (IK_solver.py, Cartesian_path.py)
 GUI MATLAB	Điều khiển robot qua giao diện Robot_6DOF_controller_GUI.mlapp
 Phát hiện đối tượng	Xử lý ảnh từ camera trong Gazebo (Detectobject.py)
 Tương tác môi trường	Thêm vật thể động (node_spawn_box_models_in_gazebo.py)
 Điều khiển đầu cuối	Theo dõi/đặt tư thế EE (EE_tracker.py, node_set_predefined_pose.py)
 Thiết kế SolidWorks	File 3D chi tiết: Solidwork 6DOF Assembly file.rar

 Yêu Cầu Hệ Thống
Thành phần	Yêu cầu
Hệ điều hành	Ubuntu 20.04 (Focal Fossa)
ROS	Noetic Ninjemys
MoveIt	1.1.9
Gazebo	11.x
MATLAB	R2024a+ với ROS Toolbox
Phụ trợ	Git, Python 3.8+, Catkin tools
Phần cứng	RAM 8GB+ (khuyến nghị 16GB), CPU quad-core, GPU hỗ trợ OpenGL (Gazebo)

⚙️ Cài Đặt
1. Cài ROS Noetic
bash
Sao chép
Chỉnh sửa
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
👉 Tham khảo thêm: ROS Noetic Installation Guide


3. Tạo ROS Workspace
bash
Sao chép
Chỉnh sửa
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
4. Clone Repository
bash
Sao chép
Chỉnh sửa
cd ~/catkin_ws/src
git clone https://github.com/ThieuChiCong1048596/Mo-phong-robot-6-bac-tu-do.git
cd ..
catkin_make
source devel/setup.bash
5. Cài MATLAB
Cài MATLAB R2024a hoặc mới hơn

Thêm ROS Toolbox (qua Add-Ons)

Kiểm tra kết nối:

matlab
Sao chép
Chỉnh sửa
rosinit('http://<ubuntu-ip>:11311')
🚀 Sử Dụng
1. Khởi động Mô phỏng
bash
Sao chép
Chỉnh sửa
roslaunch moveit_ar2_sim full_ar2_sim.launch
Gazebo: Hiển thị robot và môi trường

RViz: Theo dõi trạng thái và lập kế hoạch

2. Chạy Node ROS
Chức năng	Lệnh
Theo dõi EE	rosrun moveit_ar2_sim EE_tracker.py
IK Solver	rosrun moveit_ar2_sim IK_solver.py
Cartesian Path	rosrun moveit_ar2_sim Cartesian_path.py
Phát hiện vật thể	rosrun moveit_ar2_sim Detectobject.py
Hiệu chuẩn vị trí	rosrun moveit_ar2_sim Calibrate.py
Tạo vật thể trong Gazebo	rosrun moveit_ar2_sim node_spawn_box_models_in_gazebo.py
Đặt tư thế định sẵn	rosrun moveit_ar2_sim node_set_predefined_pose.py

3. Chạy GUI MATLAB
matlab
Sao chép
Chỉnh sửa
rosinit('http://<ubuntu-ip>:11311')
run('Robot_6DOF_controller_GUI/gui.mlapp')
Hoặc dùng bản cài đặt:
📦 ./Robot_6DOF_controller_GUI/for_redistribution/MyAppInstaller_web.exe

📁 Cấu Trúc Dự Án
arduino
Sao chép
Chỉnh sửa
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
📐 Tài Liệu & Thiết Kế
📦 SolidWorks 3D: Tải tại đây

💻 File GUI thực thi (.exe): Tải tại đây

🖼️ Ảnh Chụp Màn Hình
(Chèn link ảnh thực tế từ GitHub Issues hoặc Drive)

Gazebo

RViz

GUI MATLAB

🧠 Thuật Toán
Tính năng	Mô tả
IK Solver	Giải bài toán nghịch bằng MoveIt
Cartesian Path	Tạo đường đi tuyến tính trong không gian Cartesian
Object Detection	Xử lý ảnh phát hiện vật thể bằng OpenCV
EE Tracker	Theo dõi và chỉnh vị trí đầu cuối (end-effector)

⚠️ Lưu Ý
MATLAB & ROS phải cùng mạng

Thứ tự khởi động: Gazebo → ROS node → MATLAB





👤 Tác Giả
Tên: Thiều Chí Công

Email: thieuchicong1048596@gmail.com

GitHub: ThieuChiCong1048596

📄 License
Dự án được phát hành theo MIT License.
