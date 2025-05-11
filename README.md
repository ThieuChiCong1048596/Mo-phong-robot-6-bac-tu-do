Mô phỏng Robot 6 Bậc Tự Do (6-DOF)
Tổng Quan
Dự án cung cấp một môi trường mô phỏng robot công nghiệp 6 bậc tự do (6-DOF) sử dụng ROS Noetic, MoveIt, Gazebo, và giao diện điều khiển phát triển bằng MATLAB. Hệ thống tích hợp mô hình URDF từ SolidWorks, cho phép:

Lập kế hoạch chuyển động (trajectory planning)

Điều khiển robot (động học thuận/nghịch)

Tương tác với môi trường (phát hiện và thao tác đối tượng)

Ý tưởng chính: Xây dựng hệ thống mô phỏng robot AR2 hoàn chỉnh, kết nối ROS ↔ MATLAB, xử lý hình ảnh từ cảm biến, và điều khiển linh hoạt qua GUI hoặc script Python. Phù hợp cho mục đích nghiên cứu, phát triển robot, và học tập.

Demo
Xem video mô phỏng robot 6-DOF trong Gazebo, MoveIt và GUI MATLAB:

(Thay <YOUR_DEMO_FILE_ID> bằng ID Google Drive hoặc sử dụng link YouTube phù hợp)

Tính Năng
Mô phỏng robot AR2: Tích hợp URDF và Gazebo cho chuyển động khớp chính xác

Lập kế hoạch chuyển động: Sử dụng MoveIt với script Python (IK_solver.py, Cartesian_path.py)

Giao diện MATLAB: Điều khiển và giám sát robot qua GUI (Robot_6DOF_controller_GUI)

Phát hiện vật thể: Nhận diện đối tượng trong môi trường mô phỏng (Detectobject.py)

Tương tác với môi trường: Thêm vật thể động trong Gazebo (node_spawn_box_models_in_gazebo.py)

Theo dõi đầu cuối: Giám sát và đặt tư thế định sẵn (EE_tracker.py, node_set_predefined_pose.py)

Thiết kế 3D: Mô hình SolidWorks chi tiết (Solidwork 6DOF Assembly file.rar)

Yêu Cầu Hệ Thống
Yêu cầu	Chi tiết
Hệ điều hành	Ubuntu 20.04 (Focal Fossa)
ROS	Noetic Ninjemys
MoveIt	1.1.9
Gazebo	11.x
MATLAB	R2024a+ với ROS Toolbox
Phần mềm bổ sung	Git, Python 3.8+, Catkin tools
Phần cứng khuyến nghị	RAM: 8GB (tối thiểu, 16GB khuyến nghị), CPU: Quad-core+, GPU hỗ trợ OpenGL

Cài Đặt
Cài ROS Noetic

bash
Sao chép
Chỉnh sửa
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
Cài MoveIt

bash
Sao chép
Chỉnh sửa
sudo apt install ros-noetic-moveit
Tạo ROS Workspace

bash
Sao chép
Chỉnh sửa
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
Clone Repository

bash
Sao chép
Chỉnh sửa
cd ~/catkin_ws/src
git clone https://github.com/ThieuChiCong1048596/Mo-phong-robot-6-bac-tu-do.git
cd ../
catkin_make
source devel/setup.bash
Cài MATLAB

Cài MATLAB R2024a trở lên

Cài thêm ROS Toolbox từ Add-Ons

Kiểm tra kết nối:

matlab
Sao chép
Chỉnh sửa
rosinit('http://<ubuntu-ip>:11311')
Hướng Dẫn Sử Dụng
1. Khởi Động Mô Phỏng
bash
Sao chép
Chỉnh sửa
roslaunch moveit_ar2_sim full_ar2_sim.launch
Gazebo: hiển thị robot AR2 và môi trường.

RViz: theo dõi trạng thái và lập kế hoạch chuyển động.

2. Chạy Node ROS
Chức năng	Lệnh chạy
Theo dõi đầu cuối (EE)	rosrun moveit_ar2_sim EE_tracker.py
Lập kế hoạch IK	rosrun moveit_ar2_sim IK_solver.py
Đường đi Cartesian	rosrun moveit_ar2_sim Cartesian_path.py
Phát hiện vật thể	rosrun moveit_ar2_sim Detectobject.py
Hiệu chuẩn vị trí	rosrun moveit_ar2_sim Calibrate.py
Tạo vật thể trong Gazebo	rosrun moveit_ar2_sim node_spawn_box_models_in_gazebo.py
Đặt tư thế định sẵn	rosrun moveit_ar2_sim node_set_predefined_pose.py

3. Chạy GUI MATLAB
Kết nối ROS:

matlab
Sao chép
Chỉnh sửa
rosinit('http://<ubuntu-ip>:11311')
Chạy GUI:

matlab
Sao chép
Chỉnh sửa
run('Robot_6DOF_controller_GUI/gui.mlapp')
Hoặc sử dụng file thực thi: ./Robot_6DOF_controller_GUI/for_redistribution/MyAppInstaller_web.exe

Cấu Trúc Dự Án
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
Tài Liệu & Thiết Kế
SolidWorks 3D: Tải tại đây

File GUI thực thi (.exe): Tải tại đây

Ảnh Chụp Màn Hình
Gazebo

RViz

GUI MATLAB

(Thay thế bằng link ảnh thực tế trên GitHub hoặc Google Drive)

Thuật Toán
Tính năng	Mô tả
IK Solver	Giải bài toán nghịch (inverse kinematics) bằng MoveIt
Cartesian Path	Nội suy tuyến tính trong không gian Cartesian
Object Detection	Phát hiện vật thể bằng xử lý ảnh (OpenCV)
EE Tracker	Theo dõi và hiệu chỉnh vị trí đầu cuối

Lưu Ý
Mạng: MATLAB và ROS master phải cùng mạng (LAN hoặc localhost)

Thứ tự khởi động: Gazebo → Node ROS → GUI MATLAB

Khắc phục lỗi:

Lỗi failed_pick_in_progress: kiểm tra bằng rostopic echo /ik_solver_status

Gazebo chậm: nâng cấp RAM hoặc giảm độ chi tiết mô phỏng

GUI không phản hồi: kiểm tra ROS bằng rosnode list

Tác Giả
Tên: Thiều Chí Công

Email: thieuchicong1048596@gmail.com

GitHub: ThieuChiCong1048596

License
Dự án được phát hành theo giấy phép MIT License.
