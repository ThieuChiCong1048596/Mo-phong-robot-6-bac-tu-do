MÔ PHỎNG ROBOT 6 BẬC TỰ DO  (6-DOF)
    
Dự án này cung cấp một môi trường mô phỏng robot công nghiệp 6 bậc tự do (6-DOF) sử dụng ROS Noetic, MoveIt, Gazebo, và giao diện điều khiển (GUI) được phát triển bằng MATLAB. Dự án tích hợp mô hình URDF xuất từ SolidWorks, hỗ trợ lập kế hoạch chuyển động, điều khiển robot, và tương tác với môi trường mô phỏng, phù hợp cho nghiên cứu, phát triển robot, và học tập.
Demo
Xem video demo mô phỏng robot 6-DOF trong Gazebo, MoveIt, và GUI MATLAB tại đây:

(Thay <YOUR_DEMO_FILE_ID> bằng ID từ link Google Drive hoặc dùng link YouTube.)
Tính năng

Mô phỏng robot AR2: Sử dụng mô hình URDF và Gazebo để mô phỏng robot 6-DOF với chuyển động khớp chính xác.
Lập kế hoạch chuyển động: MoveIt tích hợp các script Python (IK_solver.py, Cartesian_path.py) để giải bài toán ngược (IK) và tạo đường đi Cartesian.
Giao diện GUI MATLAB: Điều khiển robot qua giao diện đồ họa (Robot_6DOF_controller_GUI), hỗ trợ nhập tư thế và giám sát.
Mô phỏng môi trường: Gazebo hỗ trợ thêm đối tượng động (node_spawn_box_models_in_gazebo.py) và phát hiện đối tượng (Detectobject.py).
Hỗ trợ node ROS: Bao gồm hiệu chuẩn (Calibrate.py), theo dõi đầu cuối (EE_tracker.py), và đặt tư thế định sẵn (node_set_predefined_pose.py).
Thiết kế SolidWorks: File thiết kế 3D (Solidwork 6DOF Assembly file.rar) cung cấp mô hình chi tiết của robot.

Yêu cầu

Hệ điều hành: Ubuntu 20.04 (Focal Fossa)
ROS: Noetic Ninjemys
MoveIt: 1.1.9
Gazebo: 11.x
MATLAB: R2021b trở lên với ROS Toolbox
Phần cứng:
RAM: Tối thiểu 8GB (khuyến nghị 16GB)
CPU: Quad-core trở lên
GPU: Hỗ trợ OpenGL (khuyến nghị cho Gazebo)


Phần mềm bổ sung:
Git
Python 3.8+
Catkin tools
Git LFS (nếu sử dụng file lớn)



Cài đặt
1. Cài đặt ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

2. Cài đặt MoveIt
sudo apt install ros-noetic-moveit

3. Tạo ROS Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

4. Clone Repository
cd ~/catkin_ws/src
git clone https://github.com/ThieuChiCong1048596/Mo-phong-robot-6-bac-tu-do.git
cd Mo-phong-robot-6-bac-tu-do
catkin_make
source ~/catkin_ws/devel/setup.bash

5. Cài đặt Git LFS (nếu cần)
sudo apt install git-lfs
git lfs install
git lfs pull

6. Cài đặt MATLAB ROS Toolbox

Mở MATLAB, cài ROS Toolbox qua Add-Ons.
Kiểm tra kết nối ROS:rosinit('http://<ubuntu-ip>:11311')



Sử dụng
1. Chạy Mô phỏng Gazebo và MoveIt
Khởi động môi trường mô phỏng:
roslaunch moveit_ar2_sim full_ar2_sim.launch


Gazebo hiển thị robot AR2 và môi trường.
RViz hiển thị lập kế hoạch chuyển động và trạng thái robot.

2. Chạy Các Node ROS

Phát hiện đối tượng:rosrun moveit_ar2_sim Detectobject.py


Hiệu chuẩn robot:rosrun moveit_ar2_sim Calibrate.py


Lập kế hoạch đường đi Cartesian:rosrun moveit_ar2_sim Cartesian_path.py


Đặt tư thế định sẵn:rosrun moveit_ar2_sim node_set_predefined_pose.py


Theo dõi đầu cuối (end-effector):rosrun moveit_ar2_sim EE_tracker.py


Tạo đối tượng trong Gazebo:rosrun moveit_ar2_sim node_spawn_box_models_in_gazebo.py



3. Chạy GUI MATLAB

Mở MATLAB.
Kết nối với ROS master:rosinit('http://<ubuntu-ip>:11311')


Chạy GUI:
Nếu có file gui.mlapp:run('Robot_6DOF_controller_GUI/gui.mlapp')


Hoặc chạy file thực thi (nếu có):./Robot_6DOF_controller_GUI/for_redistribution/MyAppInstaller_web.exe





Cấu trúc Dự án
Mo-phong-robot-6-bac-tu-do/
├── moveit_ws/
│   ├── src/
│   │   ├── moveit_ar2_sim/
│   │   │   ├── scripts/                # Python scripts điều khiển
│   │   │   │   ├── IK_solver.py
│   │   │   │   ├── Cartesian_path.py
│   │   │   │   ├── Detectobject.py
│   │   │   │   ├── Calibrate.py
│   │   │   │   ├── EE_tracker.py
│   │   │   │   ├── node_set_predefined_pose.py
│   │   │   │   ├── node_spawn_box_models_in_gazebo.py
│   │   │   │   ├── wait_for_gazebo.py
│   │   │   │   ├── wait_for_moveit.py
│   │   │   ├── launch/                # Launch files
│   │   │   │   ├── full_ar2_sim.launch
│   │   │   ├── config/                # File cấu hình MoveIt
│   │   │   │   ├── joint_limits.yaml
│   │   │   ├── CMakeLists.txt
│   │   │   ├── package.xml
│   │   ├── ar2_robot/
│   │   │   ├── urdf/                  # Mô hình robot
│   │   │   │   ├── ar2_robot.urdf
│   │   │   ├── meshes/                # File STL cho Gazebo
│   │   │   │   ├── Base.STL
│   │   │   ├── launch/                # Launch file robot
│   │   │   │   ├── ar2_urdf.launch
│   │   │   ├── config/                # Cấu hình điều khiển
│   │   │   │   ├── joint_trajectory_controller.yaml
├── Robot_6DOF_controller_GUI/         # GUI MATLAB
│   ├── PackagingLog.html
│   ├── gui.mlapp                      # File GUI (nếu có)
│   ├── for_redistribution/            # File thực thi MATLAB
│   │   ├── MyAppInstaller_web.exe
│   ├── for_redistribution_files_only/
│   │   ├── Robot_6DOF_controller.exe
│   ├── for_testing/
│   │   ├── Robot_6DOF_controller.exe
├── .gitignore                         # Bỏ qua file build, log
├── .gitattributes                     # Đảm bảo LF cho file ROS
├── README.md                          # Tài liệu này
├── LICENSE                            # MIT License

Tài liệu Thiết kế

File SolidWorks: Mô hình 3D của robot 6-DOF (Tải tại đây).
File thực thi GUI: Các file .exe của MATLAB (Tải tại đây).

(Thay <YOUR_SOLIDWORKS_FILE_ID> và <YOUR_EXE_FILE_ID> bằng ID từ link Google Drive.)
Ảnh chụp màn hình

Gazebo: 
RViz: 
GUI MATLAB: 

(Thay link ảnh bằng link thực tế sau khi tải lên GitHub Issues.)
Thuật toán

IK Solver: Giải bài toán ngược (inverse kinematics) bằng MoveIt, đảm bảo robot đạt tư thế mong muốn.
Cartesian Path: Tạo đường đi tuyến tính trong không gian Cartesian, tối ưu cho chuyển động mượt mà.
Object Detection: Phát hiện đối tượng trong Gazebo bằng phương pháp xử lý hình ảnh hoặc cảm biến (chi tiết trong Detectobject.py).
End-Effector Tracking: Theo dõi và điều khiển đầu cuối của robot (EE_tracker.py).

Lưu ý

File lớn:
Các file .exe, .rar được lưu trên Google Drive để giảm kích thước repository.
Nếu dùng Git LFS, đảm bảo chạy git lfs pull sau khi clone.


Yêu cầu hệ thống:
ROS master và MATLAB phải chạy trên cùng mạng.
Cấu hình joint_limits.yaml phù hợp nếu triển khai trên robot thật.


Khắc phục sự cố:
Lỗi failed_pick_in_progress: Kiểm tra topic /ik_solver_status bằng rostopic echo /ik_solver_status hoặc cấu hình lại IK_solver.py. Báo lỗi tại Issues.
Gazebo chậm: Tăng RAM hoặc giảm độ phức tạp mô phỏng.
GUI MATLAB không phản hồi: Kiểm tra kết nối ROS (rosnode list) và ROS Toolbox.



Đóng góp

Fork repository.
Tạo branch mới (git checkout -b feature/ten-tinh-nang).
Commit thay đổi (git commit -m 'Thêm tính năng XYZ').
Đẩy lên branch (git push origin feature/ten-tinh-nang).
Tạo Pull Request.

Xem chi tiết tại CONTRIBUTING.md (nếu có).
Liên hệ

Tác giả: Thieu Chi Cong
GitHub: ThieuChiCong1048596
Email: [your.email@example.com] (thay bằng email của bạn nếu muốn)

License
Dự án được cấp phép theo MIT License.
