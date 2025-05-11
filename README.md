                                                                                  MÔ PHỎNG ROBOT 6 BẬC TỰ DO  (6-DOF)
    
Tổng quan
Dự án này cung cấp một môi trường mô phỏng robot công nghiệp 6 bậc tự do (6-DOF) sử dụng ROS Noetic, MoveIt, Gazebo, và giao diện điều khiển (GUI) được phát triển bằng MATLAB. Dự án tích hợp mô hình URDF xuất từ SolidWorks, hỗ trợ lập kế hoạch chuyển động (trajectory planning), điều khiển robot (Động học thuận, Động học nghịch), và tương tác với môi trường mô phỏng, phù hợp cho nghiên cứu, phát triển robot, và học tập.
Ý tưởng chính

Demo
Xem video demo mô phỏng robot 6-DOF trong Gazebo, MoveIt, và GUI MATLAB tại đây:

(Thay <YOUR_DEMO_FILE_ID> bằng ID từ link Google Drive hoặc dùng link YouTube.)

Tính năng
Mô phỏng robot AR2: Sử dụng mô hình URDF được và Gazebo để mô phỏng robot 6-DOF với chuyển động khớp chính xác.
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

Hệ điều hành: Window 10/11
MATLAB: R2024a trở lên với ROS Toolbox

Phần mềm bổ sung:
Git
Python 3.8+
Catkin tools


Cài đặt
1. Cài đặt ROS Noetic
   
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

3. Cài đặt MoveIt
sudo apt install ros-noetic-moveit

4. Tạo ROS Workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

7. Cài đặt MATLAB bản 2024a trở lên (bao gồm Matlab ROS Toolbox)

Sử dụng
1. Chạy Mô phỏng Gazebo và MoveIt
Khởi động môi trường mô phỏng:
roslaunch moveit_ar2_sim full_ar2_sim.launch


Gazebo hiển thị robot AR2 và môi trường.
RViz hiển thị lập kế hoạch chuyển động và trạng thái robot.

2. Các chức năng chính của Node ROS

- Theo dõi tọa độ đầu cuối (end-effector):rosrun moveit_ar2_sim EE_tracker.py

- Lập kế hoạch đường đi
Di chuyển theo waypoint,nội suy: rosrun moveit_ar2_sim IK_solver.py
Di chuyển theo bảng hành động, lưới quét vật thể: rosrun moveit_ar2_sim Cartesian_path.py

- Phát hiện vật thể theo màu sắc
Phát hiện đối tượng qua camera: rosrun moveit_ar2_sim Detectobject.py
Hiệu chuẩn, dịch chuyển đến vị trí đối tượng robot: rosrun moveit_ar2_sim Calibrate.py

- Tạo đối tượng vật thể trong Gazebo:rosrun moveit_ar2_sim node_spawn_box_models_in_gazebo.py
  
- Đặt tư thế định sẵn:rosrun moveit_ar2_sim node_set_predefined_pose.py



3. Chạy GUI MATLAB
Chạy file iều khiển robot qua giao diện đồ họa (Robot_6DOF_controller_GUI)


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
│   
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

Tài liệu Mô hình Thiết kế

File SolidWorks: Mô hình 3D của robot 6-DOF (Tải tại đây).
File thực thi GUI: Các file .exe của MATLAB (Tải tại đây).

(Thay <YOUR_SOLIDWORKS_FILE_ID> và <YOUR_EXE_FILE_ID> bằng ID từ link Google Drive.)
Ảnh chụp màn hình

Gazebo: 
RViz: 
GUI MATLAB: 



Thuật toán
IK Solver: Giải bài toán ngược (inverse kinematics) bằng MoveIt, đảm bảo robot đạt tư thế mong muốn.
Cartesian Path: Tạo đường đi tuyến tính trong không gian Cartesian, tối ưu cho chuyển động mượt mà.
Object Detection: Phát hiện đối tượng trong Gazebo bằng phương pháp xử lý hình ảnh hoặc cảm biến (chi tiết trong Detectobject.py).
End-Effector Tracking: Theo dõi và điều khiển đầu cuối của robot (EE_tracker.py).

Lưu ý

Yêu cầu hệ thống:
ROS master và MATLAB phải chạy trên cùng mạng.


Tác giả: Thiều Chí Công
GitHub: ThieuChiCong1048596
Email: [thieuchicong1048596@gmail.com] 

License
Dự án được cấp phép theo MIT License.
