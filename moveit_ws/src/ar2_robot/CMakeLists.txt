cmake_minimum_required(VERSION 2.8.3)

project(ar2_robot)

# Tìm các package cần thiết
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  urdf
  xacro
  actionlib
  actionlib_msgs  # ✅ Thêm dòng này
)

# Tạo các message và service từ action files
add_action_files(
  DIRECTORY action
  FILES robot_move.action  # Tên của action file bạn tạo
)

# Sinh các message và action
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs  # ✅ Cần để generate các action message
)

# Định nghĩa package
catkin_package( 
  CATKIN_DEPENDS
  message_runtime  # ✅ Cần thiết khi dùng message/action
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
)

# Tìm package roslaunch
find_package(roslaunch)

# Cài đặt các file cấu hình, launch, meshes, urdf
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

