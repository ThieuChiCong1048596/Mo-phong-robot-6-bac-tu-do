#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float64MultiArray, String
import numpy as np

# Khởi tạo ROS Node
rospy.init_node("moveit_control", anonymous=True)

# Khởi tạo MoveIt Commander
robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("arm_group")
hand_group = moveit_commander.MoveGroupCommander("hand_ee")

# Xóa trạng thái MoveIt! khi khởi động
arm_group.stop()
arm_group.clear_pose_targets()
hand_group.stop()
hand_group.clear_pose_targets()

# Cài đặt planner tối ưu cho Cartesian path
arm_group.set_planner_id("RRTConnectkConfigDefault")
arm_group.allow_replanning(True)
arm_group.set_num_planning_attempts(10)

# Hàm tính khoảng cách Euclidean
def position_error(target, actual):
    return np.sqrt(
        (target.position.x - actual.position.x)**2 +
        (target.position.y - actual.position.y)**2 +
        (target.position.z - actual.position.z)**2
    )

# Hàm kiểm tra tin nhắn có mới không
def is_message_fresh(msg, max_age_secs=5.0):
    current_time = rospy.get_time()
    msg_time = msg.header.stamp.to_sec() if hasattr(msg, 'header') and msg.header.stamp else current_time
    return (current_time - msg_time) <= max_age_secs

# Hàm callback cho gripper commands
def gripper_command_callback(msg):
    try:
        # Kiểm tra tin nhắn mới
        current_time = rospy.get_time()
        msg_time = msg.header.stamp.to_sec() if hasattr(msg, 'header') and msg.header.stamp else current_time
        if not is_message_fresh(msg):
            rospy.logwarn("Bỏ qua lệnh gripper cũ: %s (thời gian: %.3f)", msg.data, msg_time)
            return

        command = msg.data.lower()
        rospy.loginfo("Nhận lệnh gripper: %s", command)
        hand_group.stop()
        hand_group.clear_pose_targets()
        if command == "close":
            hand_group.set_joint_value_target({"J7": 0.0, "J8": 0.0})
            hand_group.go(wait=True)
            rospy.loginfo("🤏 Gripper closed")
        elif command == "open":
            hand_group.set_joint_value_target({"J7": -0.0073, "J8": -0.0073})
            hand_group.go(wait=True)
            rospy.loginfo("🤏 Gripper opened")
        else:
            rospy.logwarn("⚠️ Lệnh gripper không hợp lệ: %s", command)
        rospy.sleep(1.0)
    except Exception as e:
        rospy.logerr("❌ Lỗi điều khiển gripper: %s", str(e))

# Hàm callback nhận tọa độ từ MATLAB (/target_pose)
def move_robot_callback(msg):
    if not is_message_fresh(msg):
        rospy.logwarn("Bỏ qua target_pose cũ: X=%.3f, Y=%.3f, Z=%.3f", 
                      msg.position.x, msg.position.y, msg.position.z)
        return

    arm_group.set_goal_position_tolerance(0.001)  
    arm_group.set_goal_orientation_tolerance(0.01)  

    rospy.loginfo("Nhận tọa độ mới từ MATLAB: X=%.3f, Y=%.3f, Z=%.3f",
                  msg.position.x, msg.position.y, msg.position.z)

    target_pose = arm_group.get_current_pose().pose
    target_pose.position.x = msg.position.x
    target_pose.position.y = msg.position.y
    target_pose.position.z = msg.position.z
    target_pose.orientation = msg.orientation

    arm_group.set_pose_target(target_pose)
    plan = arm_group.plan()
    success = False
    if plan[0]:
        arm_group.go(wait=True)
        actual_pose = arm_group.get_current_pose().pose
        error = position_error(target_pose, actual_pose)
        rospy.loginfo("Robot đã di chuyển đến vị trí mới! Sai số vị trí: %.3f m", error)
        success = True
    else:
        rospy.logwarn("Không thể tạo kế hoạch full pose cho vị trí X=%.3f, Y=%.3f, Z=%.3f",
                      msg.position.x, msg.position.y, msg.position.z)
        arm_group.set_position_target([msg.position.x, msg.position.y, msg.position.z])
        plan = arm_group.plan()
        if plan[0]:
            arm_group.go(wait=True)
            actual_pose = arm_group.get_current_pose().pose
            error = position_error(target_pose, actual_pose)
            rospy.loginfo("Robot đã di chuyển đến vị trí gần đúng (position-only)! Sai số vị trí: %.3f m", error)
            success = True
        else:
            rospy.logwarn("Cũng không thể tạo kế hoạch position-only cho X=%.3f, Y=%.3f, Z=%.3f",
                          msg.position.x, msg.position.y, msg.position.z)
            rospy.loginfo("Chấp nhận vị trí gần đúng và tiếp tục...")

    if success:
        rospy.loginfo("Vị trí thực tế: X=%.3f, Y=%.3f, Z=%.3f",
                      actual_pose.position.x, actual_pose.position.y, actual_pose.position.z)

# Hàm callback nhận PoseArray từ MATLAB (/matlab_interpolated_waypoints)
def move_interpolated_robot_callback(msg):
    if not is_message_fresh(msg):
        rospy.logwarn("Bỏ qua interpolated waypoints cũ: %d waypoints", len(msg.poses))
        return

    arm_group.set_goal_position_tolerance(0.001)  # 1mm
    arm_group.set_goal_orientation_tolerance(0.01)  # ~0.57 độ

    rospy.loginfo("Nhận %d interpolated waypoints từ MATLAB", len(msg.poses))

    # Tạo danh sách waypoints cho Cartesian path
    waypoints = []
    first_orientation = None
    for i, pose in enumerate(msg.poses):
        rospy.loginfo("Interpolated Waypoint %d: X=%.3f, Y=%.3f, Z=%.3f",
                      i+1, pose.position.x, pose.position.y, pose.position.z)
        target_pose = Pose()
        target_pose.position.x = pose.position.x
        target_pose.position.y = pose.position.y
        target_pose.position.z = pose.position.z
        # Sử dụng orientation của waypoint đầu tiên để đảm bảo tuyến tính
        if i == 0:
            first_orientation = pose.orientation
        target_pose.orientation = first_orientation
        waypoints.append(target_pose)

    # Lập kế hoạch Cartesian path
    try:
        (plan, fraction) = arm_group.compute_cartesian_path(
            waypoints,  # Danh sách waypoints
            eef_step=0.001,  # Bước 1mm
            jump_threshold=0.0  # Tắt kiểm tra nhảy khớp
        )
        if fraction == 1.0:  # Yêu cầu path hoàn chỉnh
            rospy.loginfo("Lập kế hoạch Cartesian path thành công: %.2f%%", fraction * 100)
            arm_group.execute(plan, wait=True)
            actual_pose = arm_group.get_current_pose().pose
            error = position_error(waypoints[-1], actual_pose)
            rospy.loginfo("Robot đã hoàn thành interpolated path! Sai số vị trí cuối: %.3f m", error)
            rospy.loginfo("Vị trí thực tế: X=%.3f, Y=%.3f, Z=%.3f",
                          actual_pose.position.x, actual_pose.position.y, actual_pose.position.z)
        else:
            rospy.logwarn("Lập kế hoạch Cartesian path thất bại: Chỉ hoàn thành %.2f%%", fraction * 100)
            # Fallback: Di chuyển từng waypoint
            for i, pose in enumerate(waypoints):
                arm_group.set_pose_target(pose)
                plan = arm_group.plan()
                if plan[0]:
                    arm_group.go(wait=True)
                    actual_pose = arm_group.get_current_pose().pose
                    error = position_error(pose, actual_pose)
                    rospy.loginfo("Robot đã di chuyển đến interpolated waypoint %d! Sai số vị trí: %.3f m", i+1, error)
                else:
                    rospy.logwarn("Không thể tạo kế hoạch cho interpolated waypoint %d: X=%.3f, Y=%.3f, Z=%.3f",
                                  i+1, pose.position.x, pose.position.y, pose.position.z)
                    rospy.loginfo("Bỏ qua waypoint này và tiếp tục...")
    except Exception as e:
        rospy.logerr("Lỗi khi lập kế hoạch Cartesian path: %s", str(e))
        # Fallback: Di chuyển từng waypoint
        for i, pose in enumerate(waypoints):
            arm_group.set_pose_target(pose)
            plan = arm_group.plan()
            if plan[0]:
                arm_group.go(wait=True)
                actual_pose = arm_group.get_current_pose().pose
                error = position_error(pose, actual_pose)
                rospy.loginfo("Robot đã di chuyển đến interpolated waypoint %d! Sai số vị trí: %.3f m", i+1, error)
            else:
                rospy.logwarn("Không thể tạo kế hoạch cho interpolated waypoint %d: X=%.3f, Y=%.3f, Z=%.3f",
                              i+1, pose.position.x, pose.position.y, pose.position.z)
                rospy.loginfo("Bỏ qua waypoint này và tiếp tục...")

# Hàm callback nhận PoseArray và speeds từ MATLAB (/matlab_free_waypoints, /matlab_free_speeds)
def move_free_waypoints_callback(waypoint_msg, speed_msg):
    if not is_message_fresh(waypoint_msg):
        rospy.logwarn("Bỏ qua free waypoints cũ: %d waypoints", len(waypoint_msg.poses))
        return

    arm_group.set_goal_position_tolerance(0.01)  # 5cm
    arm_group.set_goal_orientation_tolerance(0.01)  # ~2.9 độ

    rospy.loginfo("Nhận %d free waypoints và %d speeds từ MATLAB", len(waypoint_msg.poses), len(speed_msg.data))

    if len(waypoint_msg.poses) != len(speed_msg.data):
        rospy.logwarn("Số lượng waypoints (%d) và speeds (%d) không khớp!", 
                      len(waypoint_msg.poses), len(speed_msg.data))
        return

    for i, (pose, speed) in enumerate(zip(waypoint_msg.poses, speed_msg.data)):
        rospy.loginfo("Free Waypoint %d: X=%.3f, Y=%.3f, Z=%.3f, Speed=%.3f",
                      i+1, pose.position.x, pose.position.y, pose.position.z, speed)

        target_pose = arm_group.get_current_pose().pose
        target_pose.position.x = pose.position.x
        target_pose.position.y = pose.position.y
        target_pose.position.z = pose.position.z
        target_pose.orientation = pose.orientation

        arm_group.set_max_velocity_scaling_factor(speed)

        arm_group.set_pose_target(target_pose)
        plan = arm_group.plan()
        success = False
        if plan[0]:
            arm_group.go(wait=True)
            actual_pose = arm_group.get_current_pose().pose
            error = position_error(target_pose, actual_pose)
            rospy.loginfo("Robot đã di chuyển đến free waypoint %d! Sai số vị trí: %.3f m", i+1, error)
            success = True
        else:
            rospy.logwarn("Không thể tạo kế hoạch full pose cho free waypoint %d: X=%.3f, Y=%.3f, Z=%.3f, Speed=%.3f",
                          i+1, pose.position.x, pose.position.y, pose.position.z, speed)
            arm_group.set_position_target([pose.position.x, pose.position.y, pose.position.z])
            plan = arm_group.plan()
            if plan[0]:
                arm_group.go(wait=True)
                actual_pose = arm_group.get_current_pose().pose
                error = position_error(target_pose, actual_pose)
                rospy.loginfo("Robot đã di chuyển đến free waypoint %d (position-only)! Sai số vị trí: %.3f m", i+1, error)
                success = True
            else:
                rospy.logwarn("Cũng không thể tạo kế hoạch position-only cho free waypoint %d: X=%.3f, Y=%.3f, Z=%.3f, Speed=%.3f",
                              i+1, pose.position.x, pose.position.y, pose.position.z, speed)
                rospy.loginfo("Chấp nhận vị trí gần đúng và tiếp tục...")

        if success:
            rospy.loginfo("Vị trí thực tế: X=%.3f, Y=%.3f, Z=%.3f",
                          actual_pose.position.x, actual_pose.position.y, actual_pose.position.z)

# Đăng ký Subscribers với queue_size nhỏ
rospy.Subscriber("/target_pose", Pose, move_robot_callback, queue_size=1)
rospy.Subscriber("/matlab_interpolated_waypoints", PoseArray, move_interpolated_robot_callback, queue_size=1)
rospy.Subscriber("/matlab_gripper_joint_commands", String, gripper_command_callback, queue_size=1)

# Sử dụng message_filters để đồng bộ waypoints và speeds
import message_filters
wp_sub = message_filters.Subscriber('/matlab_free_waypoints', PoseArray, queue_size=1)
speed_sub = message_filters.Subscriber('/matlab_free_speeds', Float64MultiArray, queue_size=1)
ts = message_filters.ApproximateTimeSynchronizer([wp_sub, speed_sub], queue_size=1, slop=0.1, allow_headerless=True)
ts.registerCallback(move_free_waypoints_callback)

rospy.spin()  # Lắng nghe dữ liệu ROS liên tục
