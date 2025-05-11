#!/usr/bin/env python3
import rospy
import moveit_commander
from std_msgs.msg import Float64MultiArray, Float64, Bool, String
from geometry_msgs.msg import PoseArray, Pose, Quaternion
import numpy as np
import tf.transformations as tf_trans
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class UltimateRobotController:
    def __init__(self):
        rospy.init_node("robot_ultimate_controller", anonymous=True)
        rospy.loginfo("Initializing UltimateRobotController...")

        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        try:
            self.arm_group = moveit_commander.MoveGroupCommander("arm_group")
            self.hand_group = moveit_commander.MoveGroupCommander("hand_ee")
        except Exception as e:
            rospy.logerr(f"❌ Lỗi khởi tạo MoveIt!: {str(e)}")
            raise

        self.arm_group.set_planning_time(10.0)  # Tăng thời gian lập kế hoạch
        self.arm_group.set_num_planning_attempts(5)
        self.arm_group.set_goal_joint_tolerance(0.01)  # Dung sai khớp (radian)

        self.current_speed = rospy.get_param("~current_speed", 0.1)
        self.max_pose_distance = rospy.get_param("~max_pose_distance", 0.3)
        self.quat_fixed = list(tf_trans.quaternion_from_euler(
            rospy.get_param("~roll", -np.pi/2),
            rospy.get_param("~pitch", np.pi),
            rospy.get_param("~yaw", 0)
        ))
        self.durations = []

        self.is_paused = False
        self.is_executing_pick = False
        self.pause_reason = None
        self.pick_status = None

        rospy.Subscriber("/pause_waypoints", Bool, self.pause_callback)
        rospy.Subscriber("/pick_status", String, self.pick_status_callback)
        rospy.Subscriber("/matlab_waypoints", PoseArray, self.waypoints_callback)
        rospy.Subscriber("/pick_waypoints", PoseArray, self.pick_waypoints_callback)
        rospy.Subscriber("/adjust_waypoints", PoseArray, self.adjust_waypoints_callback)
        rospy.Subscriber("/matlab_joint_target", Float64MultiArray, self.joint_target_callback)
        rospy.Subscriber("/matlab_single_speed", Float64, self.speed_callback)
        rospy.Subscriber("/matlab_stop_command", Bool, self.stop_command_callback)
        rospy.Subscriber("/j5_quaternion", Quaternion, self.quat_callback)
        rospy.Subscriber("/matlab_durations", Float64MultiArray, self.durations_callback)
        rospy.Subscriber("/matlab_gripper_command", String, self.gripper_command_callback)
        rospy.Subscriber("/matlab_pause_command", Bool, self.matlab_pause_callback)

        self.pick_result_pub = rospy.Publisher("/pick_result", String, queue_size=10)
        self.adjust_result_pub = rospy.Publisher("/adjust_result", String, queue_size=10)
        self.joint_target_result_pub = rospy.Publisher("/joint_target_result", String, queue_size=10)

        rospy.loginfo("✅ Ultimate Robot Controller khởi động thành công!")

    def speed_callback(self, msg):
        new_speed = np.clip(msg.data, 0, 1)
        if abs(new_speed - self.current_speed) > 0.001:
            self.current_speed = new_speed
            rospy.loginfo(f"⚙️ Tốc độ cập nhật: {self.current_speed:.3f}")
        else:
            rospy.logdebug(f"⚙️ Tốc độ không thay đổi: {self.current_speed:.3f}")

    def joint_target_callback(self, msg):
        if self.pick_status == "in_progress":
            rospy.loginfo("⏸️ CameraAdjustNode đang gắp, bỏ qua joint target.")
            self.joint_target_result_pub.publish(String("failed_pick_in_progress"))
            return

        # Giả sử msg.data chứa danh sách joint target (mỗi target có 6 giá trị)
        joints_data = list(msg.data)
        if len(joints_data) % 6 != 0:
            rospy.logwarn(f"⚠️ Nhận dữ liệu joint không hợp lệ ({len(joints_data)} giá trị).")
            self.joint_target_result_pub.publish(String("failed_invalid_joint_count"))
            return

        # Chia dữ liệu thành danh sách các joint target
        joint_targets = [joints_data[i:i+6] for i in range(0, len(joints_data), 6)]
        rospy.loginfo(f"📍 Nhận {len(joint_targets)} joint target để thực hiện.")

        if self.is_paused:
            if not self.wait_for_resume():
                self.joint_target_result_pub.publish(String("failed_paused"))
                return

        max_attempts = rospy.get_param("~max_attempts", 1)
        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        success_count = 0

        for idx, joints in enumerate(joint_targets):
            try:
                # Log trạng thái khớp hiện tại
                current_joints = self.arm_group.get_current_joint_values()
                rospy.loginfo(f"📊 Trạng thái khớp hiện tại (radian): {dict(zip(joint_names, current_joints))}")

                joint_goal = dict(zip(joint_names, joints))
                rospy.loginfo(f"🚀 Thực thi Joint Target {idx+1}: {joint_goal}")
                self.arm_group.set_joint_value_target(joint_goal)
                self.arm_group.set_max_velocity_scaling_factor(self.current_speed)
                self.arm_group.set_max_acceleration_scaling_factor(self.current_speed / 2)

                current_pose = self.arm_group.get_current_pose().pose
                if abs(current_pose.position.x) > 1.5 or abs(current_pose.position.y) > 1.5 or \
                   current_pose.position.z < 0.095 or current_pose.position.z > 0.5:
                    rospy.logwarn(f"⚠️ Vị trí hiện tại không an toàn: x={current_pose.position.x:.3f}, "
                                  f"y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
                    self.joint_target_result_pub.publish(String(f"failed_unsafe_pose_target_{idx+1}"))
                    continue

                plan_success = False
                for attempt in range(max_attempts):
                    plan = self.arm_group.plan()
                    if isinstance(plan, tuple) and len(plan) >= 2 and plan[0]:
                        plan_success = True
                        rospy.loginfo(f"📈 Quỹ đạo Joint Target {idx+1}: {len(plan[1].joint_trajectory.points)} điểm")
                        try:
                            if self.arm_group.go(wait=True):
                                rospy.loginfo(f"✅ Joint Target {idx+1} thực hiện thành công.")
                                self.joint_target_result_pub.publish(String(f"success_target_{idx+1}"))
                                success_count += 1
                                break
                            else:
                                rospy.logwarn(f"⚠️ Thực thi Joint Target {idx+1} thất bại, "
                                              f"thử lại lần {attempt+1}. Lỗi: CONTROL_FAILED")
                        except moveit_commander.MoveItCommanderException as e:
                            rospy.logwarn(f"⚠️ Thực thi Joint Target {idx+1} thất bại, "
                                          f"thử lại lần {attempt+1}. Lỗi: {str(e)}")
                    else:
                        rospy.logwarn(f"⚠️ Kế hoạch IK thất bại cho Joint Target {idx+1}, "
                                      f"thử lại lần {attempt+1}.")
                
                if not plan_success:
                    rospy.logerr(f"❌ Joint Target {idx+1} thất bại: Không tìm được kế hoạch.")
                    self.joint_target_result_pub.publish(String(f"failed_no_plan_target_{idx+1}"))
                elif plan_success and not self.arm_group.go(wait=True):
                    rospy.logerr(f"❌ Joint Target {idx+1} thất bại: Thực thi thất bại (CONTROL_FAILED).")
                    self.joint_target_result_pub.publish(String(f"failed_control_failed_target_{idx+1}"))

            except Exception as e:
                rospy.logerr(f"❌ Lỗi khi thực thi Joint Target {idx+1}: {str(e)}")
                self.joint_target_result_pub.publish(String(f"failed_execution_error_target_{idx+1}: {str(e)}"))

        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        rospy.loginfo(f"🏁 Hoàn tất {len(joint_targets)} joint target: {success_count} thành công, "
                      f"{len(joint_targets) - success_count} thất bại.")

    def pause_callback(self, msg):
        if self.is_executing_pick:
            rospy.loginfo("⏩ Đang thực thi pick, bỏ qua pause")
            return
        self.is_paused = msg.data
        self.pause_reason = "camera" if self.is_paused else None
        if self.is_paused:
            rospy.loginfo("⏸️ Robot nhận lệnh pause từ Camera Node.")
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            rospy.sleep(0.5)
        else:
            rospy.loginfo("▶️ Robot nhận lệnh tiếp tục.")

    def matlab_pause_callback(self, msg):
        if self.is_executing_pick:
            rospy.loginfo("⏩ Đang thực thi pick, bỏ qua MATLAB pause")
            return
        self.is_paused = msg.data
        self.pause_reason = "matlab" if self.is_paused else None
        if self.is_paused:
            rospy.loginfo("⏸️ Robot nhận lệnh pause từ MATLAB.")
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            rospy.sleep(0.5)
        else:
            rospy.loginfo("▶️ Robot nhận lệnh tiếp tục từ MATLAB.")

    def pick_status_callback(self, msg):
        self.pick_status = msg.data
        rospy.loginfo(f"📬 Pick status: {self.pick_status}")
        if self.pick_status in ["success", "failed"]:
            self.is_paused = False
            self.pause_reason = None
            rospy.loginfo("▶️ CameraAdjustNode hoàn tất, tiếp tục controller.")

    def durations_callback(self, msg):
        self.durations = list(msg.data)
        rospy.loginfo(f"⏱️ Nhận {len(self.durations)} durations: {self.durations}")

    def quat_callback(self, msg):
        new_quat = [msg.x, msg.y, msg.z, msg.w]
        norm = np.sqrt(sum(x*x for x in new_quat))
        rospy.loginfo(f"📡 Nhận quaternion: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}, w={msg.w:.3f}, norm={norm:.3f}")
        if abs(norm - 1.0) > 0.05:
            rospy.logwarn("⚠️ Quaternion không hợp lệ (norm không gần 1), giữ giá trị cũ.")
            return
        self.quat_fixed = new_quat
        rospy.loginfo(f"🔄 Quaternion J5 cập nhật: {self.quat_fixed}")

    def gripper_command_callback(self, msg):
        try:
            command = msg.data.lower()
            self.hand_group.stop()
            self.hand_group.clear_pose_targets()
            if command == "close":
                self.hand_group.set_joint_value_target({"J7": 0.0, "J8": 0.0})
                self.hand_group.go(wait=True)
                rospy.loginfo("🤏 Gripper closed")
            elif command == "open":
                self.hand_group.set_joint_value_target({"J7": -0.0073, "J8": -0.0073})
                self.hand_group.go(wait=True)
                rospy.loginfo("🤏 Gripper opened")
            else:
                rospy.logwarn(f"⚠️ Lệnh gripper không hợp lệ: {command}")
            rospy.sleep(1.0)
        except Exception as e:
            rospy.logerr(f"❌ Lỗi điều khiển gripper: {str(e)}")

    def stop_command_callback(self, msg):
        if msg.data:
            rospy.loginfo("🛑 Robot nhận lệnh STOP từ MATLAB.")
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()

    def adjust_j5_orientation(self, pose):
        try:
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = self.quat_fixed
            return pose
        except Exception as e:
            rospy.logerr(f"❌ Lỗi khi gán quaternion: {str(e)}")
            return pose

    def check_pose_distance(self, target_pose):
        try:
            current_pose = self.arm_group.get_current_pose().pose
            dx = target_pose.position.x - current_pose.position.x
            dy = target_pose.position.y - current_pose.position.y
            dz = target_pose.position.z - current_pose.position.z
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            rospy.loginfo(f"📏 Khoảng cách đến pose: {distance:.4f}m")
            rospy.loginfo(f"📍 Current pose: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
            rospy.loginfo(f"📍 Target pose: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
            if distance > self.max_pose_distance:
                rospy.logwarn(f"⚠️ Khoảng cách đến pose ({distance:.4f}m) vượt quá giới hạn {self.max_pose_distance}m.")
                return False, distance
            return True, distance
        except Exception as e:
            rospy.logerr(f"❌ Lỗi khi kiểm tra khoảng cách pose: {str(e)}")
            return False, 0.0

    def get_pose_from_joint_state(self, joint_state):
        try:
            robot_state = RobotState()
            robot_state.joint_state = JointState()
            robot_state.joint_state.name = self.arm_group.get_active_joints()
            robot_state.joint_state.position = joint_state
            fk_result = self.robot.get_fk("world", self.arm_group.get_end_effector_link(), robot_state)
            if fk_result.error_code.val == 1:
                return fk_result.pose_stamped[0].pose
            else:
                rospy.logwarn("⚠️ Không thể tính forward kinematics")
                return None
        except Exception as e:
            rospy.logerr(f"❌ Lỗi khi tính forward kinematics: {str(e)}")
            return None

    def wait_for_resume(self):
        rospy.loginfo("⏸️ Đang chờ tín hiệu tiếp tục...")
        rate = rospy.Rate(10)
        timeout = rospy.get_param("~timeout", 10.0)
        start_time = rospy.get_time()
        while self.is_paused and (rospy.get_time() - start_time < timeout):
            if not self.is_paused or self.pick_status in ["success", "failed"]:
                rospy.loginfo("▶️ Tín hiệu tiếp tục nhận được.")
                return True
            rate.sleep()
        self.is_paused = False
        self.pause_reason = None
        rospy.logwarn("⚠️ Timeout khi chờ resume, bỏ qua pose này.")
        return False

    def waypoints_callback(self, msg):
        if self.pick_status == "in_progress":
            rospy.loginfo("⏸️ CameraAdjustNode đang gắp, bỏ qua waypoints.")
            return

        waypoints = msg.poses
        if not waypoints:
            rospy.loginfo("📭 Không có waypoint nhận được.")
            return

        rospy.loginfo(f"📍 Nhận {len(waypoints)} pose để thực hiện.")
        max_attempts = rospy.get_param("~max_attempts", 5)

        for i, pose in enumerate(waypoints):
            if abs(pose.position.x) > 1.5 or abs(pose.position.y) > 1.5 or pose.position.z < 0.095 or pose.position.z > 0.5:
                rospy.logwarn(f"⚠️ Pose {i+1} ngoài không gian làm việc: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
                continue

            if self.is_paused or self.pick_status == "in_progress":
                if not self.wait_for_resume():
                    continue

            if i < len(self.durations) and self.durations[i] > 0:
                dist = self.check_pose_distance(pose)[1]
                speed = dist / self.durations[i] if dist > 0 else self.current_speed
                speed = np.clip(speed, 0, 1)
                rospy.loginfo(f"⚙️ Tốc độ cho Pose {i+1}: {speed:.3f} (từ duration {self.durations[i]:.3f}s)")
            else:
                speed = self.current_speed
                rospy.loginfo(f"⚙️ Tốc độ mặc định cho Pose {i+1}: {speed:.3f}")

            pose = self.adjust_j5_orientation(pose)
            self.arm_group.set_pose_target(pose)
            self.arm_group.set_max_velocity_scaling_factor(speed)
            self.arm_group.set_max_acceleration_scaling_factor(speed / 2)

            rospy.loginfo(f"🚀 Thực thi Pose {i+1}: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")

            for attempt in range(max_attempts):
                try:
                    plan = self.arm_group.plan()
                    if isinstance(plan, tuple) and len(plan) >= 2 and plan[0]:
                        if self.arm_group.go(wait=True):
                            rospy.loginfo(f"✅ Di chuyển đến Pose {i+1} thành công.")
                            break
                        else:
                            rospy.logwarn(f"⚠️ Di chuyển đến Pose {i+1} thất bại, thử lại lần {attempt+1}.")
                    else:
                        rospy.logwarn(f"⚠️ Kế hoạch IK thất bại cho Pose {i+1}, thử lại lần {attempt+1}.")
                except Exception as e:
                    rospy.logerr(f"❌ Lỗi khi thực thi Pose {i+1}: {str(e)}")
                    break
            else:
                rospy.logerr(f"❌ Bỏ qua Pose {i+1}: Không tìm được kế hoạch sau {max_attempts} lần thử.")

        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        self.durations = []
        rospy.loginfo("🏁 Đã hoàn tất toàn bộ waypoints.")

    def adjust_waypoints_callback(self, msg):
        if self.pick_status != "in_progress":
            rospy.loginfo("⏸️ Không có pick đang thực hiện, bỏ qua adjust waypoints.")
            self.adjust_result_pub.publish(String("failed_no_pick"))
            return

        waypoints = msg.poses
        if not waypoints or len(waypoints) != 1:
            rospy.logwarn(f"⚠️ Adjust waypoints không hợp lệ: nhận {len(waypoints)} pose, cần 1 pose.")
            self.adjust_result_pub.publish(String("failed_invalid_count"))
            return

        rospy.loginfo(f"📍 Nhận {len(waypoints)} adjust pose để thực hiện.")
        max_attempts = rospy.get_param("~max_attempts", 5)
        success = True
        result_msg = "success"

        pose = waypoints[0]
        if abs(pose.position.x) > 1.5 or abs(pose.position.y) > 1.5 or pose.position.z < 0.095 or pose.position.z > 0.5:
            rospy.logwarn(f"⚠️ Adjust Pose ngoài không gian làm việc: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
            success = False
            result_msg = "failed_out_of_bounds"
        else:
            is_valid, distance = self.check_pose_distance(pose)
            if not is_valid:
                rospy.logwarn(f"⚠️ Adjust Pose có khoảng cách quá lớn: {distance:.4f}m")
                success = False
                result_msg = "failed_large_displacement"
            else:
                pose = self.adjust_j5_orientation(pose)
                self.arm_group.set_pose_target(pose)
                self.arm_group.set_max_velocity_scaling_factor(self.current_speed)
                self.arm_group.set_max_acceleration_scaling_factor(self.current_speed / 2)

                rospy.sleep(0.5)
                rospy.loginfo(f"🚀 Thực thi Adjust Pose: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")

                for attempt in range(max_attempts):
                    try:
                        plan = self.arm_group.plan()
                        if isinstance(plan, tuple) and len(plan) >= 2 and plan[0]:
                            if self.arm_group.go(wait=True):
                                rospy.loginfo("✅ Di chuyển đến Adjust Pose thành công.")
                                rospy.sleep(0.5)
                                break
                            else:
                                rospy.logwarn(f"⚠️ Di chuyển đến Adjust Pose thất bại, thử lại lần {attempt+1}.")
                        else:
                            rospy.logwarn(f"⚠️ Kế hoạch IK thất bại cho Adjust Pose, thử lại lần {attempt+1}.")
                    except Exception as e:
                        rospy.logerr(f"❌ Lỗi khi thực thi Adjust Pose: {str(e)}")
                        success = False
                        result_msg = f"failed_execution_error: {str(e)}"
                        break
                else:
                    rospy.logwarn("⚠️ Không thể di chuyển đến Adjust Pose sau tối đa lần thử.")
                    success = False
                    result_msg = "failed_no_plan"

        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        self.adjust_result_pub.publish(String(result_msg))
        rospy.loginfo(f"🏁 Đã hoàn tất adjust waypoints: {result_msg}.")

    def pick_waypoints_callback(self, msg):
        self.is_executing_pick = True
        if self.pick_status != "in_progress":
            rospy.loginfo("⏸️ Không có pick đang thực hiện, bỏ qua pick waypoints.")
            self.pick_result_pub.publish(String("failed_no_pick"))
            self.is_executing_pick = False
            return

        waypoints = msg.poses
        if not waypoints or len(waypoints) != 2:
            rospy.logwarn(f"⚠️ Pick waypoints không hợp lệ: nhận {len(waypoints)} pose, cần 2 pose.")
            self.pick_result_pub.publish(String("failed_invalid_count"))
            self.is_executing_pick = False
            return

        rospy.loginfo(f"📍 Nhận {len(waypoints)} pick pose để thực hiện.")
        max_attempts = rospy.get_param("~max_attempts", 5)
        success = True
        result_msg = "success"
        pick_pose1_success = False

        for i, pose in enumerate(waypoints):
            if abs(pose.position.x) > 1.5 or abs(pose.position.y) > 1.5 or pose.position.z < 0.095 or pose.position.z > 0.5:
                rospy.logwarn(f"⚠️ Pick Pose {i+1} ngoài không gian làm việc: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
                success = False
                result_msg = "failed_out_of_bounds"
                continue

            is_valid, distance = self.check_pose_distance(pose)
            if not is_valid:
                rospy.logwarn(f"⚠️ Pick Pose {i+1} có khoảng cách quá lớn: {distance:.4f}m")
                success = False
                result_msg = "failed_large_displacement"
                continue

            pose = self.adjust_j5_orientation(pose)
            self.arm_group.set_pose_target(pose)
            self.arm_group.set_max_velocity_scaling_factor(self.current_speed)
            self.arm_group.set_max_acceleration_scaling_factor(self.current_speed / 2)

            rospy.sleep(0.5)
            rospy.loginfo(f"🚀 Thực thi Pick Pose {i+1}: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
            rospy.loginfo(f"📍 Trạng thái trước Pick Pose {i+1}: {self.arm_group.get_current_pose().pose}")

            pose_success = False
            last_plan = None
            for attempt in range(max_attempts):
                try:
                    plan = self.arm_group.plan()
                    last_plan = plan
                    if isinstance(plan, tuple) and len(plan) >= 2 and plan[0]:
                        if self.arm_group.go(wait=True):
                            rospy.loginfo(f"✅ Di chuyển đến Pick Pose {i+1} thành công.")
                            rospy.loginfo(f"📍 Trạng thái sau Pick Pose {i+1}: {self.arm_group.get_current_pose().pose}")
                            rospy.sleep(0.5)
                            pose_success = True
                            break
                        else:
                            rospy.logwarn(f"⚠️ Di chuyển đến Pick Pose {i+1} thất bại, thử lại lần {attempt+1}.")
                    else:
                        rospy.logwarn(f"⚠️ Kế hoạch IK thất bại cho Pick Pose {i+1}, thử lại lần {attempt+1}.")
                except Exception as e:
                    rospy.logerr(f"❌ Lỗi khi thực thi Pick Pose {i+1}: {str(e)}")
                    success = False
                    result_msg = f"failed_execution_error: {str(e)}"
                    break

            if not pose_success and i == 1:
                rospy.loginfo("🔄 Pick Pose 2 thất bại, thử pose gần đúng từ IK...")
                try:
                    approx_pose = None
                    if last_plan and isinstance(last_plan, tuple) and len(last_plan) >= 2 and last_plan[1].joint_trajectory.points:
                        joint_state = last_plan[1].joint_trajectory.points[-1].positions
                        approx_pose = self.get_pose_from_joint_state(joint_state)
                        if approx_pose:
                            approx_pose = self.adjust_j5_orientation(approx_pose)
                            rospy.loginfo(f"📍 Pose gần đúng từ IK: x={approx_pose.position.x:.3f}, y={approx_pose.position.y:.3f}, z={approx_pose.position.z:.3f}")

                    if not approx_pose:
                        approx_pose = Pose()
                        approx_pose.position.x = pose.position.x
                        approx_pose.position.y = pose.position.y
                        approx_pose.position.z = max(pose.position.z - 0.02, 0.095)
                        approx_pose = self.adjust_j5_orientation(approx_pose)
                        rospy.loginfo(f"📍 Pose gần đúng điều chỉnh: x={approx_pose.position.x:.3f}, y={approx_pose.position.y:.3f}, z={approx_pose.position.z:.3f}")

                    if abs(approx_pose.position.x) > 1.5 or abs(approx_pose.position.y) > 1.5 or approx_pose.position.z > 0.5:
                        rospy.logwarn(f"⚠️ Pose gần đúng ngoài không gian làm việc: x={approx_pose.position.x:.3f}, y={approx_pose.position.y:.3f}, z={approx_pose.position.z:.3f}")
                        success = False
                        result_msg = "failed_approx_out_of_bounds"
                    else:
                        is_valid, distance = self.check_pose_distance(approx_pose)
                        if not is_valid:
                            rospy.logwarn(f"⚠️ Pose gần đúng có khoảng cách quá lớn: {distance:.4f}m")
                            success = False
                            result_msg = "failed_approx_large_displacement"
                        else:
                            rospy.loginfo(f"🚀 Thực thi pose gần đúng: x={approx_pose.position.x:.3f}, y={approx_pose.position.y:.3f}, z={approx_pose.position.z:.3f}")
                            self.arm_group.set_pose_target(approx_pose)
                            for attempt in range(2):
                                try:
                                    plan = self.arm_group.plan()
                                    if isinstance(plan, tuple) and len(plan) >= 2 and plan[0]:
                                        if self.arm_group.go(wait=True):
                                            rospy.loginfo(f"✅ Di chuyển đến pose gần đúng thành công.")
                                            pose_success = True
                                            break
                                        else:
                                            rospy.logwarn(f"⚠️ Di chuyển đến pose gần đúng thất bại, thử lại lần {attempt+1}.")
                                    else:
                                        rospy.logwarn(f"⚠️ Kế hoạch IK thất bại cho pose gần đúng, thử lại lần {attempt+1}.")
                                except Exception as e:
                                    rospy.logerr(f"❌ Lỗi khi thực thi pose gần đúng: {str(e)}")
                                    break

                except Exception as e:
                    rospy.logerr(f"❌ Lỗi khi tạo/thực thi pose gần đúng: {str(e)}")
                    success = False
                    result_msg = f"failed_approx_error: {str(e)}"

            if i == 0 and pose_success:
                pick_pose1_success = True
                rospy.loginfo("✅ Pick Pose 1 thành công, sẵn sàng đóng gripper...")

            if i == 0 and pose_success:
                try:
                    self.hand_group.stop()
                    self.hand_group.clear_pose_targets()
                    self.hand_group.set_joint_value_target({"J7": 0.0, "J8": 0.0})
                    self.hand_group.go(wait=True)
                    rospy.loginfo("🤏 Gripper closed")
                    rospy.sleep(1.0)
                except Exception as e:
                    rospy.logwarn(f"⚠️ Lỗi đóng gripper: {str(e)}")
                    success = False
                    result_msg = f"failed_gripper_error: {str(e)}"
            elif i == 1 and pick_pose1_success:
                try:
                    self.hand_group.stop()
                    self.hand_group.clear_pose_targets()
                    self.hand_group.set_joint_value_target({"J7": -0.0073, "J8": -0.0073})
                    self.hand_group.go(wait=True)
                    rospy.loginfo("🤏 Gripper opened")
                    rospy.sleep(1.0)
                except Exception as e:
                    rospy.logwarn(f"⚠️ Lỗi mở gripper: {str(e)}")
                    success = False
                    result_msg = f"failed_gripper_error: {str(e)}"

            if not pose_success:
                rospy.logwarn(f"⚠️ Pick Pose {i+1} hoặc pose gần đúng thất bại, vẫn tiếp tục với các bước tiếp theo.")

        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        self.pick_result_pub.publish(String(result_msg))
        rospy.loginfo(f"🏁 Đã hoàn tất pick waypoints: {result_msg}.")
        self.is_executing_pick = False

if __name__ == "__main__":
    try:
        node = UltimateRobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("🏁 Ultimate Robot Controller shutdown.")
