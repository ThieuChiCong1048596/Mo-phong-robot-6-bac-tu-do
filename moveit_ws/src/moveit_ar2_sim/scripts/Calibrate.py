#!/usr/bin/env python3
import rospy
import moveit_commander
import numpy as np
from std_msgs.msg import Bool, Float64MultiArray, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
import cv2
from cv_bridge import CvBridge
import time

print("Starting CameraAdjustNode...")

class CameraAdjustNode:
    def __init__(self):
        try:
            rospy.init_node("camera_adjust_node", anonymous=True)
            rospy.loginfo("Initializing CameraAdjustNode...")

            moveit_commander.roscpp_initialize([])
            self.robot = moveit_commander.RobotCommander()
            self.arm_group = moveit_commander.MoveGroupCommander("arm_group")
            self.hand_group = moveit_commander.MoveGroupCommander("hand_ee")
            rospy.loginfo("MoveIt initialized successfully")

            self.image_width = rospy.get_param("~image_width", 640)
            self.image_height = rospy.get_param("~image_height", 480)
            self.rect_width = rospy.get_param("~rect_width", 100)
            self.rect_height = rospy.get_param("~rect_height", 100)
            self.center_tolerance = rospy.get_param("~center_tolerance", 50)
            self.pixel_to_meter = rospy.get_param("~pixel_to_meter", 0.001)
            self.max_adjust_distance = rospy.get_param("~max_adjust_distance", 0.005)  # Giới hạn di chuyển
            self.height_scale = rospy.get_param("~height_scale", 0.5)
            self.min_lift_height = rospy.get_param("~min_lift_height", 0.1)
            self.base_height = rospy.get_param("~base_height", 0.0)

            self.object_detected = False
            self.object_pixel = None
            self.depth_image = None
            self.current_pose = None
            self.quat_fixed = [0.0, 0.0, 0.7071, 0.7071]
            self.adjust_result = None
            self.bridge = CvBridge()

            self.pause_pub = rospy.Publisher("/pause_waypoints", Bool, queue_size=10)
            self.status_pub = rospy.Publisher("/pick_status", String, queue_size=10)
            self.pick_waypoints_pub = rospy.Publisher("/pick_waypoints", PoseArray, queue_size=10)
            self.adjust_waypoints_pub = rospy.Publisher("/adjust_waypoints", PoseArray, queue_size=10)

            rospy.Subscriber("/object_detected", Bool, self.object_detected_callback)
            rospy.Subscriber("/object_pixel", Float64MultiArray, self.object_pixel_callback)
            rospy.Subscriber("/depth/image_raw", Image, self.depth_image_callback)
            rospy.Subscriber("/pick_result", String, self.pick_result_callback)
            rospy.Subscriber("/adjust_result", String, self.adjust_result_callback)

            self.pick_result = None

            try:
                self.arm_group.set_named_target("home")
                self.arm_group.go(wait=True, timeout=15.0)
                rospy.loginfo("✅ Moved to home position")
            except Exception as e:
                rospy.logwarn(f"⚠️ Lỗi khi di chuyển về home: {str(e)}")

            rospy.loginfo("✅ CameraAdjustNode ready!")
        except Exception as e:
            rospy.logerr(f"❌ Lỗi khởi tạo CameraAdjustNode: {str(e)}")
            raise

    def object_detected_callback(self, msg):
        try:
            self.object_detected = msg.data
            rospy.loginfo(f"📷 Object detected: {self.object_detected}")
            if self.object_detected:
                self.pause_pub.publish(Bool(True))
                self.status_pub.publish(String("in_progress"))
                try:
                    self.current_pose = self.arm_group.get_current_pose().pose
                    rospy.loginfo(f"📍 Current pose: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}, z={self.current_pose.position.z:.3f}")
                except Exception as e:
                    rospy.logwarn(f"⚠️ Lỗi khi lấy current pose: {str(e)}")
                    self.current_pose = None
            else:
                self.pause_pub.publish(Bool(False))
                self.status_pub.publish(String("idle"))
                self.current_pose = None
                self.object_pixel = None
                self.depth_image = None
        except Exception as e:
            rospy.logerr(f"❌ Lỗi trong object_detected_callback: {str(e)}")

    def object_pixel_callback(self, msg):
        try:
            x1, y1, x2, y2 = msg.data
            if (x1 < 0 or x2 > self.image_width or y2 < 0 or y2 > self.image_height or
                x2 <= x1 or y2 <= y1):
                rospy.logwarn(f"⚠️ Object pixel không hợp lệ: ({x1:.1f}, {y1:.1f})-({x2:.1f}, {y2:.1f})")
            else:
                self.object_pixel = msg.data
                rospy.loginfo(f"📷 Object pixel: ({x1:.1f}, {y1:.1f})-({x2:.1f}, {y2:.1f})")
        except Exception as e:
            rospy.logerr(f"❌ Lỗi trong object_pixel_callback: {str(e)}")

    def depth_image_callback(self, msg):
        try:
            if msg.encoding not in ["32FC1", "16UC1"]:
                rospy.logerr(f"❌ Encoding không hỗ trợ: {msg.encoding}")
                return
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
            if msg.encoding == "16UC1":
                self.depth_image = self.depth_image.astype(np.float32) / 1000.0
            rospy.loginfo(f"📷 Depth image received: {self.depth_image.shape}")
        except Exception as e:
            rospy.logerr(f"❌ Lỗi trong depth_image_callback: {str(e)}")

    def pick_result_callback(self, msg):
        try:
            self.pick_result = msg.data
            rospy.loginfo(f"📬 Pick result: {self.pick_result}")
            if self.pick_result in ["success", "failed"]:
                self.status_pub.publish(String(self.pick_result))
                if self.pick_result == "success":
                    self.pause_pub.publish(Bool(False))
                    self.object_detected = False
                    self.object_pixel = None
                    self.depth_image = None
                    self.current_pose = None
        except Exception as e:
            rospy.logerr(f"❌ Lỗi trong pick_result_callback: {str(e)}")

    def adjust_result_callback(self, msg):
        try:
            self.adjust_result = msg.data
            rospy.loginfo(f"📬 Adjust result: {self.adjust_result}")
        except Exception as e:
            rospy.logerr(f"❌ Lỗi trong adjust_result_callback: {str(e)}")

    def adjust_orientation(self, pose):
        try:
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = self.quat_fixed
            return pose
        except Exception as e:
            rospy.logerr(f"❌ Lỗi khi gán quaternion: {str(e)}")
            return pose

    def adjust_to_center(self):
        try:
            if not self.object_pixel or not self.current_pose:
                rospy.logwarn("⚠️ Thiếu object_pixel hoặc current_pose để căn chỉnh.")
                return False

            x1, y1, x2, y2 = self.object_pixel
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            image_center_x = self.image_width / 2
            image_center_y = self.image_height / 2

            delta_x = center_x - image_center_x
            delta_y = center_y - image_center_y

            if abs(delta_x) <= self.center_tolerance and abs(delta_y) <= self.center_tolerance:
                rospy.loginfo("✅ Vật thể đã ở trung tâm, không cần điều chỉnh.")
                return True

            delta_x_m = delta_x * self.pixel_to_meter
            delta_y_m = -delta_y * self.pixel_to_meter
            delta_x_m = np.clip(delta_x_m, -self.max_adjust_distance, self.max_adjust_distance)
            delta_y_m = np.clip(delta_y_m, -self.max_adjust_distance, self.max_adjust_distance)

            rospy.loginfo(f"📸 Sai lệch tâm: center_x={center_x:.2f}, center_y={center_y:.2f}, "
                         f"delta_x={delta_x:.2f}, delta_y={delta_y:.2f}, "
                         f"delta_x_m={delta_x_m:.4f}m, delta_y_m={delta_y_m:.4f}m")

            adjust_pose = Pose()
            adjust_pose.position.x = self.current_pose.position.x + delta_x_m
            adjust_pose.position.y = self.current_pose.position.y + delta_y_m
            adjust_pose.position.z = self.current_pose.position.z
            adjust_pose = self.adjust_orientation(adjust_pose)

            poses = PoseArray()
            poses.header.frame_id = "world"
            poses.header.stamp = rospy.Time.now()
            poses.poses.append(adjust_pose)

            rospy.loginfo(f"📤 Sending adjust waypoints: x={adjust_pose.position.x:.3f}, "
                         f"y={adjust_pose.position.y:.3f}, z={adjust_pose.position.z:.3f}")
            self.adjust_waypoints_pub.publish(poses)

            timeout = 15.0
            start_time = rospy.get_time()
            self.adjust_result = None
            rate = rospy.Rate(10)
            while rospy.get_time() - start_time < timeout:
                if self.adjust_result in ["success", "failed"]:
                    break
                rate.sleep()

            if self.adjust_result == "success":
                rospy.loginfo("✅ Đã căn chỉnh tâm thành công.")
                try:
                    self.current_pose = self.arm_group.get_current_pose().pose
                    rospy.loginfo(f"📍 Updated current pose: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}, z={self.current_pose.position.z:.3f}")
                except Exception as e:
                    rospy.logwarn(f"⚠️ Lỗi khi cập nhật current pose: {str(e)}")
                    self.current_pose = None
                return True
            else:
                rospy.logwarn(f"⚠️ Không thể căn chỉnh tâm ({self.adjust_result}), vẫn tiếp tục gắp.")
                return False
        except Exception as e:
            rospy.logerr(f"❌ Lỗi khi căn chỉnh tâm: {str(e)}")
            return False

    def generate_pick_posearray(self):
        try:
            if not self.current_pose or not self.object_pixel:
                rospy.logwarn("⚠️ Thiếu current_pose hoặc object_pixel.")
                return None

            x, y, z = self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z
            x1, y1, x2, y2 = self.object_pixel

            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            if self.depth_image is None:
                rospy.logwarn("⚠️ Thiếu depth_image, sử dụng z=0.1")
                z_descend = 0.1
            else:
                if 0 <= v < self.depth_image.shape[0] and 0 <= u < self.depth_image.shape[1]:
                    depth = self.depth_image[v, u]
                    if np.isnan(depth) or depth <= 0 or depth > 0.5:
                        rospy.logwarn(f"⚠️ Giá trị độ sâu không hợp lệ: {depth:.3f}, sử dụng z=0.1")
                        z_descend = 0.1
                    else:
                        z_descend = depth - self.base_height
                        if z_descend < 0.095 or z_descend > 0.5:
                            rospy.logwarn(f"⚠️ Độ cao hạ xuống ngoài phạm vi: {z_descend:.3f}, sử dụng z=0.1")
                            z_descend = 0.1
                else:
                    rospy.logwarn(f"⚠️ Tọa độ pixel ({u}, {v}) ngoài ảnh, sử dụng z=0.1")
                    z_descend = 0.1

            pixel_area = (x2 - x1) * (y2 - y1)
            area_m2 = pixel_area * (self.pixel_to_meter ** 2)
            estimated_height = self.height_scale * np.sqrt(area_m2)
            lift_height = max(estimated_height, self.min_lift_height)
            lift_z = z + lift_height

            rospy.loginfo(f"📏 Depth: {depth if self.depth_image is not None else 'N/A'}, z_descend: {z_descend:.3f}, "
                         f"Pixel area: {pixel_area:.2f}, Area (m²): {area_m2:.6f}, "
                         f"Estimated height: {estimated_height:.3f}, Lift z: {lift_z:.3f}")

            poses = PoseArray()
            poses.header.frame_id = "world"
            poses.header.stamp = rospy.Time.now()

            descend = Pose()
            descend.position.x = x
            descend.position.y = y
            descend.position.z = z_descend
            descend = self.adjust_orientation(descend)
            poses.poses.append(descend)

            lift = Pose()
            lift.position.x = x
            lift.position.y = y
            lift.position.z = lift_z
            lift = self.adjust_orientation(lift)
            poses.poses.append(lift)

            rospy.loginfo(f"📍 Generated poses: descend=({x:.3f}, {y:.3f}, {z_descend:.3f}), lift=({x:.3f}, {y:.3f}, {lift_z:.3f})")
            return poses
        except Exception as e:
            rospy.logerr(f"❌ Lỗi trong generate_pick_posearray: {str(e)}")
            return None

    def main_loop(self):
        try:
            rate = rospy.Rate(5)
            rospy.loginfo("Starting main loop...")
            max_pick_attempts = 5
            pick_attempt_count = 0

            while not rospy.is_shutdown():
                if self.object_detected and self.object_pixel and self.current_pose:
                    rospy.loginfo("⏳ Xác nhận vật thể trước khi gắp...")
                    start_time = rospy.get_time()
                    while rospy.get_time() - start_time < 1.0:
                        if not self.object_detected:
                            rospy.logwarn("⚠️ Mất phát hiện vật thể, hủy gắp.")
                            self.status_pub.publish(String("failed"))
                            self.pause_pub.publish(Bool(False))
                            self.object_detected = False
                            self.object_pixel = None
                            self.depth_image = None
                            self.current_pose = None
                            pick_attempt_count = 0
                            break
                        rate.sleep()
                    else:
                        rospy.loginfo("🔵 Vật thể ổn định, bắt đầu căn chỉnh tâm...")
                        self.adjust_to_center()
                        pick_attempt_count += 1

                        poses = self.generate_pick_posearray()
                        if poses:
                            rospy.sleep(1.0)
                            self.pick_waypoints_pub.publish(poses)
                            rospy.loginfo("📤 Sent pick waypoints to UltimateRobotController")

                            timeout = 30.0
                            start_time = rospy.get_time()
                            self.pick_result = None
                            while rospy.get_time() - start_time < timeout:
                                if self.pick_result in ["success", "failed"]:
                                    break
                                rate.sleep()
                            if self.pick_result == "success":
                                rospy.loginfo("✅ Pick-and-place completed successfully")
                                pick_attempt_count = 0
                            else:
                                rospy.logwarn(f"⚠️ Pick-and-place failed (attempt {pick_attempt_count}/{max_pick_attempts})")
                                if pick_attempt_count >= max_pick_attempts:
                                    rospy.logwarn("⚠️ Đã thử tối đa số lần, gửi failed.")
                                    self.status_pub.publish(String("failed"))
                                    self.pause_pub.publish(Bool(False))
                                    self.object_detected = False
                                    self.object_pixel = None
                                    self.depth_image = None
                                    self.current_pose = None
                                    pick_attempt_count = 0
                                else:
                                    rospy.loginfo("🔄 Thử lại pick-and-place...")
                                    try:
                                        self.current_pose = self.arm_group.get_current_pose().pose
                                        rospy.loginfo(f"📍 Updated current pose: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}, z={self.current_pose.position.z:.3f}")
                                    except Exception as e:
                                        rospy.logwarn(f"⚠️ Lỗi khi cập nhật current pose: {str(e)}")
                                        self.current_pose = None
                        else:
                            rospy.logwarn(f"⚠️ Failed to generate pick waypoints (attempt {pick_attempt_count}/{max_pick_attempts})")
                            if pick_attempt_count >= max_pick_attempts:
                                rospy.logwarn("⚠️ Đã thử tối đa số lần, gửi failed.")
                                self.status_pub.publish(String("failed"))
                                self.pause_pub.publish(Bool(False))
                                self.object_detected = False
                                self.object_pixel = None
                                self.depth_image = None
                                self.current_pose = None
                                pick_attempt_count = 0
                            else:
                                rospy.loginfo("🔄 Thử lại pick-and-place...")
                                try:
                                    self.current_pose = self.arm_group.get_current_pose().pose
                                    rospy.loginfo(f"📍 Updated current pose: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}, z={self.current_pose.position.z:.3f}")
                                except Exception as e:
                                    rospy.logwarn(f"⚠️ Lỗi khi cập nhật current pose: {str(e)}")
                                    self.current_pose = None
                else:
                    if self.object_detected:
                        rospy.loginfo("⏳ Đang chờ current pose hoặc pixel...")
                        try:
                            self.current_pose = self.arm_group.get_current_pose().pose
                            rospy.loginfo(f"📍 Current pose: x={self.current_pose.position.x:.3f}, y={self.current_pose.position.y:.3f}, z={self.current_pose.position.z:.3f}")
                        except Exception as e:
                            rospy.logwarn(f"⚠️ Lỗi khi lấy current pose: {str(e)}")
                            self.current_pose = None
                    rate.sleep()
        except Exception as e:
            rospy.logerr(f"❌ Lỗi trong main_loop: {str(e)}")
            self.status_pub.publish(String("failed"))
            self.pause_pub.publish(Bool(False))

    def shutdown(self):
        self.pause_pub.publish(Bool(False))
        self.status_pub.publish(String("idle"))
        moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        print("Creating CameraAdjustNode instance...")
        node = CameraAdjustNode()
        print("Running main loop...")
        node.main_loop()
    except rospy.ROSInterruptException:
        print("Node interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
    finally:
        print("Shutting down MoveIt...")
        node.shutdown()
        print("Node shutdown complete")
