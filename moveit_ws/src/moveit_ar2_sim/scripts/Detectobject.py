#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import Point, TransformStamped
from cv_bridge import CvBridge
import tf2_ros
import tf.transformations as tf_trans

class CameraObjectNode:
    def __init__(self):
        rospy.init_node("camera_object_node", anonymous=True)
        self.bridge = CvBridge()

        # Publishers
        self.object_detected_pub = rospy.Publisher("/object_detected", Bool, queue_size=1)
        self.object_position_pub = rospy.Publisher("/object_position", Point, queue_size=1)
        self.object_pixel_pub = rospy.Publisher("/object_pixel", Float64MultiArray, queue_size=1)
        self.object_image_pub = rospy.Publisher("/processed_image", Image, queue_size=1)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribers
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)
        self.enable_sub = rospy.Subscriber("/enable_object_detection", Bool, self.enable_callback)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Parameters
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 480)
        self.rect_width = rospy.get_param("~rect_width", 100)
        self.rect_height = rospy.get_param("~rect_height", 100)
        self.min_area = rospy.get_param("~min_area", 50)
        self.margin = rospy.get_param("~margin", 10)
        self.plane_z = rospy.get_param("~plane_z", 0.015)
        self.f = rospy.get_param("~focal_length", 640 / (2 * np.tan(0.698 / 2)))
        self.lower_green = np.array(rospy.get_param("~lower_green", [30, 50, 50]))
        self.upper_green = np.array(rospy.get_param("~upper_green", [90, 255, 255]))
        self.lower_blue = np.array(rospy.get_param("~lower_blue", [90, 60, 60]))
        self.upper_blue = np.array(rospy.get_param("~upper_blue", [150, 255, 255]))
        self.no_object_threshold = rospy.get_param("~no_object_threshold", 30)
        self.processing_timeout = rospy.get_param("~processing_timeout", 3.0)

        self.camera_matrix = np.array([[self.f, 0, self.image_width/2], [0, self.f, self.image_height/2], [0, 0, 1]])

        # Kalman filter
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.01
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5
        self.kalman.statePre = np.array([[self.image_width/2], [self.image_height/2], [0], [0]], np.float32)

        # Enable/Disable state
        self.is_enabled = False  # Mặc định bật để tương thích hành vi hiện tại
        self.no_object_counter = 0
        self.last_detection_time = None

        rospy.loginfo("📷 CameraObjectNode khởi động thành công!")

    def enable_callback(self, msg):
        """Callback for /enable_object_detection topic."""
        self.is_enabled = msg.data
        status = "bật" if self.is_enabled else "tắt"
        rospy.loginfo(f"📡 Trạng thái publish: {status}")

    def is_object_fully_in_frame(self, contour):
        x, y, w, h = cv2.boundingRect(contour)
        if (x <= self.margin or y <= self.margin or
            (x + w) >= (self.image_width - self.margin) or
            (y + h) >= (self.image_height - self.margin)):
            return False
        return True

    def detect_object_in_frame(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_green = cv2.erode(mask_green, None, iterations=2)
        mask_green = cv2.dilate(mask_green, None, iterations=2)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask_blue = cv2.erode(mask_blue, None, iterations=2)
        mask_blue = cv2.dilate(mask_blue, None, iterations=2)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours = contours_green + contours_blue
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)
            if area > self.min_area and self.is_object_fully_in_frame(max_contour):
                M = cv2.moments(max_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
                    self.kalman.correct(measurement)
                    prediction = self.kalman.predict()
                    cx_filtered, cy_filtered = int(prediction[0]), int(prediction[1])
                    rospy.loginfo(f"🔍 Tâm trước Kalman: ({cx}, {cy}), Sau Kalman: ({cx_filtered}, {cy_filtered})")
                    return [cx_filtered, cy_filtered], max_contour

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 30, 100)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_area and self.is_object_fully_in_frame(contour):
                    peri = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                    if len(approx) == 4:
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
                            self.kalman.correct(measurement)
                            prediction = self.kalman.predict()
                            cx_filtered, cy_filtered = int(prediction[0]), int(prediction[1])
                            rospy.loginfo(f"🔍 Tâm trước Kalman (phát hiện cạnh): ({cx}, {cy}), Sau Kalman: ({cx_filtered}, {cy_filtered})")
                            return [cx_filtered, cy_filtered], contour
        return None, None

    def rectangles_intersect(self, obj_rect):
        x1, y1, x2, y2 = obj_rect
        target_x1 = self.image_width/2 - self.rect_width/2
        target_y1 = self.image_height/2 - self.rect_height/2
        target_x2 = self.image_width/2 + self.rect_width/2
        target_y2 = self.image_height/2 + self.rect_height/2
        return not (x2 < target_x1 or x1 > target_x2 or y2 < target_y1 or y1 > target_y2)

    def pixel_to_world(self, u, v, Z, msg):
        try:
            trans = self.tf_buffer.lookup_transform('world', 'camera_link', rospy.Time(0), rospy.Duration(0.5))
            t = trans.transform.translation
            q = trans.transform.rotation
            rotation_matrix = tf_trans.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            camera_pos = np.array([t.x, t.y, t.z])

            uv_point = np.array([[u, v, 1]], dtype=np.float32).T
            inv_camera_matrix = np.linalg.inv(self.camera_matrix)
            camera_point = inv_camera_matrix @ uv_point
            camera_point = camera_point / camera_point[2]
            point_camera = np.array([camera_point[0] * Z, camera_point[1] * Z, Z])

            world_point = camera_pos + rotation_matrix @ point_camera
            rospy.loginfo(f"🔍 Tọa độ thế giới: {world_point}")
            return (world_point[0], world_point[1], world_point[2])
        except Exception as e:
            rospy.logerr(f"❌ Lỗi TF: {str(e)}")
            return None

    def publish_object_tf(self, world_pos):
        try:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "object_frame"
            t.transform.translation.x = world_pos[0]
            t.transform.translation.y = world_pos[1]
            t.transform.translation.z = world_pos[2]
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
            rospy.loginfo(f"📡 Published TF world -> object_frame: ({world_pos[0]}, {world_pos[1]}, {world_pos[2]})")
        except Exception as e:
            rospy.logerr(f"❌ Lỗi publish TF: {str(e)}")

    def image_callback(self, msg):
        if not hasattr(self, 'last_detection_time'):
            self.last_detection_time = None
            rospy.logwarn("⚠️ last_detection_time chưa được khởi tạo, đặt lại thành None.")

        current_time = rospy.get_time()
        if self.last_detection_time is not None:
            elapsed_time = current_time - self.last_detection_time
            if elapsed_time < self.processing_timeout:
                rospy.loginfo(f"⏳ Đang chờ {self.processing_timeout - elapsed_time:.1f} giây sau khi phát hiện vật thể.")
                return
            else:
                self.last_detection_time = None

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"❌ Lỗi chuyển ảnh: {str(e)}")
            return

        # Vẽ khung mục tiêu
        target_x1 = self.image_width/2 - self.rect_width/2
        target_y1 = self.image_height/2 - self.rect_height/2
        target_x2 = self.image_width/2 + self.rect_width/2
        target_y2 = self.image_height/2 + self.rect_height/2
        cv2.rectangle(cv_image, (int(target_x1), int(target_y1)), (int(target_x2), int(target_y2)), (255, 0, 0), 2)

        curr_center, max_contour = self.detect_object_in_frame(cv_image)
        if curr_center is not None and max_contour is not None:
            cx_filtered, cy_filtered = curr_center
            x, y, w, h = cv2.boundingRect(max_contour)
            obj_rect = [x, y, x + w, y + h]

            # Vẽ khung đối tượng và tâm
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(cv_image, (cx_filtered, cy_filtered), 5, (0, 0, 255), -1)

            # Kiểm tra giao nhau
            if self.rectangles_intersect(obj_rect):
                rospy.loginfo("✅ Khung đối tượng giao nhau với khung mục tiêu.")
            else:
                rospy.loginfo("⚠️ Khung đối tượng không giao nhau với khung mục tiêu.")

            Z = self.plane_z
            world_pos = self.pixel_to_world(cx_filtered, cy_filtered, Z, msg)
            if world_pos and (abs(world_pos[0]) <= 1.5 and abs(world_pos[1]) <= 1.5 and 0 <= world_pos[2] <= 0.5):
                self.no_object_counter = 0
                self.last_detection_time = current_time
                rospy.loginfo(f"✅ Phát hiện đối tượng tại pixel: ({cx_filtered}, {cy_filtered}), world: {world_pos}")
                
                # Chỉ publish khi được bật
                if self.is_enabled:
                    pixel_msg = Float64MultiArray()
                    pixel_msg.data = obj_rect
                    self.object_pixel_pub.publish(pixel_msg)
                    world_msg = Point(x=world_pos[0], y=world_pos[1], z=world_pos[2])
                    self.object_position_pub.publish(world_msg)
                    self.object_detected_pub.publish(Bool(data=True))
                    self.publish_object_tf(world_pos)
                else:
                    rospy.loginfo("⏸ Publish bị tắt, bỏ qua xuất bản thông điệp.")
            else:
                rospy.logwarn(f"⚠️ Tọa độ thế giới không hợp lý: {world_pos}")
        else:
            self.no_object_counter += 1
            if self.no_object_counter >= self.no_object_threshold:
                self.no_object_counter = 0
                if self.is_enabled:
                    self.object_detected_pub.publish(Bool(data=False))
                    rospy.loginfo("⚠️ Không phát hiện đối tượng.")
                else:
                    rospy.loginfo("⏸ Publish bị tắt, bỏ qua thông báo không phát hiện đối tượng.")

        # Hiển thị và xuất bản hình ảnh (chỉ khi được bật)
        cv2.imshow("Detection", cv_image)
        cv2.waitKey(1)
        if self.is_enabled:
            try:
                processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                self.object_image_pub.publish(processed_image_msg)
            except Exception as e:
                rospy.logerr(f"❌ Lỗi xuất bản ảnh: {str(e)}")
        else:
            rospy.loginfo("⏸ Publish bị tắt, bỏ qua xuất bản ảnh xử lý.")

if __name__ == "__main__":
    try:
        node = CameraObjectNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
