#!/usr/bin/env python3

import rospy
import tf

def get_ee_position():
    rospy.init_node("ee_tf_listener", anonymous=True)
    listener = tf.TransformListener()

    ee_frame = "Claw_mid"  # End-effector của robot
    base_frame = "Base"  # Frame gốc

    rospy.loginfo("⏳ Chờ TF cập nhật...")
    listener.waitForTransform(base_frame, ee_frame, rospy.Time(0), rospy.Duration(3.0))

    rate = rospy.Rate(10)  # Lặp 10 lần/s
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(base_frame, ee_frame, rospy.Time(0))
            rospy.loginfo(f"📍 End-Effector ({ee_frame}) tọa độ: x={trans[0]:.3f}, y={trans[2]:.3f}, z={trans[1]:.3f}")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("⚠️ Không lấy được tọa độ EE từ TF!")

        rate.sleep()

if __name__ == "__main__":
    try:
        get_ee_position()
    except rospy.ROSInterruptException:
        pass

