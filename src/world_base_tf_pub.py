#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

prev_stamp = None

def odom_cb(msg):
    global prev_stamp
    if prev_stamp == msg.header.stamp:
        return  # tránh gửi trùng timestamp
    prev_stamp = msg.header.stamp

    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "world"
    t.child_frame_id = "base"

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf_broadcaster")
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber("/unitree_go1/pose", Odometry, odom_cb, queue_size=10)
    rospy.spin()
