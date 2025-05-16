#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations as tft

def pose_callback(msg: PoseStamped, args):
    object_type, pub = args  # unpack tuple

    # --- Chuyển đổi vị trí ---
    x_old, y_old, z_old = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    x_new = x_old
    y_new = -z_old
    z_new = y_old

    # --- Chuyển orientation ---
    q_old = [
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    ]
    q_rot = tft.quaternion_from_euler(math.pi/2, 0.0, 0.0)
    q_new = tft.quaternion_multiply(q_rot, q_old)

    if object_type == "robot":
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.header.frame_id = "world"  # hoặc "world" nếu cần
        odom_msg.pose.pose.position.x = x_new
        odom_msg.pose.pose.position.y = y_new
        odom_msg.pose.pose.position.z = z_new
        odom_msg.pose.pose.orientation.x = q_new[0]
        odom_msg.pose.pose.orientation.y = q_new[1]
        odom_msg.pose.pose.orientation.z = q_new[2]
        odom_msg.pose.pose.orientation.w = q_new[3]
        pub.publish(odom_msg)

    elif object_type == "flying_object":
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position.x = x_new
        pose_msg.pose.position.y = y_new
        pose_msg.pose.position.z = z_new
        pose_msg.pose.orientation.x = q_new[0]
        pose_msg.pose.orientation.y = q_new[1]
        pose_msg.pose.orientation.z = q_new[2]
        pose_msg.pose.orientation.w = q_new[3]
        pub.publish(pose_msg)

# ----------------------
if __name__ == '__main__':
    rospy.init_node('mocap_y_up_to_z_up_node')

    robot_in_topic  = rospy.get_param('robot_pose_y_up_topic')
    robot_out_topic = rospy.get_param('robot_pose_z_up_topic')
    obj_in_topic    = rospy.get_param('object_pose_y_up_topic')
    obj_out_topic   = rospy.get_param('object_pose_z_up_topic')
    
    pub_robot = rospy.Publisher(robot_out_topic, Odometry, queue_size=10)
    pub_object = rospy.Publisher(obj_out_topic, PoseStamped, queue_size=10)

    rospy.Subscriber(robot_in_topic,
                     PoseStamped,
                     pose_callback,
                     callback_args=("robot", pub_robot))

    rospy.Subscriber(obj_in_topic,
                     PoseStamped,
                     pose_callback,
                     callback_args=("flying_object", pub_object))

    rospy.loginfo("[PoseConverter] Running... robot → Odometry | flying object → PoseStamped")
    rospy.spin()
