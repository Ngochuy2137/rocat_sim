#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft

class PoseConverter:
    def __init__(self):
        # --- 1) Đọc tham số (private) hoặc dùng mặc định ---
        self.robot_in_topic  = rospy.get_param('robot_pose_y_up_topic')
        self.robot_out_topic = rospy.get_param('robot_pose_z_up_topic')
        self.obj_in_topic    = rospy.get_param('object_pose_y_up_topic')
        self.obj_out_topic   = rospy.get_param('object_pose_z_up_topic')

        # --- 2) Tạo 2 publisher ---
        self.pub_robot = rospy.Publisher(self.robot_out_topic,  PoseStamped, queue_size=1)
        self.pub_obj   = rospy.Publisher(self.obj_out_topic,    PoseStamped, queue_size=1)

        # --- 3) Tạo 2 subscriber và gán callback chung, truyền kèm publisher tương ứng ---
        rospy.Subscriber(self.robot_in_topic, PoseStamped, self.callback, callback_args=self.pub_robot, queue_size=1)
        rospy.Subscriber(self.obj_in_topic,   PoseStamped, self.callback, callback_args=self.pub_obj,   queue_size=1)

        rospy.loginfo(f"[PoseConverter] Subscribed: robot⇨{self.robot_in_topic}, object⇨{self.obj_in_topic}")
        rospy.loginfo(f"[PoseConverter] Publishing z-up: robot⇨{self.robot_out_topic}, object⇨{self.obj_out_topic}")

    def callback(self, msg: PoseStamped, pub):
        # --- chuyển vị trí ---
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        new_pos = ( x,     -z,     y )

        # --- chuyển quaternion (xoay +90° quanh trục x) ---
        q_old = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        q_rot = tft.quaternion_from_euler(math.pi/2, 0.0, 0.0)
        q_new = tft.quaternion_multiply(q_rot, q_old)

        # --- đóng gói PoseStamped mới và publish ---
        out = PoseStamped()
        out.header        = msg.header
        out.pose.position.x = new_pos[0]
        out.pose.position.y = new_pos[1]
        out.pose.position.z = new_pos[2]
        out.pose.orientation.x = q_new[0]
        out.pose.orientation.y = q_new[1]
        out.pose.orientation.z = q_new[2]
        out.pose.orientation.w = q_new[3]

        pub.publish(out)

if __name__ == '__main__':
    rospy.init_node('mocap_y_up_to_z_up_node')
    converter = PoseConverter()
    rospy.spin()
