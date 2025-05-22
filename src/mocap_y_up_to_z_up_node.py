#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations as tft

class PoseToOdomConverter:
    def __init__(self, in_topic, out_topic, object_type):
        self.object_type = object_type
        self.pub = rospy.Publisher(out_topic, Odometry, queue_size=10)
        self.last_time = None
        self.last_pos = None
        rospy.Subscriber(in_topic, PoseStamped, self.pose_callback)
        print(f'[PoseConverter] Converting for {object_type}: {in_topic} -> {out_topic}')

    def pose_callback(self, msg: PoseStamped):
        # 1) Thời gian hiện tại
        cur_time = msg.header.stamp

        # 2) Chuyển toạ độ y-up → z-up
        x_old, y_old, z_old = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        x_new = x_old
        y_new = -z_old
        z_new = y_old

        # 3) Chuyển orientation
        q_old = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        q_rot = tft.quaternion_from_euler(math.pi/2, 0.0, 0.0)
        q_new = tft.quaternion_multiply(q_rot, q_old)

        # 4) Tạo Odometry message
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.header.stamp = cur_time
        odom.pose.pose.position.x = x_new
        odom.pose.pose.position.y = y_new
        odom.pose.pose.position.z = z_new
        odom.pose.pose.orientation.x = q_new[0]
        odom.pose.pose.orientation.y = q_new[1]
        odom.pose.pose.orientation.z = q_new[2]
        odom.pose.pose.orientation.w = q_new[3]

        # 5) Tính vận tốc tuyến tính
        pos = np.array([x_new, y_new, z_new])
        if self.last_time is not None:
            dt = (cur_time - self.last_time).to_sec()
            if dt > 0:
                vel = (pos - self.last_pos) / dt
            else:
                vel = np.zeros(3)
        else:
            vel = np.zeros(3)

        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.y = vel[1]
        odom.twist.twist.linear.z = vel[2]

        # 6) (Tuỳ chọn) Tính vận tốc góc (angular) nếu cần:
        #    Bạn có thể dùng quaternion delta: q_delta = q_new * q_old^{-1},
        #    rồi chuyển sang trục-ghi-độ để lấy omega = angle/dt.

        # 7) Cập nhật state cho lần kế tiếp
        self.last_time = cur_time
        self.last_pos = pos

        # 8) Publish
        self.pub.publish(odom)


if __name__ == '__main__':
    rospy.init_node('mocap_y_up_to_z_up_with_twist')

    # Lấy param
    robot_in  = rospy.get_param('robot_pose_y_up_topic')
    robot_out = rospy.get_param('robot_pose_z_up_topic')
    obj_in    = rospy.get_param('object_pose_y_up_topic')
    obj_out   = rospy.get_param('object_pose_z_up_topic')

    # Khởi tạo converter cho cả robot và flying_object
    PoseToOdomConverter(robot_in, robot_out, "robot")
    PoseToOdomConverter(obj_in, obj_out,    "flying_object")
    rospy.spin()
