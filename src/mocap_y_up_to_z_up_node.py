#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations as tft

# thêm import cho tf2
import tf2_ros
from geometry_msgs.msg import TransformStamped

class PoseToOdomConverter:
    def __init__(self, in_topic, out_topic, object_type, publish_tf=False):
        self.object_type = object_type  # "robot" hoặc "flying_object"
        self.pub = rospy.Publisher(out_topic, Odometry, queue_size=10)

        # bật/tắt TF broadcasting theo param
        self.publish_tf = publish_tf
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.last_time = None
        self.last_pos = None
        rospy.Subscriber(in_topic, PoseStamped, self.pose_callback)
        print(f'[PoseConverter] Converting for {object_type}: {in_topic} -> {out_topic}')
        if object_type == "flying_object":
            self.catching_height_real = rospy.get_param('/catching_height_real')
            self.stop_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])   # if object height is lower than catching height real, publish stop pos instead of object pos
        else:
            self.catching_height_real = None

        self.real_trigger_threshold_x = rospy.get_param('rocat_sim_manager/real_trigger_threshold_x')

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

        # cập nhật trạng thái cho lần sau
        self.last_time = cur_time
        self.last_pos = pos
        if self.object_type == "flying_object":
            if z_new <= self.catching_height_real or (x_new >= self.real_trigger_threshold_x and z_new > self.stop_state[2] and self.stop_state[5]<0) :
                # the second condition is to prevent the object from bouncing up
                odom.pose.pose.position.x = self.stop_state[0]
                odom.pose.pose.position.y = self.stop_state[1]
                odom.pose.pose.position.z = self.stop_state[2]
            else:
                self.stop_state = [pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]]

        # 8) Publish
        self.pub.publish(odom)

        # --- nếu bật, gửi thêm TF transform ---
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp    = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id  = f"{self.object_type}_frame"  # e.g. flying_object_frame
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            if self.object_type == "flying_object":
                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1
            else:
                t.transform.rotation.x = q_new[0]
                t.transform.rotation.y = q_new[1]
                t.transform.rotation.z = q_new[2]
                t.transform.rotation.w = q_new[3]
            self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('mocap_y_up_to_z_up_with_twist')

    # đọc param topics
    robot_in  = rospy.get_param('robot_pose_y_up_topic')
    robot_out = rospy.get_param('robot_pose_z_up_topic')
    obj_in    = rospy.get_param('object_pose_y_up_topic')
    obj_out   = rospy.get_param('object_pose_z_up_topic')

    # đọc param bật/tắt TF (mặc định False)
    publish_tf = rospy.get_param('~publish_tf', False)

    # khởi tạo converter, truyền publish_tf vào
    PoseToOdomConverter(robot_in, robot_out, "robot", publish_tf)
    PoseToOdomConverter(obj_in, obj_out,    "flying_object", publish_tf)

    rospy.spin()
