#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose, PoseWithCovariance, Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import tf
import math
import time
import numpy as np
from python_utils.printer import Printer

util_printer = Printer()

class HolonomicSim:
    def __init__(self):
        rospy.init_node('holonomic_sim')
        # get params
        # max_vel_x: 1.0
        # max_vel_y: 1.0
        # max_acc_x: 1.0
        # max_acc_y: 1.0
        # friction: 0.1

        self.max_vel_x = rospy.get_param('point_mass_sim/max_vel_x', 1.0)
        self.max_vel_y = rospy.get_param('point_mass_sim/max_vel_y', 1.0)
        self.max_vel_theta = rospy.get_param('point_mass_sim/max_vel_theta', 1.0)

        self.max_acc_x = rospy.get_param('point_mass_sim/max_acc_x', 1.0)
        self.max_acc_y = rospy.get_param('point_mass_sim/max_acc_y', 1.0)
        self.max_acc_theta = rospy.get_param('point_mass_sim/max_acc_theta', 1.0)

        self.friction = rospy.get_param('point_mass_sim/friction', 0.1)

        # wait for message from topic "/unitree_go1/pose" to initialize the position
        try:
            init_msg = rospy.wait_for_message("/unitree_go1/pose", Odometry, timeout=5.0)
            # Nếu nhận được, gán giá trị ban đầu
            self.x = init_msg.pose.pose.position.x
            self.y = init_msg.pose.pose.position.y
            # Tính theta từ quaternion (nếu cần)
            q = init_msg.pose.pose.orientation
            _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.theta = yaw

            self.vx_real = init_msg.twist.twist.linear.x
            self.vy_real = init_msg.twist.twist.linear.y
            self.vtheta_real = init_msg.twist.twist.angular.z
            rospy.loginfo(f"Got init pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
        except rospy.ROSException:
            # Nếu quá timeout mà chưa có message, khởi tạo mặc định
            rospy.logwarn("Timeout waiting for /unitree_go1/pose. Using zeros for initial state.")
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.vx_real = 0.0
            self.vy_real = 0.0
            self.vtheta_real = 0.0

        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("/unitree_go1/pose", Odometry, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        self.tf_broadcaster = TransformBroadcaster()

        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vtheta_cmd = 0.0

    def cmd_callback(self, msg):
        self.vx_cmd = msg.linear.x
        self.vy_cmd = msg.linear.y
        self.vtheta_cmd = msg.angular.z

    def update(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            if dt > 0.01+0.002:
                util_printer.print_red("dt is too large: {}".format(dt))

            # Tính toán vận tốc thực tế
            # giới hạn gia tốc
            self.vx_real += np.clip((self.vx_cmd - self.vx_real), -self.max_acc_x*dt, self.max_acc_x*dt)
            self.vx_real *= (1.0 - self.friction * dt)
            self.vx_real = np.clip(self.vx_real, -self.max_vel_x, self.max_vel_x)

            self.vy_real += np.clip((self.vy_cmd - self.vy_real), -self.max_acc_y*dt, self.max_acc_y*dt)
            self.vy_real *= (1.0 - self.friction * dt)
            self.vy_real = np.clip(self.vy_real, -self.max_vel_y, self.max_vel_y)

            self.vtheta_real += np.clip((self.vtheta_cmd - self.vtheta_real), -self.max_acc_theta*dt, self.max_acc_theta*dt)
            self.vtheta_real *= (1.0 - self.friction * dt)
            self.vtheta_real = np.clip(self.vtheta_real, -self.max_vel_theta, self.max_vel_theta)

            # Tính toán vị trí mới (simple Euler integration)
            self.x += (self.vx_real * math.cos(self.theta) - self.vy_real * math.sin(self.theta)) * dt
            self.y += (self.vx_real * math.sin(self.theta) + self.vy_real * math.cos(self.theta)) * dt
            self.theta += self.vtheta_real * dt
            self.last_time = now

            # Quaternions
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

            # TF
            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0.0),
                odom_quat,
                now,
                "base_link",
                "world"
            )

            # Odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "world"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = Quaternion(*odom_quat)

            odom.twist.twist.linear.x = self.vx_real
            odom.twist.twist.linear.y = self.vy_real
            odom.twist.twist.angular.z = self.vtheta_real

            self.odom_pub.publish(odom)

            # check if real rate is bigger than 100Hz
            rate.sleep()

if __name__ == '__main__':
    sim = HolonomicSim()
    sim.update()
