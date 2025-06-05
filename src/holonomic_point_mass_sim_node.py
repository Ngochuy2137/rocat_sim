#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose, PoseWithCovariance, Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import tf
import math
import time
from python_utils.printer import Printer

util_printer = Printer()

class HolonomicSim:
    def __init__(self):
        rospy.init_node('holonomic_sim')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # hướng quay
        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("/unitree_go1/pose", Odometry, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        self.tf_broadcaster = TransformBroadcaster()

        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z

    def update(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            if dt > 0.01+0.002:
                util_printer.print_red("dt is too large: {}".format(dt))

            # Tính toán vị trí mới (simple Euler integration)
            self.x += (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
            self.y += (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
            self.theta += self.omega * dt
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

            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.linear.y = self.vy
            odom.twist.twist.angular.z = self.omega

            self.odom_pub.publish(odom)

            # check if real rate is bigger than 100Hz
            rate.sleep()

if __name__ == '__main__':
    sim = HolonomicSim()
    sim.update()
