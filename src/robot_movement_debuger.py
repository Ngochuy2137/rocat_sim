#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from python_utils.printer import Printer
from python_utils import math as math_utils
import rospkg
import os
from rocat_sim.src.utils.utils import Config
import random
import math

# Load global config
rospack = rospkg.RosPack()
package_path = rospack.get_path('rocat_sim')  # Tên package của bạn
json_file_path = os.path.join(package_path, 'configs/config.json')
print(f'json_file_path: {json_file_path}')
GLOBAL_CONFIG = Config(json_file_path)

global_printer = Printer()
class MovementDelayDebugger:
    def __init__(self):
        rospy.init_node('movement_delay_debugger')

        # 1. Get trigger signal
        rospy.Subscriber(GLOBAL_CONFIG.trigger_topic, PoseStamped, self.get_trigger_callback)
        self.got_trigger = False
        self.trigger_time = None

        # 2. Get original robot pose and current robot pose
        rospy.Subscriber(GLOBAL_CONFIG.realtime_robot_pose_topic, Odometry, self.realtime_robot_pose_callback)
        self.odom_threshold = 0.05  # mét, ngưỡng chuyển động nhỏ nhất để tính là "di chuyển"
        self.curr_robot_pos = None

        # 3. pub to topic GLOBAL_CONFIG.predicted_impact_point_topic PoseStamped
        self.impact_pub = rospy.Publisher(GLOBAL_CONFIG.predicted_impact_point_topic, PoseStamped, queue_size=10)

    def get_trigger_callback(self, msg):
        if not self.got_trigger:
            rospy.loginfo("Got trigger at time: %s" % msg.header.stamp.to_sec())
            self.got_trigger = True
            self.trigger_time = msg.header.stamp.to_sec()
            delta_time = (rospy.Time.now() - msg.header.stamp).to_sec()
            if delta_time > 0.003:
                global_printer.print_red(f"TRIGGER comm problem: delta_time = {delta_time}")

    def realtime_robot_pose_callback(self, msg):
        self.curr_robot_pos = msg.pose.pose.position

        if not self.initial_odom_pos:
            self.initial_odom_pos = self.curr_robot_pos
            return

        dx = self.curr_robot_pos.x - self.initial_odom_pos.x
        dy = self.curr_robot_pos.y - self.initial_odom_pos.y
        dz = self.curr_robot_pos.z - self.initial_odom_pos.z

        distance_moved = (dx**2 + dy**2 + dz**2)**0.5

        if not self.odom_started and distance_moved > self.odom_threshold:
            delay = (rospy.Time.now() - self.last_chip_pose_time).to_sec()
            rospy.loginfo("Robot bắt đầu di chuyển sau %.3f giây kể từ khi nhận lệnh." % delay)
            self.odom_started = True

    def _compute_new_goal(self, robot_pos, robot_quat, distance, alpha_max):
        '''
        Calculate a new goal based on current robot pose
        distance: distance from robot to goal
        alpha_max: maximum angle offset
        '''
        yaw = math_utils.quaternion_to_yaw(robot_quat)

        alpha_offset = random.uniform(-alpha_max, alpha_max)
        goal_yaw = yaw + alpha_offset

        goal_x = robot_pos[0] + distance * math.cos(goal_yaw)
        goal_y = robot_pos[1] + distance * math.sin(goal_yaw)
        goal_z = robot_pos[2]
        return goal_x, goal_y, goal_z



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MovementDelayDebugger()
        node.run()
    except rospy.ROSInterruptException:
        pass
