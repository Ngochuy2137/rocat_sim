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
import numpy as np
from std_srvs.srv import SetBool, SetBoolResponse

# Load global config
rospack = rospkg.RosPack()
package_path = rospack.get_path('rocat_sim')  # Tên package của bạn
json_file_path = os.path.join(package_path, 'configs/config.json')
print(f'json_file_path: {json_file_path}')
GLOBAL_CONFIG = Config(json_file_path)

CONTROLLER_NEED_Y_UP = False   # only apply to simulate in gazebo, not apply for data feed to model

global_printer = Printer()
class MovementDelayDebugger:
    def __init__(self):
        rospy.init_node('movement_delay_debugger')

        # 1. Get original robot pose and current robot pose
        rospy.Subscriber(GLOBAL_CONFIG.realtime_robot_pose_topic, Odometry, self.realtime_robot_pose_callback)
        self.odom_threshold = 0.01  # mét, ngưỡng chuyển động nhỏ nhất để tính là "di chuyển"
        self.curr_robot_pose = None
        self.init_robot_pose = None
        self.robot_started_moving = False

        # 2. pub trigger
        self.trigger_pub = rospy.Publisher(GLOBAL_CONFIG.trigger_topic, PoseStamped, queue_size=10)
        self.trigger_time = None

        # 3. pub to topic GLOBAL_CONFIG.predicted_impact_point_topic PoseStamped
        self.impact_pub = rospy.Publisher(GLOBAL_CONFIG.predicted_impact_point_topic, PoseStamped, queue_size=10)
        self.goal_debug_pub = rospy.Publisher('goal_debug', PoseStamped, queue_size=10)

        # 4. srv
        rospy.Service('/robot_reached_goal_srv', SetBool, self.handle_reached_goal_signal)
        self.trial_count = 0
        self.robot_reached_goal = False

    def realtime_robot_pose_callback(self, msg:Odometry):
        self.curr_robot_pose = PoseStamped()
        self.curr_robot_pose.header = msg.header
        self.curr_robot_pose.pose = msg.pose.pose

        if self.init_robot_pose is None:
            # Get initial robot pose if not set
            self.init_robot_pose = self.curr_robot_pose
            global_printer.print_green("1. Initial robot pose set")
            return

        dx = self.curr_robot_pose.pose.position.x - self.init_robot_pose.pose.position.x
        dy = self.curr_robot_pose.pose.position.y - self.init_robot_pose.pose.position.y
        dz = self.curr_robot_pose.pose.position.z - self.init_robot_pose.pose.position.z

        distance_moved = (dx**2 + dy**2 + dz**2)**0.5

        if self.trigger_time is not None and distance_moved > self.odom_threshold:
            delay = (rospy.Time.now() - self.trigger_time).to_sec()
            global_printer.print_yellow(f"Warm up time: {delay} seconds", background=True)
            self.trigger_time = None  # Reset trigger time after printing
            self.robot_started_moving = True

    def handle_reached_goal_signal(self, req):
        """Xử lý service request để reset self.trial_count."""
        global_printer.print_green(f"✅ Done trial number {self.trial_count + 1}")
        self.trial_count += 1

        # self.wow_impact = False
        self.robot_reached_goal = True
        return SetBoolResponse(success=True, message="Counter reset to 0")
        

    def _compute_new_goal(self, robot_pos, robot_quat, distance, alpha_deg_max):
        '''
        Calculate a new goal based on current robot pose
        distance: distance from robot to goal
        alpha_deg_max: maximum angle offset
        '''
        yaw = math_utils.quaternion_to_yaw(robot_quat)

        alpha_offset = math.radians(random.uniform(-alpha_deg_max, alpha_deg_max))
        goal_yaw = yaw + alpha_offset

        goal_x = robot_pos[0] + distance * math.cos(goal_yaw)
        goal_y = robot_pos[1] + distance * math.sin(goal_yaw)
        goal_z = robot_pos[2]
        return goal_x, goal_y, goal_z

    def run(self):
        RATE = rospy.Rate(100)  # 10Hz

        last_pos_time = rospy.Time.now()
        latest_robot_pos = np.empty((0, 3))
        showed_robot_stopping = False
        print("Press ENTER to start the robot movement delay debugger")
        input()

        TOTAL_RUN = 100
        while self.trial_count < TOTAL_RUN:
            if not self.robot_reached_goal:
                # self.robot_reached_goal = False
                global_printer.print_green(f'\n\n========================= TRIAL {self.trial_count + 1} =========================')
                print("Press ENTER to start the robot movement delay debugger")
                global_printer.print_green('========================================================')
                # input()

                # reset 
                self.init_robot_pose = None
                self.trigger_time = None
                self.robot_started_moving = False

                while self.init_robot_pose is None:
                    print('Waiting for first movement')
                    RATE.sleep()

                # 1. Compute new goal based on current robot pose
                pos = self.init_robot_pose.pose.position
                robot_pos = [pos.x, pos.y, pos.z]
                robot_quat = self.init_robot_pose.pose.orientation
                robot_quat = [robot_quat.x, robot_quat.y, robot_quat.z, robot_quat.w]
                new_goal = self._compute_new_goal(robot_pos=robot_pos,
                                                robot_quat=robot_quat,
                                                distance=GLOBAL_CONFIG.rocat_sim_conf.catch_dist, alpha_deg_max=GLOBAL_CONFIG.rocat_sim_conf.catch_ori_dev_deg_thres)

                # # select goal randomly within x range, y range relative to robot
                # x_range = [-2, 2]
                # y_range = [-2, 2]
                # new_goal = [robot_pos[0] + random.uniform(x_range[0], x_range[1]), robot_pos[1] + random.uniform(y_range[0], y_range[1]), 0.45]

                print(f'    Robot pose: {robot_pos}')
                print(f'    New goal: {new_goal}')
                print(f'    CONTROLLER_NEED_Y_UP: {CONTROLLER_NEED_Y_UP}')
                # 2. Pub trigger
                trigger_pose = PoseStamped()
                self.trigger_time = rospy.Time.now()
                trigger_pose.header.stamp = self.trigger_time
                trigger_pose.header.frame_id = "world"
                trigger_pose.pose.position.x = 0
                trigger_pose.pose.position.y = 0
                trigger_pose.pose.position.z = 0
                trigger_pose.pose.orientation.x = 0
                trigger_pose.pose.orientation.y = 0
                trigger_pose.pose.orientation.z = 0
                trigger_pose.pose.orientation.w = 1.0
                self.trigger_pub.publish(trigger_pose)
                # global_printer.print_green(f"2. Trigger published at {self.trigger_time.to_sec()} seconds")
                # 3. sleep awhile with rospy
                rospy.sleep(GLOBAL_CONFIG.rocat_sim_conf.trigger_n_thow_time_gap_sim)
                global_printer.print_green(f"3. Sleeping for {GLOBAL_CONFIG.rocat_sim_conf.trigger_n_thow_time_gap_sim} seconds")

                last_pos_time = rospy.Time.now()
                cur_pos = self.curr_robot_pose.pose.position
                last_pos = [cur_pos.x, cur_pos.y, cur_pos.z]

                # 4. Pub goal frequently
                while not rospy.is_shutdown():
                    if self.robot_reached_goal:
                        self.robot_reached_goal = False
                        rospy.sleep(2)
                        break
                    # # append robot pose to latest_robot_pos, if len latest_robot_pos > 10, remove oldest elements to keep only 10 elements
                    # if self.curr_robot_pose is not None and self.robot_started_moving:
                    #     cur_pos = self.curr_robot_pose.pose.position
                    #     latest_robot_pos = np.append(latest_robot_pos, [[cur_pos.x, cur_pos.y, cur_pos.z]], axis=0)
                    #     if len(latest_robot_pos) > 10:
                    #         latest_robot_pos = latest_robot_pos[-10:]
                    # check if all elements in latest_robot_pos are the same
                    # if len(latest_robot_pos) == 10 and showed_robot_stopping == False:
                    #     robot_is_stopping = np.allclose(latest_robot_pos, latest_robot_pos[0], atol=0.01)
                    #     if robot_is_stopping:
                    #         print(latest_robot_pos)
                    #         global_printer.print_green("Robot is stopping", background=True)
                    #         showed_robot_stopping = True

                    # 4. Publish the new goal
                    goal_pose = PoseStamped()
                    goal_pose.header.stamp = rospy.Time.now()
                    goal_pose.header.frame_id = "world"
                    goal_pose.pose.position.x = new_goal[0]
                    if CONTROLLER_NEED_Y_UP:
                        goal_pose.pose.position.y = -new_goal[2]
                        goal_pose.pose.position.z = new_goal[1]
                    else:
                        goal_pose.pose.position.y = new_goal[1]
                        goal_pose.pose.position.z = new_goal[2]
                    goal_pose.pose.orientation.x = 0
                    goal_pose.pose.orientation.y = 0
                    goal_pose.pose.orientation.z = 0
                    goal_pose.pose.orientation.w = 1.0
                    self.impact_pub.publish(goal_pose)
                    
                    goal_pose_debug = goal_pose
                    goal_pose_debug.pose.position.x = new_goal[0]
                    goal_pose_debug.pose.position.y = new_goal[1]
                    goal_pose_debug.pose.position.z = new_goal[2]
                    self.goal_debug_pub.publish(goal_pose)

                    RATE.sleep()

if __name__ == '__main__':
    try:
        node = MovementDelayDebugger()
        node.run()
    except rospy.ROSInterruptException:
        pass
