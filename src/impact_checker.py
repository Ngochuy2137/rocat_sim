#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
from python_utils.printer import Printer
from std_srvs.srv import SetBool, SetBoolResponse
from rocat_sim.src.utils.utils import Config
import os
import rospkg
import numpy as np

# Load global config
rospack = rospkg.RosPack()
package_path = rospack.get_path('rocat_sim')  # Tên package của bạn
json_file_path = os.path.join(package_path, 'configs/config.json')
print(f'json_file_path: {json_file_path}')
GLOBAL_CONFIG = Config(json_file_path)

global_printer = Printer()

# Ngưỡng sai số để kiểm tra va chạm
TOLERANCE_XY_LIST = GLOBAL_CONFIG.controller_tolerance_xy
CATCHING_HEIGHT = GLOBAL_CONFIG.catching_height

count = 0
class ImpactChecker:
    def __init__(self):
        rospy.init_node("impact_checker", anonymous=True)

        # Subscribe vào topic vị trí robot (Odometry) và vật thể (Point)
        self.robot_sub = rospy.Subscriber(GLOBAL_CONFIG.realtime_robot_pose_topic, Odometry, self.robot_callback)
        self.object_sub = rospy.Subscriber(GLOBAL_CONFIG.realtime_object_pose_topic, PoseStamped, self.object_callback)

        self.trial_count = 0
        # self.success_count = 0

        rospy.Service('/reset_impact_checker_srv', SetBool, self.handle_reset_request)
        rospy.Service('/ask_if_impact_checker_is_reset_srv', SetBool, self.handle_if_is_reset_ask)
        rospy.loginfo("ImpactChecker Service Server is running...")

        self.robot_position = None
        self.reset()

    def reset(self,):
        self.success_matrix = []
        self.this_trial_result = [] # [False] * len(TOLERANCE_XY_LIST)
        self.object_position = None
    
    def is_reset(self):
        if len(self.success_matrix) == 0 and len(self.this_trial_result) == 0 and self.object_position is None:
            return True
        else:
            print('Not reset yet:')
            print(f'     self.success_matrix:       {len(self.success_matrix)}')
            print(f'     self.this_trial_result:    {len(self.this_trial_result)}')
            print(f'     self.object_position:      {self.object_position}')
            return False

    def handle_reset_request(self, req):
        global_printer.print_green("Received service RESET REQUEST /reset_impact_checker_srv")
        """Xử lý service request để reset self.trial_count."""
        if len(self.this_trial_result) == 0:
            return SetBoolResponse(success=True, message="Counter reset to 0")
        
        self.trial_count += 1
        global_printer.print_green(f"\n{'-'*25} ✅ New catching {self.trial_count} {'-'*25}")

        # tổng hợp kết quả
        self.this_trial_result = np.array(self.this_trial_result)
        self.this_trial_result = np.sum(self.this_trial_result, axis=0, dtype=bool)
        if self.this_trial_result.shape[0] != len(TOLERANCE_XY_LIST):
            raise ValueError(f"self.this_trial_result shape {self.this_trial_result.shape} is not equal to len(TOLERANCE_XY_LIST) {len(TOLERANCE_XY_LIST)}")

        print(f"       Trial {self.trial_count} result: {self.this_trial_result}")
        self.success_matrix.append(self.this_trial_result)

        # print percentage of self.success_matrix
        success_matrix_np = np.array(self.success_matrix)
        success_percentage = np.sum(success_matrix_np, axis=0) / success_matrix_np.shape[0]
        if success_percentage.shape[0] != len(TOLERANCE_XY_LIST):
            raise ValueError(f"success_percentage shape {success_percentage.shape} is not equal to len(TOLERANCE_XY_LIST) {len(TOLERANCE_XY_LIST)}")
        print(f"       Success percentage: {success_percentage}")

        # reset 
        self.reset()
        
        return SetBoolResponse(success=True, message="Counter reset to 0")
    
    def handle_if_is_reset_ask(self, req):
        done_reset = self.is_reset()
        global_printer.print_green(f"Received service QUESTION /ask_if_impact_checker_is_reset_srv -> {done_reset}")
        if done_reset:
            return SetBoolResponse(success=True, message="Impact checker has been reset")
        else:
            return SetBoolResponse(success=False, message="Impact checker has not been reset yet")

    def robot_callback(self, msg):
        """Hàm callback nhận dữ liệu từ topic Odometry."""
        self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.check_impact()

    def object_callback(self, msg:PoseStamped):
        """Hàm callback nhận dữ liệu từ topic chứa vị trí vật thể."""
        self.object_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.check_impact()

    def check_impact(self):
        # global count; print(f'Count: {count} - {self.trial_count}'); count += 1
        """Hàm kiểm tra xem robot có đến gần vật thể trong khoảng sai số không."""
        if self.robot_position is None or self.object_position is None:
            return  # Chưa có đủ thông tin
        
        if abs(self.object_position[2] - CATCHING_HEIGHT) > 0.02:
            return
        
        robot_x, robot_y, _ = self.robot_position
        object_x, object_y, _ = self.object_position

        # Tính khoảng cách giữa robot và vật thể
        distance_xy = math.sqrt((robot_x - object_x) ** 2 + (robot_y - object_y) ** 2)

        this_loop_result = [False] * len(TOLERANCE_XY_LIST)
        for i, tolerance in enumerate(TOLERANCE_XY_LIST):
            if distance_xy <= tolerance:
                this_loop_result[i] = True
        
        self.this_trial_result.append(this_loop_result)

if __name__ == "__main__":
    try:
        ImpactChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
