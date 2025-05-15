#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
from python_utils.printer import Printer
from std_srvs.srv import SetBool, SetBoolResponse
from rocat_sim.src.utils.utils import warn_beep
import os
import rospkg
import numpy as np
import datetime
import threading
# package_path
rospack = rospkg.RosPack()
package_path = rospack.get_path('rocat_sim')

# Load global config
TOLERANCE_XY_LIST = rospy.get_param('/impact_checker/control_tolerance_xy') # Ngưỡng sai số để kiểm tra va chạm

global_printer = Printer()

def shutdown_node():
    rospy.loginfo("Shutting down the node...")
    rospy.signal_shutdown("User requested shutdown")

def wait_for_param(param_name, timeout=5.0):
    start_time = rospy.get_time()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if rospy.has_param(param_name):
            return rospy.get_param(param_name)

        if rospy.get_time() - start_time > timeout:
            raise TimeoutError(f"Timeout waiting for param: {param_name}")

        rate.sleep()
    return None

class ImpactChecker:
    def __init__(self, rate_hz=240):
        rospy.init_node("impact_checker", anonymous=True)

        # Subscribe vào topic vị trí robot (Odometry) và vật thể (Point)
        robot_pose_z_up_topic = rospy.get_param('robot_pose_z_up_topic')
        object_pose_z_up_topic = rospy.get_param('object_pose_z_up_viz_topic')

        self.robot_sub = rospy.Subscriber(robot_pose_z_up_topic, Odometry, self.robot_callback)
        self.object_sub = rospy.Subscriber(object_pose_z_up_topic, PoseStamped, self.object_callback)

        self.trial_count = 0
        # self.success_count = 0

        rospy.Service('/trigger_impact_checker_srv', SetBool, self.handle_trigger_req)

        rospy.loginfo("ImpactChecker Service Server is running...")

        self.robot_position = None
        self.success_matrix = []
        self.reset()

        now = datetime.datetime.now()
        self.time_start = now.strftime("%d-%m-%Y_%H-%M-%S")

        self.log = ''
        self.trigger_evt = threading.Event()
        self.rate_hz = rate_hz
        self.rate = rospy.Rate(rate_hz)

    def reset(self,):
        self.object_position = None
        self.hello_object = False
        self.hello_robot = False
    
    def handle_trigger_req(self, req):
        global_printer.print_green("1. Got trigger request")
        self.reset()
        self.trigger_evt.set()

        return SetBoolResponse(success=True, message="Reset and ready to check impact")
    
    def robot_callback(self, msg):
        """Hàm callback nhận dữ liệu từ topic Odometry."""
        self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.hello_robot = True

    def object_callback(self, msg:PoseStamped):
        """Hàm callback nhận dữ liệu từ topic chứa vị trí vật thể."""
        self.object_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.hello_object = True

    def check_impact(self, real_catching_point_with_z_up):
        while not rospy.is_shutdown():
            if self.object_position is None:
                self.log = 'Waiting for object position'
                self.rate.sleep()
                continue
            if self.robot_position is None:
                self.log = 'Waiting for robot position'
                self.rate.sleep()
                continue

            time_start = rospy.Time.now().to_sec()
            """Hàm kiểm tra xem robot có đến gần vật thể trong khoảng sai số không."""
            real_catching_point_with_z_up = np.array(real_catching_point_with_z_up)
            if np.allclose(self.object_position, real_catching_point_with_z_up, atol=0.01):
                self.log = '3'
                global_printer.print_green(f'Detected catching point')

                # 1. Check kết quả của lần thử nghiệm này
                robot_xy = np.array(self.robot_position)
                object_xy = np.array(self.object_position)
                # Tính khoảng cách giữa robot và vật thể
                dis_xy = np.linalg.norm(robot_xy - object_xy)
                this_trial_result = dis_xy <= np.array(TOLERANCE_XY_LIST)

                if rospy.Time.now().to_sec()- time_start > 1/self.rate_hz:
                    global_printer.print_purple(f"       Checking takes so long time: {rospy.Time.now().to_sec()- time_start}")
                
                print(f'This trial result: {this_trial_result} - DIST: {dis_xy}')
                self.success_matrix.append(this_trial_result)


                # 2. Check kết quả của tất cả các lần thử nghiệm
                success_matrix_np = np.array(self.success_matrix)
                success_percentage = np.sum(success_matrix_np, axis=0) / success_matrix_np.shape[0]
                assert success_percentage.shape[0] == len(TOLERANCE_XY_LIST)
                print(f"       Success percentage: {success_percentage}")
                
                # 3. Ghi file
                try:
                    result_folder = os.path.join(package_path, 'results')
                    os.makedirs(result_folder, exist_ok=True)
                    # update object name, model name from param server
                    object_name = rospy.get_param('/object_name')
                    model_name = rospy.get_param('/model_name')
                    file_path = os.path.join(result_folder, f'impact_checker-{self.time_start}-{object_name}-{model_name}.txt')
                    with open(file_path, 'a') as f:
                        f.write(f"Trial {self.trial_count} result: {this_trial_result}\n")
                        f.write(f"Success percentage: {success_percentage}\n")
                        f.write(f"{'-'*50}\n")
                    print(f"       Result saved to {file_path}")
                except Exception as e:
                    rospy.logerr(f"Error when logging to file: {e}")
                return
                    
            self.rate.sleep()

    def run(self,):
        """Hàm chạy vòng lặp chính của node."""
        while not rospy.is_shutdown():
            if self.trigger_evt.wait():
                global_printer.print_blue(f"\n{'='*25} TRIAL #{self.trial_count} {'='*25}", background=True)
                self.trial_count += 1
                # wait awhile
                rospy.sleep(1)
                # update catching height and number of trajectory points
                real_catching_point_with_z_up = rospy.get_param('/real_catching_point_with_z_up')
                print(f'Updated new trajectory params:')
                print(f'    Real catching point (with z up):    {real_catching_point_with_z_up}')
                print(f'    Catching height:                    {real_catching_point_with_z_up[-1]}')
                
                self.check_impact(real_catching_point_with_z_up)
                self.trigger_evt.clear()

if __name__ == "__main__":
    try:
        ic = ImpactChecker()
        # rospy.spin()
        ic.run()
    except rospy.ROSInterruptException:
        pass
