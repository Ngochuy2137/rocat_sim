#!/home/huynn/anaconda3/envs/nae-dynamic-3-pc/bin/python

import rospy
import rospkg
import os
import random
import math
import numpy as np
from tqdm import tqdm
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import threading

from nae_static.utils.submodules.training_utils.data_loader import DataLoader as NAEDataLoader
from python_utils.printer import Printer
from python_utils.plotter import Plotter
from python_utils import ros_node_handle
from python_utils import singer

# from rocat_sim.srv import UpdateCatchingHeight, UpdateCatchingHeightRequest

# def shutdown_node():
#     rospy.loginfo("Shutting down the node...")
#     rospy.signal_shutdown("User requested shutdown")

global_printer = Printer()
global_plotter = Plotter()
class ThrowManager:
    def __init__(self, object_name):
        # Initialize ROS node
        rospy.init_node('throw_manager', anonymous=True)

        # Load configuration
        real_trajectory_viz_topic = rospy.get_param('real_trajectory_viz_topic')
        trigger_dummy_run_topic = rospy.get_param('trigger_dummy_run_topic')
        object_pose_z_up_viz_topic = rospy.get_param('object_pose_z_up_viz_topic')
        object_topic_y_up = rospy.get_param('object_pose_y_up_topic')
        self.using_real_robot = rospy.get_param('using_real_robot')

        # Publishers
        self.traj_pub = rospy.Publisher(object_topic_y_up, PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher(real_trajectory_viz_topic, Marker, queue_size=10)
        self.go1_trigger_pub = rospy.Publisher(trigger_dummy_run_topic, PoseStamped, queue_size=100)
        self.rviz_object_pub = rospy.Publisher(object_pose_z_up_viz_topic, PoseStamped, queue_size=10)
        
        # Subscribers
        if self.using_real_robot:
            object_pose_z_up_topic = rospy.get_param('object_pose_z_up_topic')
            rospy.Subscriber(object_pose_z_up_topic, Odometry, self.real_object_pose_callback)
            self.real_catching_session_started = threading.Event()
            self.real_catching_session_started.clear()
            self.real_object_pose = None
            self.real_trigger_threshold_x = rospy.get_param('rocat_sim_manager/real_trigger_threshold_x')
            self.active_zone_x = rospy.get_param('high_level_controller/active_zone_x')
            self.active_zone_y = rospy.get_param('high_level_controller/active_zone_y')

        # Service server
        # Service from robot controller node
        rospy.Service('/robot_reached_goal_srv', SetBool, self.handle_robot_reach_goal_srv)

        # delete param catching height if it exists, will be set again in publish_trajectories
        self.catching_height_real = rospy.get_param('catching_height_real')
        # Service clients        
        # # 1. Impact checker
        # if not self.using_real_robot:
        #     rospy.wait_for_service('/trigger_impact_checker_srv', timeout=10)
        #     self.trigger_impact_checker_client = rospy.ServiceProxy('/trigger_impact_checker_srv', SetBool)

        # 2. Robot controller
        rospy.wait_for_service('/ask_if_robot_is_ready_srv', timeout=10)
        self.ask_robot_controller_client = rospy.ServiceProxy('/ask_if_robot_is_ready_srv', SetBool)

        rospy.wait_for_service('/stop_control_session_srv', timeout=10)
        self.stop_control_client = rospy.ServiceProxy('/stop_control_session_srv', SetBool)

        # 3. NAE predictor
        rospy.wait_for_service('NAE/ask_if_predictor_is_ready_srv', timeout=10)
        self.trigger_nae_predictor_client = rospy.ServiceProxy('NAE/ask_if_predictor_is_ready_srv', SetBool)

        rospy.wait_for_service('NAE/stop_prediction_session_srv', timeout=10)
        self.stop_prediction_client = rospy.ServiceProxy('NAE/stop_prediction_session_srv', SetBool)


        self.already_asked_last_result = False # this var is to guarantee that asking for last result before reset the impact checker

    def real_object_pose_callback(self, msg:Odometry):
        """Callback function to handle the real object pose."""
        # check if object is in active zone
        obj_x = msg.pose.pose.position.x
        obj_y = msg.pose.pose.position.y
        if obj_x < self.active_zone_x[0] or obj_x > self.active_zone_x[1] or obj_y < self.active_zone_y[0] or obj_y > self.active_zone_y[1]:
            self.real_object_pose = None
        self.real_object_pose = PoseStamped()
        self.real_object_pose.header = msg.header
        self.real_object_pose.pose = msg.pose.pose

    def send_ask_if_robot_ready_srv(self):
        """Call the ask service to check if the robot is free."""
        print("\n-> ROBOT CONTROLLER: Asking if robot is free...")
        try:
            req = SetBoolRequest(data=False)
            resp = self.ask_robot_controller_client(req)
            print(f"        Ask response: success={resp.success}, message='{resp.message}'")
            return resp.success
        except rospy.ServiceException as e:
            global_printer.print_red(f"     Ask robot controller failed: {e}")
            return False
        
    def send_stop_control_session_srv(self):
        """Call the stop service to stop the robot."""
        print("\n-> ROBOT CONTROLLER: Sending STOP signal (/stop_control_session_srv) to robot controller...")
        try:
            req = SetBoolRequest(data=True)
            resp = self.stop_control_client(req)
            return resp.success
        except rospy.ServiceException as e:
            global_printer.print_red(f"     Stop robot controller failed: {e}")
            return False
        
    def send_stop_prediction_session_srv(self):
        """Call the stop service to stop the NAE predictor."""
        print("\n-> NAE PREDICTOR: Sending STOP signal (NAE/stop_prediction_session_srv) to NAE predictor...")
        try:
            req = SetBoolRequest(data=True)
            resp = self.stop_prediction_client(req)
            return resp.success
        except rospy.ServiceException as e:
            global_printer.print_red(f"     Stop NAE predictor failed: {e}")
            return False
        
    def send_trigger_nae_predictor_srv(self):
        """Call the trigger service to trigger the NAE predictor."""
        print("\n-> NAE PREDICTOR: Asking if NAE predictor is ready for new prediction ...- SRV: NAE/ask_if_predictor_is_ready_srv")
        try:
            req = SetBoolRequest(data=False)
            resp = self.trigger_nae_predictor_client(req)
            print(f"        Ask response: success={resp.success}, message='{resp.message}'")
            return resp.success
        except rospy.ServiceException as e:
            global_printer.print_red(f"     Ask NAE predictor failed: {e}")
            return False
    
        
    def handle_robot_reach_goal_srv(self, req: SetBoolRequest) -> SetBoolResponse:
        """Service callback resetting the impact checker when robot reaches goal."""
        print('request:', req)
        if not req.data:
            global_printer.print_red("Robot cannot reach goal, check simulation")
            singer.warn_beep(5)
            ros_node_handle.shutdown_node()
        
        print("        Received INFO robot reach goal signal")
        rospy.sleep(1)
        return SetBoolResponse(success=True, message="Thank you for the signal")

    def handle_catching_session(self):
        rospy.set_param('/catching_height', self.catching_height_real)    # height is y axis in this case
        print(f'    Updated new catching height {self.catching_height_real} ->')
        trial_count = 0
        while not rospy.is_shutdown():
            # 1. Press enter
            global_printer.print_blue(f"\n{'='*25} TRIAL #{trial_count} {'='*25}", background=True)
            trial_count += 1
            print('Press ENTER to start catching session. And wait a moment !')
            global_printer.print_blue(f"{'='*60}", background=True); input()
            # reset variables
            done_trigger = False
            self.real_object_pose = None
            # loop inside a catching session
            while not rospy.is_shutdown():
                # 2. Check if components are ready
                while not rospy.is_shutdown() and not self.send_ask_if_robot_ready_srv():
                    global_printer.print_yellow("       Waiting for Robot controller ready")
                    rospy.sleep(1)

                while not rospy.is_shutdown() and not self.send_trigger_nae_predictor_srv():
                    global_printer.print_yellow("       Waiting for NAE predictor ready for new prediction")
                    rospy.sleep(1)
                
                print(('\n\n'))
                global_printer.print_blue("       All components are ready, starting catching session ...")

                rate = rospy.Rate(120)
                count_loop_wait_object = 0
                while self.real_object_pose is None:
                    if count_loop_wait_object % 240 == 0:
                        print(f'waiting for object pose from topic {rospy.get_param("object_pose_y_up_topic")} - {count_loop_wait_object}')
                    count_loop_wait_object += 1
                    rate.sleep()
                
                # 3.1 Wait until self.real_object_pose.pose.position.x >= self.real_trigger_threshold_x before triggering
                rate = rospy.Rate(120)
                count_loop_b4_fly = 0
                while not rospy.is_shutdown() and (self.real_object_pose.pose.position.x < self.real_trigger_threshold_x):
                    # print every 1 second
                    if count_loop_b4_fly % 240 == 0:
                        print(f'waiting for object passing trigger line ... current pose: [{self.real_object_pose.pose.position.x:.3f}, \
                                                                                            {self.real_object_pose.pose.position.y:.3f}, \
                                                                                            {self.real_object_pose.pose.position.z:.3f}]')
                    count_loop_b4_fly += 1
                    rate.sleep()
                    
                # 3.2 Trigger robot catch
                if not done_trigger:
                    global_printer.print_green(f'trigger robot controller ... - {self.real_object_pose.pose.position.x:.3f}')
                    pose:PoseStamped = self.real_object_pose
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = 'world'
                    self.go1_trigger_pub.publish(pose)
                    done_trigger = True
                    trigger_time = rospy.Time.now()
                    singer.beep(duration = 0.1, freq = 100.0)

                    # 4. Wait until object is on ground to stop catching session
                    rate = rospy.Rate(120)
                    count_loop_fly = 0
                    while not rospy.is_shutdown() and self.real_object_pose.pose.position.z >= self.catching_height_real+0.1:
                        # print every 1 second
                        if count_loop_fly % 120 == 0:
                            print(f'Object is flying ... current pose: [{self.real_object_pose.pose.position.x:.3f}, {self.real_object_pose.pose.position.y:.3f}, {self.real_object_pose.pose.position.z:.3f}]')
                        count_loop_fly += 1
                        rate.sleep()
                    flying_time = rospy.Time.now() - trigger_time
                    # rospy.sleep(2) 
                    global_printer.print_green(f'Object is on ground, stop catching session ... -> Flying time: {flying_time.to_sec()} \n\n')
                    self.send_stop_control_session_srv()
                    self.send_stop_prediction_session_srv()
                    singer.beep(duration=1, freq=750)
                    break
                
    def run(self):
        try:
            self.handle_catching_session()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    # object_name = 'cap'    # ball big_sized_plane boomerang cardboard cookie_box
    #                         # cookie_box water_bottle paper_cup noodle_cup cap
    object_name = rospy.get_param('object_name')
    global_printer.print_green(f'{"="*25} LOADED PARAMS {"="*25}', background=True)
    print('    object_name:', object_name)

    # update object name to param server
    global_printer.print_blue(f"Starting throw manager for {object_name} ...", background=True)
    manager = ThrowManager(object_name)
    manager.run()
    singer.warn_beep(3)
