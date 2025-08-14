#!/home/huynn/anaconda3/envs/nae-dynamic-3-pc/bin/python

import rospy
import rospkg
import os
import random
import math
import numpy as np
import time
from tqdm import tqdm
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from nae_static.utils.submodules.training_utils.data_loader import DataLoader as NAEDataLoader
from python_utils.printer import Printer
from python_utils.plotter import Plotter
from rocat_sim.src.utils.utils import (
    reset_robot,
    publish_marker_list_2gzb,
    publish_points_2rviz,
    publish_special_point,
    find_point_A,
    warn_beep,
    compute_init_distance_by_flight_time,
    compute_init_pose
)

import subprocess

def kill_ros_node(node_name):
    try:
        subprocess.run(["rosnode", "kill", node_name], check=True)
        print(f"Đã kill node: {node_name}")
    except subprocess.CalledProcessError as e:
        print(f"Không thể kill node: {node_name}. Lỗi: {e}")
        raise RuntimeError(f"Failed to kill node {node_name}. Please check if the node is running or if you have the correct permissions.")

# from rocat_sim.srv import UpdateCatchingHeight, UpdateCatchingHeightRequest

def shutdown_node():
    rospy.loginfo("Shutting down the node...")
    rospy.signal_shutdown("User requested shutdown")

np.random.seed(42)

global_printer = Printer()
global_plotter = Plotter()
class ThrowManager:
    def __init__(self, ):
        # Initialize ROS node
        rospy.init_node('throw_manager', anonymous=False)

        # Load configuration
        real_trajectory_viz_topic = rospy.get_param('real_trajectory_viz_topic')
        trigger_dummy_run_topic = rospy.get_param('trigger_dummy_run_topic')
        object_pose_z_up_viz_topic = rospy.get_param('object_pose_z_up_viz_topic')
        object_topic_y_up = rospy.get_param('object_pose_y_up_topic')
        self.wait_time_b4_trigger_ctrl = rospy.get_param('/rocat_sim_manager/wait_time_b4_trigger_ctrl')
        self.wait_time_after_robot_reset = rospy.get_param('/rocat_sim_manager/wait_time_after_robot_reset')

        self.enable_trigger_ctrl = rospy.get_param('/rocat_sim_manager/trigger_ctrl/enable')
        self.trigger_n_thow_time_gap_sim = rospy.get_param('/rocat_sim_manager/trigger_ctrl/trigger_n_thow_time_gap_sim')

        # Constants
        self.DATA_WITH_Y_UP = True

        # Environment variables and data directories
        # data_dir = os.path.join(os.getenv('NAE_DATASET20'), object_name, '3-data-augmented', 'data_plit')
        # self.data = self.load_trajectory_data(data_dir)

        # Publishers
        self.traj_pub = rospy.Publisher(object_topic_y_up, PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher(real_trajectory_viz_topic, Marker, queue_size=10)
        self.go1_trigger_pub = rospy.Publisher(trigger_dummy_run_topic, PoseStamped, queue_size=100)
        self.rviz_object_pub = rospy.Publisher(object_pose_z_up_viz_topic, PoseStamped, queue_size=10)

        # self.traj_id_start = rospy.get_param('~traj_id_start')  # 0 là giá trị mặc định nếu param không có

        # Service server
        # Service from robot controller node
        rospy.Service('/robot_reached_goal_srv', SetBool, self.handle_robot_reach_goal_srv)

        # delete param catching height if it exists, will be set again in publish_trajectories
        if rospy.has_param('/catching_height'):
            rospy.delete_param('/catching_height')
            print("Deleted param /catching_height")
        
        # # Service clients        
        # # 1. Impact checker
        # rospy.wait_for_service('/trigger_impact_checker_srv', timeout=10)
        # self.trigger_impact_checker_client = rospy.ServiceProxy('/trigger_impact_checker_srv', SetBool)

        # # 2. Robot controller
        # rospy.wait_for_service('/ask_if_robot_is_ready_srv', timeout=10)
        # self.ask_robot_controller_client = rospy.ServiceProxy('/ask_if_robot_is_ready_srv', SetBool)

        # rospy.wait_for_service('/stop_control_session_srv', timeout=10)
        # self.stop_control_client = rospy.ServiceProxy('/stop_control_session_srv', SetBool)

        # # 3. NAE predictor
        # rospy.wait_for_service('NAE/ask_if_predictor_is_ready_srv', timeout=10)
        # self.trigger_nae_predictor_client = rospy.ServiceProxy('NAE/ask_if_predictor_is_ready_srv', SetBool)

        # rospy.wait_for_service('NAE/stop_prediction_session_srv', timeout=10)
        # self.stop_prediction_client = rospy.ServiceProxy('NAE/stop_prediction_session_srv', SetBool)


    def load_data(self, object_name):
        """Reset the ThrowManager state."""
        # Load configuration
        # self.wait_time_b4_trigger_ctrl = rospy.get_param('/rocat_sim_manager/wait_time_b4_trigger_ctrl')
        # self.wait_time_after_robot_reset = rospy.get_param('/rocat_sim_manager/wait_time_after_robot_reset')
        # self.enable_trigger_ctrl = rospy.get_param('/rocat_sim_manager/trigger_ctrl/enable')
        # self.trigger_n_thow_time_gap_sim = rospy.get_param('/rocat_sim_manager/trigger_ctrl/trigger_n_thow_time_gap_sim')

        # Environment variables and data directories
        data_dir = os.path.join(os.getenv('NAE_DATASET20'), object_name, '3-data-augmented', 'data_plit')
        self.data = self.load_trajectory_data(data_dir)
        self.traj_id_start = rospy.get_param('~traj_id_start')  # 0 là giá trị mặc định nếu param không có

        # # delete param catching height if it exists, will be set again in publish_trajectories
        # if rospy.has_param('/catching_height'):
        #     rospy.delete_param('/catching_height')
        #     print("Deleted param /catching_height")
        

    def send_trigger_impact_checker_srv(self, ):
        """Call the trigger service to trigger the impact checker trigger."""
        print("-> IMPACT CHECKER: Sending trigger signal to impact checker...")
        try:
            req = SetBoolRequest(data=True)
            resp = self.trigger_impact_checker_client(req)
            return resp.success

        except rospy.ServiceException as e:
            rospy.logerr(f"trigger service call failed: {e}")
            return False

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
            warn_beep(5)
        
        print("        Received INFO robot reach goal signal")
        rospy.sleep(1)
        return SetBoolResponse(success=True, message="Thank you for the signal")

    def publish_trajectories(self, time_start, trial_num_target):
        n = len(self.data)
        trial_count = 0
        traj_idx = 0
        min_trajectory_len_thres = rospy.get_param('/rocat_sim_manager/min_trajectory_len_thres')
        while trial_count < trial_num_target:
            if rospy.is_shutdown():
                break
            # input(f'Press ENTER to continue')
            time_pass = (time.time() - time_start)/60
            time_left_predict = time_pass * (trial_num_target - traj_idx - 1) / (traj_idx + 1)
            global_printer.print_blue(f"\n{'='*25} TRIAL #{trial_count} - time: {time_pass:.3f} - time left {time_left_predict:.3f} - trajectory id: {traj_idx} {'='*25} ", background=True)

            traj = self.data[traj_idx % n]
            traj_idx += 1
            if len(traj) < min_trajectory_len_thres:
                global_printer.print_yellow(f'Skip trajectory {traj_idx-1} with length {len(traj)} < {min_trajectory_len_thres}')
                continue
            
            trial_count += 1    # only increase trial count if trajectory is valid => to keep the number of trials to trial_num_target

            # 1. Check trajectory shape
            if traj.shape[1] != 4:
                raise ValueError('Trajectory point must have 4 dimensions (t, x, y, z)')
            
            # 2. update param server
            # Compute impact point
            if self.DATA_WITH_Y_UP:
                real_catching_point_with_z_up = [traj[-1, 1], -traj[-1, 3], traj[-1, 2]]
            else:
                real_catching_point_with_z_up = [traj[-1, 1], traj[-1, 2], -traj[-1, 3]]
            real_catching_point_with_z_up = [float(x) for x in real_catching_point_with_z_up]
            catching_height = real_catching_point_with_z_up[2]
            rospy.set_param('/catching_height', catching_height)    # height is y axis in this case
            rospy.set_param('/real_catching_point_with_z_up', real_catching_point_with_z_up)
            rospy.set_param('/trajectory_idx', traj_idx)

            # 3. Check if components are ready
            while not self.send_ask_if_robot_ready_srv():
                global_printer.print_yellow("       Waiting for Robot controller ready")
                rospy.sleep(1)
            while not self.send_trigger_nae_predictor_srv():
                global_printer.print_yellow("       Waiting for NAE predictor ready for new prediction")
                rospy.sleep(1)
            while not self.send_trigger_impact_checker_srv():
                global_printer.print_yellow("       Waiting for Impact checker reset")
                rospy.sleep(1)

            # input('Press ENTER to continue to next trajectory')

            # 4. Set robot to initial position
            # Calculate robot initial position

            # Load config for setting robot initial position

            # catch_dist = rospy.get_param('/rocat_sim_manager/catching_distance')
            v_robot_max = rospy.get_param('/point_mass_sim/max_vel_x')
            a_robot_max = rospy.get_param('/point_mass_sim/max_acc_x')
            if rospy.get_param('/rocat_sim_manager/init_dist_by_flight_time/enable'):
                safety_factor = rospy.get_param('/rocat_sim_manager/init_dist_by_flight_time/init_dist_safety_factor')
                catch_dist = compute_init_distance_by_flight_time(T_flight=((len(traj)-35)/120), v_max=v_robot_max, a_max=a_robot_max, safety_factor=safety_factor)
                init_dist_min_thres = rospy.get_param('/rocat_sim_manager/init_dist_by_flight_time/init_dist_min_thres')
                catch_dist = max(catch_dist, init_dist_min_thres)  # ensure catch_dist is at least 0.5 m
            else:
                catch_dist = rospy.get_param('/rocat_sim_manager/init_catching_distance_hardcoded')

            catch_ori_dev_deg_thre_ranges = rospy.get_param('/rocat_sim_manager/catching_orientation_dev_deg_thre_ranges')
            random_range_idx = random.randint(0, len(catch_ori_dev_deg_thre_ranges) - 1)
            catch_ori_dev_deg_thres_min = catch_ori_dev_deg_thre_ranges[random_range_idx][0]
            catch_ori_dev_deg_thres_max = catch_ori_dev_deg_thre_ranges[random_range_idx][1]
            alpha = random.uniform(catch_ori_dev_deg_thres_min,
                                   catch_ori_dev_deg_thres_max)
            
            # init_pos = find_point_A(real_catching_point_with_z_up[0], real_catching_point_with_z_up[1], alpha_degree=alpha,
            #                         d=catch_dist)
            # # Reset robot to initial position
            # if np.cos(alpha*3.14159/180) < 0:
            #     yaw_init = 180
            # else:
            #     yaw_init = 0


            x_goal, y_goal = real_catching_point_with_z_up[0], real_catching_point_with_z_up[1]
            sample_within_circle = rospy.get_param('/rocat_sim_manager/sample_within_circle')
            x_init, y_init, yaw_init = compute_init_pose(x_goal, y_goal, alpha, catch_dist, sample_within_circle=sample_within_circle)
            init_pos = [x_init, y_init]

            # print('alpha:', alpha, 'yaw_init:', yaw_init); input()
            reset_robot(x_init=init_pos[0], y_init=init_pos[1], yaw_init=yaw_init)

            global_printer.print_green('\n--------------------')
            print(f'    Catching height: {catching_height}')
            print(f'    Trajectory length: {len(traj)}')
            print(f'    init cathing distance: {catch_dist:.2f} m - traj length: {len(traj)}')
            print('     sample_within_circle:', sample_within_circle)
            if sample_within_circle:
                real_catch_dist = np.linalg.norm(np.array(init_pos) - np.array(real_catching_point_with_z_up[:2]))
                print('         real_catch_dist:', real_catch_dist)

            # wait for second before new run
            rospy.sleep(self.wait_time_after_robot_reset)

            # 5. Visualization
            # Prepare visualization markers for trajectory
            if self.DATA_WITH_Y_UP:
                traj_vis = np.array([[p[1], -p[3], p[2]] for p in traj])
            else:
                traj_vis = traj[:, 1:]
            # publish_marker_list_2gzb(traj_vis, model_name="real_IP", color="green")
            publish_points_2rviz(points_pub=self.marker_pub, points=traj_vis)
            # Delay before trigger
            global_printer.print_green('Waiting 2 seconds before triggering controller ...')
            rospy.sleep(self.wait_time_b4_trigger_ctrl)

            # Trigger robot catch
            if self.enable_trigger_ctrl:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'world'
                pose.pose.orientation.w = 1.0
                self.go1_trigger_pub.publish(pose)
                rospy.sleep(self.trigger_n_thow_time_gap_sim) # sleep awhile after trigger

            # Publish trajectory points in real-time
            rate = rospy.Rate(120)
            for p_idx, point in tqdm(list(enumerate(traj)), total=len(traj)):
                time_now = rospy.Time.now()
                if rospy.is_shutdown():
                    break
                ps = PoseStamped()
                # convert point[0] to ros time
                ps.header.stamp = rospy.Time.from_sec(point[0])
                ps.header.frame_id = 'world'
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = point[1], point[2], point[3]
                ps.pose.orientation.w = 1.0

                self.traj_pub.publish(ps)
                # publish realtime flying object
                publish_special_point(x=point[1], y=-point[3], z=point[2], special_point_pub=self.rviz_object_pub)   # only for rviz viz
                rate.sleep()

                dt = rospy.Time.now().to_sec() - time_now.to_sec()
                if dt > 0:
                    freq = 1/dt
                    if abs(freq - 120) > 5:
                        global_printer.print_red(f"WARNING: Real rate is {freq:.2f} Hz")
            
            self.send_stop_control_session_srv()
            self.send_stop_prediction_session_srv()

    def load_trajectory_data(self, data_dir):
        loader = NAEDataLoader()
        _, _, data_test = loader.load_train_val_test_dataset(data_dir, file_format='csv')
        return data_test

    def run(self, trial_num_target):
        # Service clients        
        # 1. Impact checker
        rospy.wait_for_service('/trigger_impact_checker_srv', timeout=10)
        self.trigger_impact_checker_client = rospy.ServiceProxy('/trigger_impact_checker_srv', SetBool)

        # 2. Robot controller
        rospy.wait_for_service('/ask_if_robot_is_ready_srv', timeout=20)
        self.ask_robot_controller_client = rospy.ServiceProxy('/ask_if_robot_is_ready_srv', SetBool)

        rospy.wait_for_service('/stop_control_session_srv', timeout=10)
        self.stop_control_client = rospy.ServiceProxy('/stop_control_session_srv', SetBool)

        # 3. NAE predictor
        rospy.wait_for_service('NAE/ask_if_predictor_is_ready_srv', timeout=10)
        self.trigger_nae_predictor_client = rospy.ServiceProxy('NAE/ask_if_predictor_is_ready_srv', SetBool)

        rospy.wait_for_service('NAE/stop_prediction_session_srv', timeout=10)
        self.stop_prediction_client = rospy.ServiceProxy('NAE/stop_prediction_session_srv', SetBool)
        try:
            time_start = time.time()
            self.publish_trajectories(time_start, trial_num_target)
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    # UNSEEN: 
      # cookie_box        -> 
      # water_bottle      -> 
      # paper_cup         -> 
      # noodle_cup        -> 
      # cap               -> 
    # SEEN:
      # ball              -> 
      # big_sized_plane   -> 
      # boomerang         ->   
      # cardboard         -> 
      # ring_frisbee      -> 

    all_objects_list = ['boomerang', 'big_sized_plane', 'carpet', 'hat', 'ring_frisbee',
                        'cap', 'noodle_cup', 'paper_cup', 'pinwheel', 'small_sized_plane']
    trial_num_target_per_obj = 100
    manager = ThrowManager()
    for object_name in all_objects_list:
        print(f'Object: {object_name}')
        rospy.set_param('object_name', object_name)

        global_printer.print_green(f'{"="*25} LOADED PARAMS {"="*25}', background=True)
        print('    object_name:', object_name)

        # update object name to param server
        global_printer.print_blue(f"Starting throw manager for {object_name} ...", background=True)
        manager.load_data(object_name)
        manager.run(trial_num_target=trial_num_target_per_obj)

        # kill_ros_node('throw_manager')
        warn_beep(3)


    # object_name = rospy.get_param('object_name')
    # global_printer.print_green(f'{"="*25} LOADED PARAMS {"="*25}', background=True)
    # print('    object_name:', object_name)

    # # update object name to param server
    # global_printer.print_blue(f"Starting throw manager for {object_name} ...", background=True)
    # manager = ThrowManager(object_name)
    # manager.run()
    # warn_beep(3)
