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

from nae_static.utils.submodules.training_utils.data_loader import DataLoader as NAEDataLoader
from python_utils.printer import Printer
from python_utils.plotter import Plotter
from rocat_sim.src.utils.utils import (
    reset_robot,
    publish_marker_list_2gzb,
    publish_points_2rviz,
    publish_special_point,
    find_point_A,
    warn_beep
)

# from rocat_sim.srv import UpdateCatchingHeight, UpdateCatchingHeightRequest

def shutdown_node():
    rospy.loginfo("Shutting down the node...")
    rospy.signal_shutdown("User requested shutdown")

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
        object_topic = rospy.get_param('object_pose_topic')
        self.trigger_n_thow_time_gap_sim = rospy.get_param('/rocat_sim_manager/trigger_n_thow_time_gap_sim')

        # Constants
        self.MAX_CATCH_DIST = 0.8
        self.DATA_WITH_Y_UP = True

        # Environment variables and data directories
        data_dir = os.path.join(os.getenv('NAE_DATASET20'), object_name, '3-data-augmented', 'data_plit')
        self.data = self.load_trajectory_data(data_dir)

        # Publishers
        self.traj_pub = rospy.Publisher(object_topic, PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher(real_trajectory_viz_topic, Marker, queue_size=10)
        self.go1_trigger_pub = rospy.Publisher(trigger_dummy_run_topic, PoseStamped, queue_size=100)
        self.rviz_object_pub = rospy.Publisher(object_pose_z_up_viz_topic, PoseStamped, queue_size=10)

        # Service server
        # Service from robot controller node
        rospy.Service('/robot_reached_goal_srv', SetBool, self.handle_robot_reach_goal_srv)

        # delete param catching height if it exists, will be set again in publish_trajectories
        if rospy.has_param('/catching_height'):
            rospy.delete_param('/catching_height')
            print("Deleted param /catching_height")
        
        # Service clients        
        # 1. Impact checker
        rospy.wait_for_service('/trigger_impact_checker_srv', timeout=10)
        self.trigger_impact_checker_client = rospy.ServiceProxy('/trigger_impact_checker_srv', SetBool)

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
            shutdown_node()
        
        print("        Received INFO robot reach goal signal")
        rospy.sleep(1)
        return SetBoolResponse(success=True, message="Thank you for the signal")

    def publish_trajectories(self):
        n = len(self.data)
        trial_num_target = max(n, 100)
        # for traj_idx, traj in enumerate(self.data):
        for traj_idx in range(trial_num_target):
            traj = self.data[traj_idx % n]
            if rospy.is_shutdown():
                break
            global_printer.print_blue(f"\n{'='*25} TRIAL #{traj_idx} {'='*25}", background=True)
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

            print(f'    Updated new catching height {catching_height} ->')
            print(f'    Trajectory length: {len(traj)}')
            # input('Press ENTER to continue to next trajectory')

            # 4. Set robot to initial position
            # Calculate robot initial position

            # Load config for setting robot initial position
            catch_ori_dev_deg_thres = rospy.get_param('/rocat_sim_manager/catching_orientation_dev_deg_thres')
            catch_dist = rospy.get_param('/rocat_sim_manager/catching_distance')
            alpha = random.uniform(-catch_ori_dev_deg_thres,
                                   catch_ori_dev_deg_thres)
            init_pos = find_point_A(real_catching_point_with_z_up[0], real_catching_point_with_z_up[1], alpha_degree=alpha,
                                    d=catch_dist)

            # input(f"Press ENTER to reset robot to init position {init_pos}")
            # wait for second before new run
            rospy.sleep(5)
            # Reset robot to initial position
            reset_robot(x_init=init_pos[0], y_init=init_pos[1])

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
            rospy.sleep(2)

            # Trigger robot catch
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

    def run(self):
        try:
            self.publish_trajectories()
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
    warn_beep(3)
