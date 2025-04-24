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
    publish_points,
    publish_special_point,
    find_point_A,
    Config,
    warn_beep
)


global_printer = Printer()
global_plotter = Plotter()
class ThrowManager:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('throw_manager', anonymous=True)

        # Load configuration
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('rocat_sim')
        config_path = os.path.join(pkg_path, 'configs', 'config.json')
        self.config = Config(config_path)

        # Constants
        self.MAX_CATCH_DIST = 0.8
        self.DATA_WITH_Y_UP = True

        # Environment variables and data directories
        data_dir = os.path.join(os.getenv('NAE_DATASET20'), 'ball', '3-data-augmented', 'data_plit')
        self.data = self.load_trajectory_data(data_dir)

        # Publishers
        self.traj_pub = rospy.Publisher(f'/mocap_sim/ball', PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher(self.config.real_trajectory_topic, Marker, queue_size=10)
        self.trigger_pub = rospy.Publisher(self.config.trigger_topic, PoseStamped, queue_size=100)
        self.object_pub = rospy.Publisher(self.config.realtime_object_pose_topic, PoseStamped, queue_size=10)

        # Service server
        # Service from robot controller node
        rospy.Service('/robot_reached_goal_srv', SetBool, self.handle_robot_reach_goal_srv)

        # Service clients        
        rospy.wait_for_service('/reset_impact_checker_srv', timeout=10)
        self.reset_impact_checker_client = rospy.ServiceProxy('/reset_impact_checker_srv', SetBool)

        rospy.wait_for_service('/ask_if_impact_checker_is_reset_srv', timeout=5)
        self.ask_impact_checker_client = rospy.ServiceProxy('/ask_if_impact_checker_is_reset_srv', SetBool)

        rospy.wait_for_service('/ask_if_robot_is_free_srv', timeout=10)
        self.ask_robot_controller_client = rospy.ServiceProxy('/ask_if_robot_is_free_srv', SetBool)

        rospy.wait_for_service('NAE/trigger_new_prediction', timeout=10)
        self.trigger_nae_predictor_client = rospy.ServiceProxy('NAE/trigger_new_prediction', SetBool)

    def send_reset_impact_checker_srv(self, ):
        """Call the reset service to trigger the impact checker reset."""
        global_printer.print_green("Sending reset signal to impact checker...")
        try:
            req = SetBoolRequest(data=True)
            resp = self.reset_impact_checker_client(req)
            if resp.success:
                global_printer.print_green(f"   Reset response: success={resp.success}, message='{resp.message}'")
            else:
                global_printer.print_red(f"     Reset response: success={resp.success}, message='{resp.message}'")
        except rospy.ServiceException as e:
            rospy.logerr(f"Reset service call failed: {e}")

    def send_ask_impact_checker_srv(self):
        """Call the ask service to check if the impact checker is reset."""
        global_printer.print_green("Asking if impact checker is reset...")
        try:
            req = SetBoolRequest(data=False)
            resp = self.ask_impact_checker_client(req)
            print(f"        Ask response: success={resp.success}, message='{resp.message}'")
            return resp.success
        except rospy.ServiceException as e:
            global_printer.print_red(f"     Ask impact checker failed: {e}")
            return False
        
    def send_ask_robot_srv(self):
        """Call the ask service to check if the robot is free."""
        global_printer.print_green("Asking if robot is free...")
        try:
            req = SetBoolRequest(data=False)
            resp = self.ask_robot_controller_client(req)
            print(f"        Ask response: success={resp.success}, message='{resp.message}'")
            return resp.success
        except rospy.ServiceException as e:
            global_printer.print_red(f"     Ask robot controller failed: {e}")
            return False
        
    def send_trigger_nae_predictor_srv(self):
        """Call the trigger service to trigger the NAE predictor."""
        global_printer.print_green("Asking if NAE predictor is ready for new prediction ...")
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
            global_printer.print_red("        Robot cannot reach goal")
            warn_beep(5)
        
        global_printer.print_green("        Received INFO robot reach goal signal")
        self.send_reset_impact_checker_srv()
        return SetBoolResponse(success=True, message="Thank you for the signal")

    def publish_trajectories(self):
        # Initial reset
        self.send_reset_impact_checker_srv()

        for traj_idx, traj in enumerate(self.data):
            if rospy.is_shutdown():
                break
            # ask for init state before processing new trajectory
            while not self.send_ask_impact_checker_srv() or not self.send_ask_robot_srv() or not self.send_trigger_nae_predictor_srv():
                global_printer.print_yellow("Waiting for: ")
                print(f"        - Impact checker reset")
                print(f"        - Robot controller free")
                print(f"        - NAE predictor ready for new prediction")
                rospy.sleep(1)


            global_printer.print_green(f"\n{'-'*25} Ready to publish trajectory {traj_idx} {'-'*25}", background=True)

            # Validate trajectory shape
            if traj.shape[1] != 4:
                raise ValueError('Trajectory point must have 4 dimensions (t, x, y, z)')

            # cut so that last point is close to catching_height
            index = np.abs(traj[:, 2] - self.config.catching_height).argmin()
            if abs(traj[index, 2] - self.config.catching_height) > 0.05:
                global_printer.print_red(f"WARNING: Trajectory {traj_idx} is not close to catching height")
            traj = traj[:index + 1, :]

            # Compute impact point
            impact_no_t = traj[-1, 1:]
            if self.DATA_WITH_Y_UP:
                impact_up = [impact_no_t[0], -impact_no_t[2], impact_no_t[1]]
            else:
                impact_up = impact_no_t

            # Determine robot initial position
            alpha = random.uniform(-self.config.rocat_sim_conf.catch_ori_dev_deg_thres,
                                   self.config.rocat_sim_conf.catch_ori_dev_deg_thres)
            init_pos = find_point_A(impact_up[0], impact_up[1], alpha_degree=alpha,
                                    d=self.config.rocat_sim_conf.catch_dist)

            input(f"Press ENTER to reset robot to init position {init_pos}")
            reset_robot(x_init=init_pos[0], y_init=init_pos[1])

            # Prepare visualization markers for trajectory
            if self.DATA_WITH_Y_UP:
                traj_vis = np.array([[p[1], -p[3], p[2]] for p in traj])
            else:
                traj_vis = traj[:, 1:]

            publish_marker_list_2gzb(traj_vis, model_name="real_IP", color="green")
            publish_points(points_pub=self.marker_pub, points=traj_vis)

            # Delay before trigger
            global_printer.print_green('Waiting 2 seconds before triggering...')
            rospy.sleep(2)

            # Trigger robot catch
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'world'
            pose.pose.orientation.w = 1.0
            self.trigger_pub.publish(pose)
            rospy.sleep(self.config.rocat_sim_conf.trigger_n_thow_time_gap_sim)

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
                publish_special_point(x=point[1], y=-point[3], z=point[2], special_point_pub=self.object_pub)
                rate.sleep()

                dt = rospy.Time.now().to_sec() - time_now.to_sec()
                if dt > 0:
                    freq = 1/dt
                    if abs(freq - 120) > 5:
                        global_printer.print_red(f"WARNING: Real rate is {freq:.2f} Hz")

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
    manager = ThrowManager()
    manager.run()
