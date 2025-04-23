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
    spawn_marker_sequence_parallel,
    publish_points,
    publish_special_point,
    find_point_A,
    Config
)

class ThrowManager:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('throw_manager', anonymous=True)

        # Printer and Plotter
        self.printer = Printer()
        self.plotter = Plotter()

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

        # Service advertisement
        self.done_reset = False
        self.reset_srv = rospy.Service('/reset_catching_srv', SetBool, self.handle_robot_reach_goal)

        # Service client (calls own service)
        rospy.wait_for_service('/reset_catching_srv', timeout=5)
        self.reset_client = rospy.ServiceProxy('/reset_catching_srv', SetBool)

    def send_reset_signal(self, value: bool):
        """Call the reset service to trigger the impact checker reset."""
        self.printer.print_green("Sending reset signal to impact checker...", background=True)
        try:
            req = SetBoolRequest(data=value)
            resp = self.reset_client(req)
            rospy.loginfo(f"Reset response: success={resp.success}, message='{resp.message}'")
            self.done_reset = resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Reset service call failed: {e}")

    def handle_robot_reach_goal(self, req: SetBoolRequest) -> SetBoolResponse:
        """Service callback resetting the impact checker when robot reaches goal."""
        self.send_reset_signal(True)
        return SetBoolResponse(success=self.done_reset, message="Impact checker reset triggered.")

    def publish_trajectories(self):
        rate = rospy.Rate(120)

        # Initial reset
        self.send_reset_signal(True)

        for traj_idx, traj in enumerate(self.data):
            if rospy.is_shutdown():
                break

            self.printer.print_green(f"\n{'-'*50}", background=True)
            self.printer.print_green(f"Publishing trajectory {traj_idx}")

            # Validate trajectory shape
            if traj.shape[1] != 4:
                raise ValueError('Trajectory point must have 4 dimensions (t, x, y, z)')

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

            spawn_marker_sequence_parallel(traj_vis, model_name="real_IP", color="green")
            publish_points(self.marker_pub, traj_vis)

            # Wait for reset confirmation
            if not self.done_reset:
                self.printer.print_green('Waiting for impact checker to reset...')
                while not self.done_reset and not rospy.is_shutdown():
                    rospy.sleep(0.1)
                self.done_reset = False

            # Delay before trigger
            self.printer.print_green('Waiting 2 seconds before triggering...')
            rospy.sleep(2)

            # Trigger robot catch
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'world'
            pose.pose.orientation.w = 1.0
            self.trigger_pub.publish(pose)
            rospy.sleep(self.config.rocat_sim_conf.trigger_n_thow_time_gap_sim)

            # Publish trajectory points in real-time
            last_time = rospy.Time.now().to_sec()
            for _, point in tqdm(list(enumerate(traj)), total=len(traj)):
                if rospy.is_shutdown():
                    break

                ps = PoseStamped()
                ps.header.stamp = rospy.Time.now()
                ps.header.frame_id = 'world'
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = point[1], point[2], point[3]
                ps.pose.orientation.w = 1.0

                self.traj_pub.publish(ps)
                publish_special_point(x=point[1], y=-point[3], z=point[2], special_point_pub=self.object_pub)
                rate.sleep()

                dt = rospy.Time.now().to_sec() - last_time
                if dt > 0:
                    freq = 1/dt
                    if abs(freq - 120) > 5:
                        self.printer.print_red(f"WARNING: Real rate is {freq:.2f} Hz")
                last_time = rospy.Time.now().to_sec()

    def load_trajectory_data(self, data_dir):
        loader = NAEDataLoader()
        _, _, data = loader.load_train_val_test_dataset(data_dir, file_format='csv')
        return data

    def run(self):
        try:
            self.publish_trajectories()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    manager = ThrowManager()
    manager.run()
