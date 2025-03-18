import rospy
from geometry_msgs.msg import PoseStamped
from nae_static.utils.submodules.training_utils.data_loader import DataLoader as NAEDataLoader
import os
from tqdm import tqdm
import random
import math
from python_utils.printer import Printer
from python_utils.plotter import Plotter
from utils.utils import reset_robot, spawn_marker
import time

global_printer = Printer()
global_plotter = Plotter()

MAX_CATCH_DIST = 0.6
DATA_WITH_Y_UP = False

data_mother_dir = os.getenv('NAE_DATASET20')
object_name = 'ball'
data_dir = os.path.join(data_mother_dir, object_name, '3-data-augmented', 'data_plit')


def publish_trajectories(data):
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher(f'/mocap_sim/{object_name}', PoseStamped, queue_size=10)
    rate = rospy.Rate(120)  # 120 Hz

    while not rospy.is_shutdown():
        for traj_idx, traj in enumerate(data):
            global_printer.print_green(f'\nPublishing trajectory {traj_idx}. ENTER to continue...')
            impact_point = traj[-1]
            dist_random_x = random.uniform(-MAX_CATCH_DIST, MAX_CATCH_DIST)
            dist_random_y = math.sqrt(MAX_CATCH_DIST**2 - dist_random_x**2)
            if DATA_WITH_Y_UP:
                impact_point_z_up = [impact_point[0], -impact_point[2], impact_point[1]]
            else:
                impact_point_z_up = impact_point
            init_robot_pos = (impact_point_z_up[0]+dist_random_x, impact_point_z_up[1]+dist_random_y, 0.45)
            # calculate orientation of robot to head to impact point
            angle = math.atan2(-dist_random_y, -dist_random_x)

            global_printer.print_yellow(f'  ENTER to reset robot to initial position: {init_robot_pos[0]}, {init_robot_pos[1]}'); input()
            print('     Data points: ', len(traj))
            print('     Estimated time: ', len(traj)/120)
            print('     impact_point: ', impact_point)
            # 1. Setup initial conditions before publishing trajectory
            reset_robot(init_robot_pos[0], init_robot_pos[1], init_robot_pos[2], 0, 0, angle)
            spawn_marker(impact_point_z_up[0], impact_point_z_up[1], impact_point_z_up[2], color='green')
            # 2. Start to publish trajectory, step by step
            # last_pub_time = time.time()
            last_pub_time = rospy.Time.now().to_sec()
            # for point in tqdm(traj):
            for point in traj:
                if point.shape[0] != 4:
                    raise ValueError('Trajectory point must have 4 dimensions (t, x, y, z)')
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()    # Use real time instead of trajectory time
                pose.header.frame_id = "world"  # Thay đổi frame_id nếu cần

                # Thiết lập giá trị vị trí (Ví dụ: cố định tại gốc tọa độ)
                pose.pose.position.x = point[1]
                pose.pose.position.y = point[2]
                pose.pose.position.z = point[3]

                # Thiết lập giá trị quay (Ví dụ: không quay)
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0

                pub.publish(pose)
                rate.sleep()
                # dt = time.time() - last_pub_time
                dt = rospy.Time.now().to_sec() - last_pub_time
                if dt != 0.0:
                    real_rate = 1/dt
                    if abs(real_rate - 120) > 5.0:
                        global_printer.print_red(f'  WARNING: Real rate is {real_rate} Hz')
                # last_pub_time = time.time()
                last_pub_time = rospy.Time.now().to_sec()

def load_trajectory_data():
    nae_data_loader = NAEDataLoader() 
    data_train, data_val, data_test = nae_data_loader.load_train_val_test_dataset(data_dir, file_format='csv')
    return data_test

if __name__ == "__main__":
    data_test = load_trajectory_data()
    try:
        publish_trajectories(data_test)
    except rospy.ROSInterruptException:
        pass
