import rospy
from geometry_msgs.msg import PoseStamped
from nae_static.utils.submodules.training_utils.data_loader import DataLoader as NAEDataLoader
import os
from tqdm import tqdm
import random
import math
from python_utils.printer import Printer
from python_utils.plotter import Plotter
from rocat_sim.src.utils.utils import reset_robot, spawn_marker_sequence_parallel, publish_points, publish_special_point, send_bool_signal_srv
import time
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped

global_printer = Printer()
global_plotter = Plotter()

MAX_CATCH_DIST = 0.8
DATA_WITH_Y_UP = True   # only apply to simulate in gazebo, not apply for data feed to model

data_mother_dir = os.getenv('NAE_DATASET20')
object_name = 'ball'
data_dir = os.path.join(data_mother_dir, object_name, '3-data-augmented', 'data_plit')

def find_point_A(x_B, y_B, alpha_degree, d):
    """
    Tính tọa độ điểm A từ điểm B, góc alpha và khoảng cách d.

    :param x_B: Tọa độ x của điểm B
    :param y_B: Tọa độ y của điểm B
    :param alpha: Góc của tia AB so với trục OX (đơn vị độ)
    :param d: Khoảng cách từ A đến B
    :return: (x_A, y_A) tọa độ của điểm A
    """
    alpha_rad = math.radians(alpha_degree)  # Chuyển độ sang radian
    x_A = x_B - d * math.cos(alpha_rad)
    y_A = y_B - d * math.sin(alpha_rad)
    return [x_A, y_A]

def publish_trajectories(data):
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher(f'/mocap_sim/{object_name}', PoseStamped, queue_size=10)
    rate = rospy.Rate(120)  # 120 Hz

    while not rospy.is_shutdown():
        for traj_idx, traj in enumerate(data):
            if traj.shape[1] != 4:
                raise ValueError('Trajectory point must have 4 dimensions (t, x, y, z)')
            global_printer.print_green(f'\nPublishing trajectory {traj_idx}. ENTER to continue...')
            impact_point_no_t = traj[-1, 1:]
            if DATA_WITH_Y_UP:
                impact_point_z_up = [impact_point_no_t[0], -impact_point_no_t[2], impact_point_no_t[1]]
            else:
                impact_point_z_up = impact_point_no_t
            # dist_random_x = random.uniform(-MAX_CATCH_DIST, MAX_CATCH_DIST)
            # dist_random_y = math.sqrt(MAX_CATCH_DIST**2 - dist_random_x**2)
            # init_robot_pos = (impact_point_z_up[0]+dist_random_x, impact_point_z_up[1]+dist_random_y, 0.45)
            # # calculate orientation of robot to head to impact point
            # angle = math.atan2(-dist_random_y, -dist_random_x)

            alpha_degree = random.uniform(-20, 20)
            init_robot_pos = find_point_A(impact_point_z_up[0], impact_point_z_up[1], alpha_degree=alpha_degree, d=MAX_CATCH_DIST)

            global_printer.print_yellow(f'  ENTER to reset ROBOT to initial position: {init_robot_pos[0]}, {init_robot_pos[1]}'); input('----------------')
            print('     Data points: ', len(traj))
            print('     Estimated time: ', len(traj)/120)
            print('     impact_point: ', impact_point_no_t)
            # 1. Setup initial conditions before publishing trajectory
            reset_robot(x_init = init_robot_pos[0], y_init = init_robot_pos[1])
            if DATA_WITH_Y_UP:
                traj_show_gzb = [[p[1], -p[3], p[2]] for p in traj]
                traj_show_gzb = np.array(traj_show_gzb)
                print(traj_show_gzb.shape)
            else:
                traj_show_gzb = traj[:, 1:]
            traj_show_gzb = np.array(traj_show_gzb)
            spawn_marker_sequence_parallel(traj_show_gzb, model_name="real_IP", color="green")
            traj_pub = rospy.Publisher("/flying_object_trajectory", Marker, queue_size=10)
            publish_points(points_pub=traj_pub, points=traj_show_gzb)
            # sleep 2 seconds
            time.sleep(2)

            # 2. trigger Go1, pub to topic /mocap_pose_topic/chip_star_pose PoseStamped
            trigger_pub = rospy.Publisher("/mocap_pose_topic/chip_star_pose", PoseStamped, queue_size=100)
            trigger_pose = PoseStamped()
            trigger_pose.header.stamp = rospy.Time.now()
            trigger_pose.header.frame_id = "world"
            trigger_pose.pose.orientation.w = 1.0
            trigger_pub.publish(trigger_pose)

            # trigger impact checker
            send_bool_signal_srv(True)
            time.sleep(1)
            
            # 3. Start to publish trajectory, step by step
            # last_pub_time = time.time()
            last_pub_time = rospy.Time.now().to_sec()
            # for point in tqdm(traj):
            for p_idx, point in enumerate(traj):
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
                # spawn_marker(x=point[1], y=-point[3], z=point[2], model_name=f"Object_{p_idx}", color="blue")
                # flying_object_pub = rospy.Publisher("/flying_object", Marker, queue_size=10)
                flying_object_pub = rospy.Publisher("/flying_object", PoseStamped, queue_size=10)
                publish_special_point(x=point[1], y=-point[3], z=point[2], special_point_pub=flying_object_pub)
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
