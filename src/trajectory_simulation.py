import rospy
from geometry_msgs.msg import PoseStamped
from nae_static.utils.submodules.training_utils.data_loader import DataLoader as NAEDataLoader
import os
from python_utils.printer import Printer
from tqdm import tqdm

global_printer = Printer()

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
            print('     Data points: ', len(traj))
            print('     Estimated time: ', len(traj)/120); input()

            for point in tqdm(traj):
                if point.shape[0] != 3:
                    raise ValueError('Trajectory point must have 3 dimensions (x, y, z)')
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "world"  # Thay đổi frame_id nếu cần

                # Thiết lập giá trị vị trí (Ví dụ: cố định tại gốc tọa độ)
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = point[2]

                # Thiết lập giá trị quay (Ví dụ: không quay)
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0

                pub.publish(pose)
                rate.sleep()

def load_trajectory_data():
    nae_data_loader = NAEDataLoader() 
    data_train, data_val, data_test = nae_data_loader.load_train_val_test_dataset(data_dir, file_format='csv')
    data_test = [d_test[:, :3] for d_test in data_test]
    return data_test

if __name__ == "__main__":
    data_test = load_trajectory_data()
    try:
        publish_trajectories(data_test)
    except rospy.ROSInterruptException:
        pass
