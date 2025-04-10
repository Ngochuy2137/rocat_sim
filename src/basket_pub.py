#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32
import rospkg
import os
from rocat_sim.src.utils.utils import Config

# Load global config
rospack = rospkg.RosPack()
package_path = rospack.get_path('rocat_sim')  # Tên package của bạn
json_file_path = os.path.join(package_path, 'configs/config.json')
print(f'json_file_path: {json_file_path}')
global_config = Config(json_file_path)

# Bán kính basket
BASKET_RADIUS = 0.125  # Đơn vị: mét
BASKET_HEIGHT = 0.45  # Đơn vị: mét
NUM_POINTS = 20  # Số điểm trên đường tròn

class BasketPublisher:
    def __init__(self):
        rospy.init_node("basket_publisher", anonymous=True)

        # Subscribe vào topic Odometry của robot để lấy vị trí
        self.odom_sub = rospy.Subscriber(global_config.realtime_robot_pose_topic, Odometry, self.odom_callback)

        # Publisher để xuất basket dưới dạng PolygonStamped
        self.basket_pub = rospy.Publisher(global_config.realtime_basket_pose_topic, PolygonStamped, queue_size=10)

    def odom_callback(self, msg):
        """Hàm callback nhận dữ liệu từ topic Odometry để cập nhật basket."""
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y

        # Tạo PolygonStamped cho basket
        basket = PolygonStamped()
        basket.header.frame_id = "world"
        basket.header.stamp = rospy.Time.now()

        # Thêm các điểm để tạo đường tròn
        for i in range(NUM_POINTS):
            angle = 2 * math.pi * i / NUM_POINTS
            point = Point32()
            point.x = robot_x + BASKET_RADIUS * math.cos(angle)
            point.y = robot_y + BASKET_RADIUS * math.sin(angle)
            point.z = BASKET_HEIGHT  # basket nằm trên mặt phẳng XY
            basket.polygon.points.append(point)

        # Publish basket
        self.basket_pub.publish(basket)

if __name__ == "__main__":
    try:
        BasketPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
