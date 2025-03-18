#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
from python_utils.printer import Printer
from std_srvs.srv import SetBool, SetBoolResponse

global_printer = Printer()

# Ngưỡng sai số để kiểm tra va chạm
TOLERANCEXY = 0.125  # Đơn vị: mét (có thể chỉnh sửa)
IMPACT_HEIGHT = 0.45  # Đơn vị: mét (có thể chỉnh sửa)

class ImpactChecker:
    def __init__(self):
        rospy.init_node("impact_checker", anonymous=True)

        # Subscribe vào topic vị trí robot (Odometry) và vật thể (Point)
        self.robot_sub = rospy.Subscriber("/unitree_go1/pose", Odometry, self.robot_callback)
        self.object_sub = rospy.Subscriber("/flying_object", PoseStamped, self.object_callback)

        self.robot_position = None
        self.object_position = None

        self.trial_count = 0
        self.success_count = 0
        self.wow_impact = False
        self.service = rospy.Service('/reset_catching_srv', SetBool, self.handle_reset_request)
        rospy.loginfo("ImpactChecker Service Server is running...")

    def handle_reset_request(self, req):
        """Xử lý service request để reset self.trial_count."""
        if req.data:
            self.trial_count += 1
            global_printer.print_green(f"✅ New catching {self.trial_count}")
            self.wow_impact = False
            return SetBoolResponse(success=True, message="Counter reset to 0")
        else:
            return SetBoolResponse(success=False, message="Reset not triggered")

    def robot_callback(self, msg):
        """Hàm callback nhận dữ liệu từ topic Odometry."""
        self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.check_impact()

    def object_callback(self, msg:PoseStamped):
        """Hàm callback nhận dữ liệu từ topic chứa vị trí vật thể."""
        self.object_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.check_impact()

    def check_impact(self):
        """Hàm kiểm tra xem robot có đến gần vật thể trong khoảng sai số không."""
        if self.robot_position is None or self.object_position is None:
            return  # Chưa có đủ thông tin
        
        if abs(self.object_position[2] - IMPACT_HEIGHT) > 0.02:
            return
        robot_x, robot_y, _ = self.robot_position
        object_x, object_y, _ = self.object_position

        # Tính khoảng cách giữa robot và vật thể
        distance_xy = math.sqrt((robot_x - object_x) ** 2 + (robot_y - object_y) ** 2)

        if distance_xy <= TOLERANCEXY:
            # global_printer.print_green(f"IMPACT DETECTED {self.trial_count}!")
            if self.wow_impact == False:
                self.success_count += 1
                global_printer.print_green(f"IMPACT DETECTED - CE: {distance_xy} !")
                global_printer.print_green(f'SUCCESS RATE: {self.success_count}/{self.trial_count}')
                self.wow_impact = True
        else:
            global_printer.print_yellow('No impact detected')

if __name__ == "__main__":
    try:
        ImpactChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
