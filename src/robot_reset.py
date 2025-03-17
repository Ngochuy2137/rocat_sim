#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

def reset_robot(x_init=0.0, y_init=0.0, z_init=0.6):
    rospy.init_node("reset_robot_node")
    rospy.wait_for_service("/gazebo/set_model_state")
    
    try:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state_msg = SetModelStateRequest()
        state_msg.model_state.model_name = "go1_gazebo"
        state_msg.model_state.pose.position.x = x_init
        state_msg.model_state.pose.position.y = y_init
        state_msg.model_state.pose.position.z = z_init
        state_msg.model_state.pose.orientation.w = 1.0
        state_msg.model_state.reference_frame = "world"

        resp = set_state(state_msg)
        if resp.success:
            rospy.loginfo("Robot reset thành công!")
        else:
            rospy.logwarn("Reset thất bại!")
    except rospy.ServiceException as e:
        rospy.logerr("Lỗi khi gọi service: %s" % e)

if __name__ == "__main__":
    reset_robot()
