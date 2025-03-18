#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
import tf.transformations
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

def reset_robot(x_init=0.0, y_init=0.0, z_init=0.6, roll_init=0.0, pitch_init=0.0, yaw_init=0.0):
    rospy.wait_for_service("/gazebo/set_model_state")
    try:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state_msg = SetModelStateRequest()
        state_msg.model_state.model_name = "go1_gazebo"
        state_msg.model_state.pose.position.x = x_init
        state_msg.model_state.pose.position.y = y_init
        state_msg.model_state.pose.position.z = z_init
        # convert roll, pitch, yaw to quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll_init, pitch_init, yaw_init)
        state_msg.model_state.pose.orientation.x = quaternion[0]
        state_msg.model_state.pose.orientation.y = quaternion[1]
        state_msg.model_state.pose.orientation.z = quaternion[2]
        state_msg.model_state.pose.orientation.w = quaternion[3]
        state_msg.model_state.reference_frame = "world"

        resp = set_state(state_msg)
        if resp.success:
            rospy.loginfo("Robot reset thành công!")
        else:
            rospy.logwarn("Reset thất bại!")
    except rospy.ServiceException as e:
        rospy.logerr("Lỗi khi gọi service: %s" % e)

def model_exists(model_name):
    """ Kiểm tra xem model có tồn tại trong Gazebo không """
    rospy.wait_for_service('/gazebo/get_world_properties')
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties = get_world_properties()
        return model_name in world_properties.model_names
    except rospy.ServiceException as e:
        rospy.logwarn(f"Failed to get world properties: {e}")
        return False  # Nếu không thể kiểm tra, giả định model không tồn tại

def delete_model(model_name):
    """ Xóa model khỏi Gazebo nếu nó tồn tại """
    if model_exists(model_name):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_srv(model_name)
            rospy.loginfo(f"Deleted model: {model_name}")
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to delete model {model_name}: {e}")
    else:
        rospy.loginfo(f"Model {model_name} does not exist. No need to delete.")

    # Chờ 2s để đảm bảo Gazebo cập nhật trạng thái
    rospy.sleep(2)

def spawn_marker(x, y, z):
    model_name = "marker_sphere"

    # Xóa model nếu đã tồn tại
    delete_model(model_name)

    # Chờ dịch vụ spawn của Gazebo
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Định nghĩa SDF chỉ có visual, không có va chạm
    sphere_sdf = """<?xml version="1.0"?>
    <sdf version="1.6">
      <model name="marker_sphere">
        <static>true</static>  <!-- Không bị tác động bởi trọng lực -->
        <link name="visual_link">
          <visual name="visual">
            <geometry>
              <sphere>
                <radius>0.125</radius>  <!-- Kích thước hình cầu -->
              </sphere>
            </geometry>
            <material>
              <ambient>1 0 0 1</ambient>  <!-- Màu đỏ -->
            </material>
          </visual>
        </link>
      </model>
    </sdf>"""

    # Thiết lập vị trí của marker
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    try:
        spawn_model(model_name, sphere_sdf, "", pose, "world")
        rospy.loginfo(f"Spawned marker in Gazebo successfully at ({x}, {y}, {z})")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to spawn marker: %s" % e)


if __name__ == "__main__":
    rospy.init_node("reset_robot_node")
    reset_robot()
