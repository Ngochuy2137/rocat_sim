#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
import tf.transformations
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

def reset_robot(x_init=0.0, y_init=0.0, z_init=0.45, roll_init=0.0, pitch_init=0.0, yaw_init=0.0):
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

COLOR_MAP = {
    "red": (1, 0, 0, 0.5),
    "green": (0, 1, 0, 0.5),
    "blue": (0, 0, 1, 0.5),
    "yellow": (1, 1, 0, 0.5),
    "purple": (0.5, 0, 0.5, 0.5),
    "white": (1, 1, 1, 0.5),
    "black": (0, 0, 0, 0.5),
}

import threading
def delete_model(model_name):
    """Tìm và xóa tất cả các model có chứa model_name trong tên một cách song song."""
    rospy.wait_for_service('/gazebo/get_world_properties')
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties = get_world_properties()
        models_to_delete = [m for m in world_properties.model_names if model_name in m]

        if not models_to_delete:
            rospy.loginfo(f"No models matching '{model_name}' found.")
            return

        rospy.wait_for_service('/gazebo/delete_model')
        delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        def delete_task(model):
            """Xóa một model."""
            try:
                delete_srv(model)
                rospy.loginfo(f"Deleted model: {model}")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to delete model {model}: {e}")

        # Khởi tạo các luồng xóa model song song
        threads = []
        for model in models_to_delete:
            thread = threading.Thread(target=delete_task, args=(model,))
            threads.append(thread)
            thread.start()

        # Chờ tất cả các luồng kết thúc
        for thread in threads:
            thread.join()

    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to get world properties: {e}")

# def delete_model(model_name):
#     """Xóa tất cả model có chứa model_name song song nhưng có delay tránh crash Gazebo."""
#     rospy.wait_for_service('/gazebo/get_world_properties')
#     try:
#         get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
#         world_properties = get_world_properties()
#         models_to_delete = [m for m in world_properties.model_names if model_name in m]

#         if not models_to_delete:
#             rospy.loginfo(f"No models matching '{model_name}' found.")
#             return

#         rospy.wait_for_service('/gazebo/delete_model')
#         delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

#         def delete_task(model):
#             """Xóa một model với delay."""
#             try:
#                 delete_srv(model)
#                 rospy.loginfo(f"Deleted model: {model}")
#                 rospy.sleep(0.05)  # 🔹 Thêm delay nhỏ để Gazebo cập nhật
#             except rospy.ServiceException as e:
#                 rospy.logwarn(f"Failed to delete model {model}: {e}")

#         # Xóa model song song
#         threads = []
#         for model in models_to_delete:
#             thread = threading.Thread(target=delete_task, args=(model,))
#             threads.append(thread)
#             thread.start()

#         for thread in threads:
#             thread.join()

#         rospy.sleep(0.2)  # 🔹 Chờ thêm chút sau khi xóa

#     except rospy.ServiceException as e:
#         rospy.logerr(f"Failed to get world properties: {e}")

def spawn_marker(x, y, z, model_name='marker_sphere', color='red', size=0.05):
    """Xóa tất cả marker cũ và vẽ một marker mới trong Gazebo."""
    delete_model(model_name)

    # Chờ dịch vụ spawn của Gazebo
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    color_rgba = COLOR_MAP.get(color.lower(), (1, 0, 0, 1))  # Mặc định là màu đỏ

    # Định nghĩa SDF chỉ có visual, không có va chạm
    sphere_sdf = f"""<?xml version="1.0"?>
    <sdf version="1.6">
      <model name="{model_name}">
        <static>true</static>  <!-- Không bị tác động bởi trọng lực -->
        <link name="visual_link">
          <visual name="visual">
            <geometry>
              <sphere>
                <radius>{size}</radius>  <!-- Kích thước hình cầu -->
              </sphere>
            </geometry>
            <material>
              <ambient>{color_rgba[0]} {color_rgba[1]} {color_rgba[2]} {color_rgba[3]}</ambient> 
              <diffuse>{color_rgba[0]} {color_rgba[1]} {color_rgba[2]} {color_rgba[3]}</diffuse>
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
        rospy.loginfo(f"Spawned marker '{model_name}' in Gazebo successfully at ({x}, {y}, {z})")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn marker '{model_name}': {e}")
        
# def spawn_marker_sequence(points, base_model_name='marker_sphere', color='red'):
#     """
#     Vẽ một chuỗi các điểm (x, y, z) trong Gazebo bằng cách spawn nhiều marker.
    
#     :param points: Danh sách các tọa độ [(x1, y1, z1), (x2, y2, z2), ...]
#     :param base_model_name: Tên cơ sở của marker (sẽ được đánh số để tránh trùng)
#     :param color: Màu của marker
#     """
#     delete_model(base_model_name)  # Xóa tất cả các marker có cùng tên gốc

#     for i, (x, y, z) in enumerate(points):
#         model_name = f"{base_model_name}_{i}"  # Tạo tên duy nhất cho mỗi marker
#         spawn_marker(x, y, z, model_name, color)


def spawn_marker_sequence_parallel(points, model_name='marker_sphere', color='red', size=0.05):
    """
    Vẽ toàn bộ chuỗi điểm như một model duy nhất thay vì từng marker riêng lẻ.
    
    :param points: Danh sách các tọa độ [(x1, y1, z1), (x2, y2, z2), ...]
    :param model_name: Tên của model duy nhất
    :param color: Màu sắc của các điểm
    """
    delete_model(model_name)  # Xóa model cũ nếu tồn tại
    rospy.sleep(1)  # Chờ Gazebo cập nhật trạng thái
    
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    color_rgba = COLOR_MAP.get(color.lower(), (1, 0, 0, 1))  # Mặc định là đỏ

    # Bắt đầu SDF Model
    sdf_content = f"""<?xml version="1.0"?>
    <sdf version="1.6">
      <model name="{model_name}">
        <static>true</static>"""  # Model tĩnh, không bị trọng lực ảnh hưởng

    # Thêm từng điểm dưới dạng visual element
    for i, (x, y, z) in enumerate(points):
        sdf_content += f"""
        <link name="point_{i}">
          <visual name="visual_{i}">
            <geometry>
              <sphere>
                <radius>{size}</radius>  <!-- Kích thước điểm -->
              </sphere>
            </geometry>
            <material>
              <ambient>{color_rgba[0]} {color_rgba[1]} {color_rgba[2]} {color_rgba[3]}</ambient>
              <diffuse>{color_rgba[0]} {color_rgba[1]} {color_rgba[2]} {color_rgba[3]}</diffuse>
            </material>
          </visual>
          <pose>{x} {y} {z} 0 0 0</pose>  <!-- Đặt vị trí -->
        </link>"""

    # Kết thúc SDF Model
    sdf_content += """
      </model>
    </sdf>"""

    # Spawn model duy nhất
    pose = Pose()
    try:
        spawn_model(model_name, sdf_content, "", pose, "world")
        rospy.loginfo(f"Spawned trajectory model '{model_name}' successfully with {len(points)} points!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn trajectory model '{model_name}': {e}")

# def spawn_marker_sequence_parallel(points, base_model_name='marker_sphere', color='red'):
#     """Spawn nhiều marker song song nhưng có delay nhỏ tránh lỗi."""
#     delete_model(base_model_name)  # 🔹 Xóa model cũ trước khi vẽ mới

#     threads = []
#     for i, (x, y, z) in enumerate(points):
#         model_name = f"{base_model_name}_{i}"
#         thread = threading.Thread(target=spawn_marker, args=(x, y, z, model_name, color))
#         threads.append(thread)
#         thread.start()
#         rospy.sleep(0.05)  # 🔹 Thêm delay nhỏ để tránh overload Gazebo

#     for thread in threads:
#         thread.join()

#     rospy.sleep(0.01)  # 🔹 Đảm bảo tất cả model đã spawn xong

if __name__ == "__main__":
    rospy.init_node("reset_robot_node")
    reset_robot()
