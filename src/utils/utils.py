#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
import tf.transformations
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, PoseStamped
import json
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from python_utils.printer import Printer
global_printer = Printer()


import subprocess
import time


def beep(duration: float = 0.1, freq: float = 440.0) -> None:
    """
    Phát một tiếng beep dùng sox qua ALSA.

    Args:
        duration: độ dài âm thanh (giây), mặc định 0.1s
        freq: tần số (Hz), mặc định 440 Hz (A4)
    """
    try:
        subprocess.run([
            'play', '-nq', '-t', 'alsa',
            'synth', str(duration), 'sine', str(freq)
        ], check=True)
    except FileNotFoundError:
        print("Lỗi: không tìm thấy lệnh 'play'. Hãy cài sox (sudo apt install sox libsox-fmt-all).")
    except subprocess.CalledProcessError as e:
        print(f"Beep thất bại, return code: {e.returncode}")

def warn_beep(iter):
    """
    Phát tiếng beep cảnh báo.
    """
    for _ in range(iter):
        for _ in range(3):
            beep(duration=0.2, freq=440.0)
        # nghỉ 0.5 giây giữa các tiếng beep, không dùng rospy
        time.sleep(0.5)


def reset_robot(x_init=0.0, y_init=0.0, z_init=0.45, roll_init=0.0, pitch_init=0.0, yaw_init=0.0):
    '''
    args:
    x_init: tọa độ x ban đầu
    y_init: tọa độ y ban đầu
    z_init: tọa độ z ban đầu
    roll_init: góc roll ban đầu (độ)
    pitch_init: góc pitch ban đầu (độ)
    yaw_init: góc yaw ban đầu (độ)
    '''
    rospy.wait_for_service("/gazebo/set_model_state", timeout=2)
    # convert roll, pitch, yaw from degree to radian
    roll_init = roll_init * (3.141592653589793 / 180.0)
    pitch_init = pitch_init * (3.141592653589793 / 180.0)
    yaw_init = yaw_init * (3.141592653589793 / 180.0)
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
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=2)
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties = get_world_properties()
        return model_name in world_properties.model_names
    except rospy.ServiceException as e:
        rospy.logwarn(f"Failed to get world properties: {e}")
        return False  # Nếu không thể kiểm tra, giả định model không tồn tại

COLOR_MAP = {
    "red": (1, 0, 0, 1.0),
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
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=2)
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties = get_world_properties()
        models_to_delete = [m for m in world_properties.model_names if model_name in m]

        if not models_to_delete:
            rospy.loginfo(f"No models matching '{model_name}' found.")
            return

        rospy.wait_for_service('/gazebo/delete_model', timeout=2)
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
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=2)
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


def publish_marker_list_2gzb(points, model_name='marker_sphere', color='red', size=0.05):
    """
    Vẽ toàn bộ chuỗi điểm như một model duy nhất thay vì từng marker riêng lẻ.
    
    :param points: Danh sách các tọa độ [(x1, y1, z1), (x2, y2, z2), ...]
    :param model_name: Tên của model duy nhất
    :param color: Màu sắc của các điểm
    """
    delete_model(model_name)  # Xóa model cũ nếu tồn tại
    rospy.sleep(1)  # Chờ Gazebo cập nhật trạng thái
    
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=2)
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

# def publish_marker_list_2gzb(points, base_model_name='marker_sphere', color='red'):
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

def publish_marker_list_2rviz(points,
                              topic_name='NAE/impact_point_list',
                              size=0.05,
                              text_height=0.02, this_data_is_y_up=False):
    
    if this_data_is_y_up:
        points = [[p[0], -p[2], p[1]] for p in points]

    # Publisher for a MarkerArray, latched so RViz can grab it even if it connects later
    pub = rospy.Publisher(topic_name, MarkerArray, queue_size=1, latch=True)
    rospy.sleep(0.1)  # give publisher a moment to register

    marker_array = MarkerArray()

    # 1) Marker POINTS cho tất cả các điểm
    pts_marker = Marker()
    pts_marker.header.frame_id = "world"
    pts_marker.header.stamp = rospy.Time.now()
    pts_marker.ns = "pts"
    pts_marker.id = 0
    pts_marker.type = Marker.POINTS
    pts_marker.action = Marker.ADD
    pts_marker.scale.x = pts_marker.scale.y = size
    pts_marker.color.r = 1.0
    pts_marker.color.g = 1.0
    pts_marker.color.b = 1.0
    pts_marker.color.a = 1.0
    pts_marker.points = [Point(x, y, z) for x, y, z in points]
    marker_array.markers.append(pts_marker)

    # 2) Một Marker TEXT_VIEW_FACING cho mỗi điểm để hiển thị thứ tự
    for i, (x, y, z) in enumerate(points):
        txt_marker = Marker()
        txt_marker.header.frame_id = "world"
        txt_marker.header.stamp = pts_marker.header.stamp
        txt_marker.ns = "labels"
        txt_marker.id = i + 1
        txt_marker.type = Marker.TEXT_VIEW_FACING
        txt_marker.action = Marker.ADD
        # đặt text ngay trên điểm để không bị chồng lên
        txt_marker.pose.position.x = x
        txt_marker.pose.position.y = y
        txt_marker.pose.position.z = z + size * 1.2
        txt_marker.text = str(i)
        txt_marker.scale.z = text_height
        txt_marker.color.r = 1.0
        txt_marker.color.g = 0.0
        txt_marker.color.b = 0.0
        txt_marker.color.a = 1.0
        marker_array.markers.append(txt_marker)

    # publish một vài lần để chắc RViz nhận được
    for _ in range(10):
        pub.publish(marker_array)
        rospy.sleep(0.1)

def publish_points_2rviz(points, points_pub: rospy.Publisher, color='green', size=0.05):
    """
    Publish danh sách điểm lên RViz.
    :param points: List các tuple (x, y, z)
    """
    if color not in COLOR_MAP:
        color = 'green'  # Màu mặc định
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "flying_object_trajectory"
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = size  # Kích thước điểm
    marker.scale.y = size
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = COLOR_MAP[color]
    marker.color.a = 0.5  # Độ trong suốt
    
    # Thêm các điểm vào marker
    for x, y, z in points:
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        marker.points.append(p)
        marker.pose.orientation.w = 1.0

    points_pub.publish(marker)
    rospy.loginfo(f"Published {len(points)} points to RViz, topic: {points_pub.name}")

# def publish_special_point(x, y, z, special_point_pub: rospy.Publisher):
#     """
#     Publish một điểm đặc biệt lên RViz với màu đỏ.
#     """
#     marker = Marker()
#     marker.header.frame_id = "world"
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "flying_object"
#     marker.id = 1
#     marker.type = Marker.SPHERE
#     marker.action = Marker.ADD
#     marker.pose.position.x = x
#     marker.pose.position.y = y
#     marker.pose.position.z = z
#     marker.scale.x = 0.1  # Kích thước điểm đặc biệt
#     marker.scale.y = 0.1
#     marker.scale.z = 0.1
#     marker.color.r = 0.0  # Màu đỏ
#     marker.color.g = 0.0
#     marker.color.b = 1.0
#     marker.color.a = 1.0  # Độ trong suốt

#     special_point_pub.publish(marker)
#     # rospy.loginfo("Published special point at ({}, {}, {}) to RViz".format(x, y, z))

def publish_special_point(x, y, z, special_point_pub: rospy.Publisher):
    point = PoseStamped()
    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "world"
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    point.pose.orientation.w = 1.0
    special_point_pub.publish(point)
    # rospy.loginfo("Published special point at ({}, {}, {}) to RViz".format(x, y, z))

import math
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


def compute_init_catching_distance(T_flight, v_max, a_max, safety_factor=1.0):
    """
    Tính khoảng cách tối đa robot có thể di chuyển từ vị trí đứng yên
    trong thời gian T_flight, với vận tốc và gia tốc tối đa cho trước.

    Tham số:
    - T_flight (float): thời gian bay của vật thể (giây)
    - v_max (float): vận tốc tối đa của robot (m/s)
    - a_max (float): gia tốc tối đa của robot (m/s^2)

    Trả về:
    - d_start (float): khoảng cách tối đa robot có thể đi được (mét)
    """
    t_acc = v_max / a_max  # thời gian cần để đạt đến vận tốc tối đa

    if T_flight <= t_acc:
        # Nếu chưa kịp đạt v_max → chỉ tăng tốc đều từ đứng yên
        d_start = 0.5 * a_max * T_flight**2
    else:
        # Nếu đủ thời gian tăng tốc và chạy đều
        d_start = 0.5 * a_max * t_acc**2 + v_max * (T_flight - t_acc)

    d_start *= safety_factor  # Thêm hệ số an toàn nếu cần
    return d_start

if __name__ == "__main__":
    rospy.init_node("reset_robot_node")
    reset_robot()
