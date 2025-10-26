import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Lấy đường dẫn đến package của bạn
    pkg_share_dir = get_package_share_directory('path_planning') # <-- THAY TÊN PACKAGE CỦA BẠN

    # Đường dẫn đến file config
    # config_file_path = os.path.join(pkg_share_dir, 'config', 'costmap.yaml')

    # # Khai báo node costmap
    # costmap_node = Node(
    #     package='nav2_costmap_2d',
    #     executable='costmap_2d_node',  # Đây là node độc lập (không phải lifecycle)
    #     name='costmap',                # Tên này PHẢI khớp với tên trong file YAML
    #     output='screen',
    #     parameters=[config_file_path]  # Tải file cấu hình
    # )
    a_star_planner_node = Node(
        package='path_planning',
        executable='a_star_planner_costmap',
        name='a_star_planner',
        output='screen',
    )
    return LaunchDescription([
        # costmap_node,
        a_star_planner_node,
    ])