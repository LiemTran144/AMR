from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # map_path = os.path.join(get_package_share_directory('robot_bringup'), 'map', 'new_map.yaml')
    # map_path = os.path.join(get_package_share_directory('robot_bringup'), 'map', 'map_turtlebot3.yaml')
    map_path = os.path.join(get_package_share_directory('robot_bringup'), 'map', 'fablab_map.yaml')

    amcl_config_path = os.path.join(get_package_share_directory('robot_bringup'), 'config', 'amcl_config.yaml')
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",)     

   
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'yaml_filename': map_path}]
    )

    nav2_acml_node = Node(
        package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[amcl_config_path]
    )

    nav2_lifecycle_manager_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': ['map_server','amcl']}])
    
    return LaunchDescription([
        use_sim_time_arg,
        map_server_node,
        nav2_acml_node,
        nav2_lifecycle_manager_node   
    ])