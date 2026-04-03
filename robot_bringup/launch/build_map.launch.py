#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    node_path = get_package_share_directory("robot_bringup")
    twist_mux_pkg = get_package_share_directory('twist_mux')
    firmware_pkg_path = get_package_share_directory('nhatbot_firmware')

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",)    

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz"
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value=os.path.join(node_path, "rviz", "build_map.rviz"),  #  nav_through_poses  ,   display_rviz
        description="Path to RViz config file"
    )
   
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz'),],
        condition=IfCondition(LaunchConfiguration("use_rviz")) 
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        # output="screen",
        parameters=[os.path.join(node_path, "config", "joy_config.yaml")],
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(node_path, "config", "joy_teleop.yaml")],
    )

    twist_relay_node = Node(
        package="nhatbot_twist_teleop",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": use_sim_time}],
        )
    
    # Twist Teleop node
    joy_to_twist = Node(
        package="nhatbot_twist_teleop",
        executable="joy_to_twist",
        name="joy_to_twist",
        output="screen",
        # parameters=[
        #     {"linear_scale": 0.5},        
        #     {"angular_scale": 0.5},        
        #     # {"deadman_button": 5},          
        #     {"deadzone_threshold": 0.01}  
        # ]
    )
    
    lidar_ole_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                node_path,'launch/lidar_ole.launch.py')))
    
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                twist_mux_pkg,"launch", "twist_mux_launch.py")
        ),
        launch_arguments={
            "cmd_vel_out": "/liem/cmd_vel_unstamped",     
            "config_topics": os.path.join(node_path, "config", "twist_mux_topics.yaml"),
            "config_locks": os.path.join(node_path, "config", "twist_mux_locks.yaml"),
            "config_joy":  os.path.join(node_path, "config", "twist_mux_joy.yaml"),
            "use_sim_time":  use_sim_time,
        }.items()
    )

    hw_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                firmware_pkg_path,'launch','bringup_hardware_interface.launch.py')))
    
    build_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                node_path,'launch/build_map.launch.py')))
      
    return LaunchDescription([
        rviz_arg,
        use_rviz_arg,
        use_sim_time_arg,
        rviz_node,
        joy_node,
        joy_teleop,
        twist_mux_launch,
        twist_relay_node,
        joy_to_twist,
        lidar_ole_launch,
        hw_interface_launch,
        build_map_launch
    ])