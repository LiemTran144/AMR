#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    node_path = get_package_share_directory("robot_bringup")
    twist_mux_pkg = get_package_share_directory('twist_mux')
    root_pkg_path = get_package_share_directory('robot_bringup')

    # rviz_path = DeclareLaunchArgument("rviz", default_value=os.path.join(
    #     get_package_share_directory("robot_bringup"), 
    #     "rviz", 
    #     "display_rviz.rviz"))

    rviz_path = DeclareLaunchArgument(
        "rviz",
        default_value=os.path.join(node_path, "rviz", "display_rviz.rviz"),
        description="Path to RViz config file"
    )



    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        # output="screen",
        parameters=[os.path.join(node_path, "config", "joy_config.yaml")],
    )
    
    # joy_teleop = Node(
    #     package="joy_teleop",
    #     executable="joy_teleop",
    #     parameters=[os.path.join(node_path, "config", "joy_teleop.yaml")],
    # )


    twist_relay_node = Node(
        package="nhatbot_twist_teleop",
        executable="twist_relay",
        name="twist_relay",
        # parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
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
    
    speed_control = Node(
        package="differential_drive",
        executable="speed_control.py",
    )

    differentialDrive = Node(
        package="differential_drive",
        executable="differentialDrive.py",
        output="screen",
    )
    odom = Node(
        package="differential_drive",
        executable="odom.py",
    )

    path_rviz = Node(
        package="differential_drive",
        executable="path_rviz.py",
    )

    motion_control = Node(
        package="differential_drive",
        executable="motion_control.py",
    )

    ui_Node = Node(
        package="user_interface",
        executable="ui_Node.py",
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                node_path,'launch/lidar_a1.launch.py')))


    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                twist_mux_pkg,"launch", "twist_mux_launch.py")
        ),
        launch_arguments={
            "cmd_vel_out": "/liem_controller/cmd_vel",     #  /nhatbot/cmd_vel_unstamped
            "config_topics": os.path.join(node_path, "config", "twist_mux_topics.yaml"),
            # "config_locks": os.path.join(node_path, "config", "twist_mux_locks.yaml"),
            # "config_joy":  os.path.join(nhatbot_stack_pkg, "config", "twist_mux_joy.yaml"),
            # "use_sim_time":  LaunchConfiguration("use_sim_time")
        }.items()
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory(
            'robot_bringup'), 'rviz', 'display_rviz.rviz')])

    provide_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                node_path,'launch/provide_map.launch.py')))
    
    safety_stop = Node(
        package="safety",
        executable="safety_stop.py",
    )
    

    return LaunchDescription([
        rviz_path,
        # rviz_node,
        joy_node,
        speed_control,
        differentialDrive,
        # motion_control,
        odom, 
        lidar_launch,
        # twist_mux_launch,
        twist_relay_node,
        joy_to_twist,
        provide_map,
        # path_rviz,
        # ui_Node,
        # safety_stop,
    ])