import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    slam_config_arg = DeclareLaunchArgument("slam_config", default_value=os.path.join(get_package_share_directory("robot_bringup"), 
                                                                          "config", 
                                                                          "slam_config.yaml"))
    
    root_pkg_path = DeclareLaunchArgument("rviz", default_value=os.path.join(get_package_share_directory("robot_bringup"), 
                                                                          "rviz", 
                                                                          "build_map.rviz"))

    slam_config = LaunchConfiguration("slam_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = ["map_saver_server"]
    
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config, {"use_sim_time":use_sim_time}],
    )

    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        parameters=[
                    {"save_map_timeout":5.0},
                    {"use_sim_time":use_sim_time},
                    {"free_thresh_default":0.196},
                    {"occupied_thresh_default":0.65}])

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names":lifecycle_nodes},
            {"use_sim_time":use_sim_time},
            {"autostart":True}
        ]
    )

    rviz_node = Node( package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(root_pkg_path, "visualize", "display_map.rviz")],)


    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        nav2_map_saver,
        slam_toolbox,
        nav2_lifecycle_manager,
        rviz_node,
    ])