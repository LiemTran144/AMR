import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = [  "smoother_server", "planner_server" ]
    # lifecycle_nodes = [ "planner_server" ]
    path_planning_pkg = get_package_share_directory("path_planning")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",)

    
    a_star_planner_costmap_node = Node(
        package='path_planning',
        executable='a_star_planner_costmap',
        name='a_star_planner',
        output='screen',
    )

    compute_path_client_node = Node(
        package='path_planning',
        executable='compute_path_client',
        name='compute_path_client',
        output='screen',
    )

    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(
                path_planning_pkg,
                "config",
                "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(
                path_planning_pkg,
                "config",
                "smoother_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        a_star_planner_costmap_node,
        # compute_path_client_node,
        use_sim_time_arg,
        nav2_planner_server,
        nav2_smoother_server,
        nav2_lifecycle_manager,
    ])