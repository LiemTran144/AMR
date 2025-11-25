import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = [  "controller_server" ]
    controller_pkg = get_package_share_directory("liem_controller")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",)

    path_follower_client_node = Node(
        package='liem_controller',
        executable='path_follower_node',
        name='path_follower_node',
        output='screen',
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            os.path.join(
                controller_pkg,
                "config",
                "controller.yaml"),
            {"use_sim_time": use_sim_time}],
        remappings=[
            ("/cmd_vel", "/nav/cmd_vel")],
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
        use_sim_time_arg,
        path_follower_client_node,
        controller_server,
        nav2_lifecycle_manager,
    ])