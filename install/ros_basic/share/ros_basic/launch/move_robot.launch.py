from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    move_robot_path = os.path.join(
        get_package_share_directory("ros_basic"),
        "param",
        "param.yaml"
    )

    move_robot_node = Node(
        package="ros_basic",
        executable="move_robot.py",
        name="move_robot",
        output="screen",
        parameters=[move_robot_path]
    )

    ros_pub_node = Node(
        package="ros_basic",
        executable="ros_publisher.py",
        name="ros_publisher",
        output="screen"
    )
    ros_sub_node = Node(
        package="ros_basic",
        executable="ros_subscriber.py",
        name="ros_subscriber",
        output="screen"
    )

    return LaunchDescription([
        move_robot_node,
        ros_sub_node,
        ros_pub_node
    ])