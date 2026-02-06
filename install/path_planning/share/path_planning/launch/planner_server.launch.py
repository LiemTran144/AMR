import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
import lifecycle_msgs.msg

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    path_planning_pkg = get_package_share_directory("path_planning")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",)

    nav2_planner_server = LifecycleNode(
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
        namespace="",
    )

    configure_planner = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(nav2_planner_server),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate events for both servers
    activate_planner = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=nav2_planner_server,
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Planner server is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(nav2_planner_server),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Shutdown events for both servers
    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name(node_name="planner_server"),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(msg="[LifecycleLaunch] Lifecycle nodes are shutting down."),
            ],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_planner_server,
        configure_planner, 
        activate_planner,
        shutdown_event,
    ])