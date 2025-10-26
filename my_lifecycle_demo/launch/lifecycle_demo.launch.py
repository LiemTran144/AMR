#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo
from launch_ros.events.lifecycle import ChangeState, matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers.on_shutdown import OnShutdown
import lifecycle_msgs.msg


def generate_launch_description():
    node_name = 'lifecycle_talker'

    lifecycle_node = LifecycleNode(
        package='my_lifecycle_demo',
        executable='lifecycle_talker',
        name=node_name,
        namespace='/',
        output='screen',
        emulate_tty=True
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name(node_name),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            goal_state='inactive',
            entities=[
                LogInfo(msg="[Launch] Node configured, activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name(node_name),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg="[Launch] Node shutting down..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_node_name(node_name),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
            ],
        )
    )

    return LaunchDescription([
        lifecycle_node,
        configure,
        activate,
        shutdown,
    ])
