#!/usr/bin/env python3

import csv
import sys
from math import sqrt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QFileDialog

from robot_ui import RobotUI

# Constants
CMD_VEL_TOPIC = "/liem_controller/cmd_vel"
NAV_PATH_TOPIC = "/liem/ui_path"
ODOM_TOPIC = "/liem_controller/odom"
FAULT_TOPIC = "/liem_controller/fault"
AMCL_TOPIC = "/amcl_pose"
CMD_TIMER_INTERVAL = 100  # ms (10 Hz)
PATH_TIMER_INTERVAL = 500  # ms (2 Hz)
SPIN_TIMER_INTERVAL = 10  # ms (100 Hz)
VELOCITIES = {
    "forward":  ( 0.1,  0.0),
    "backward": (-0.1,  0.0),
    "left":     ( 0.0,  0.3),
    "right":    ( 0.0, -0.3),
    "stop":     ( 0.0,  0.0),
}


class RobotUINode(Node, RobotUI):
    """ROS node integrated with RobotUI for controlling a mobile robot."""

    def __init__(self):
        Node.__init__(self, "robot_ui_node")
        RobotUI.__init__(self)
        self.get_logger().info("Robot UI Node Initialized")

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.nav_pub = self.create_publisher(Path, NAV_PATH_TOPIC, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.fault_sub = self.create_subscription(Bool, FAULT_TOPIC, self.fault_callback, 10)
        self.amcl_sub = self.create_subscription(PoseStamped, AMCL_TOPIC, self.amcl_callback, 10)

        # Initialize timers
        self.cmd_timer = QTimer(self)
        self.cmd_timer.setInterval(CMD_TIMER_INTERVAL)
        self.cmd_timer.timeout.connect(self._publish_current_cmd)

        self.path_timer = QTimer(self)
        self.path_timer.setInterval(PATH_TIMER_INTERVAL)
        self.path_timer.timeout.connect(self._publish_path)

        # Initialize state variables
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.path_msg = None
        self.last_x = None
        self.last_y = None
        self.is_paused = False
        self.index = 0
        self.rmse = 0.0

        # Connect UI signals
        self.forwardButton.pressed.connect(lambda: self._start_publishing("forward"))
        self.backwardButton.pressed.connect(lambda: self._start_publishing("backward"))
        self.leftButton.pressed.connect(lambda: self._start_publishing("left"))
        self.rightButton.pressed.connect(lambda: self._start_publishing("right"))
        self.forwardButton.released.connect(self._stop_publishing)
        self.backwardButton.released.connect(self._stop_publishing)
        self.leftButton.released.connect(self._stop_publishing)
        self.rightButton.released.connect(self._stop_publishing)
        self.stopButton.pressed.connect(self._stop_publishing)
        self.sendGoalButton.clicked.connect(self._start_sending_path_loop)
        self.loadCSVButton.clicked.connect(self.publish_path_from_csv)
        self.pauseButton.clicked.connect(self.toggle_pause_path)

    # ROS Callback Methods
    def odom_callback(self, msg: Odometry):
        """Handle odometry updates and update velocity display."""
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z
        self.update_velocity(self.linear_vel, self.angular_vel)

        if abs(self.linear_vel) <= 0.01 and abs(self.angular_vel) <= 0.01:
            self.set_indicator_state("stop")
        else:
            self.set_indicator_state("run")

    def amcl_callback(self, msg: PoseStamped):
        """Handle AMCL pose updates, calculate RMSE, and update plot."""
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        self.index += 1

        min_error = float("inf")
        closest_waypoint = None

        if self.path_msg and self.path_msg.poses:
            for pose in self.path_msg.poses:
                waypoint_x = pose.pose.position.x
                waypoint_y = pose.pose.position.y
                error = sqrt((current_x - waypoint_x) ** 2 + (current_y - waypoint_y) ** 2)

                if error < min_error:
                    min_error = error
                    closest_waypoint = pose

            if closest_waypoint:
                waypoint_x = closest_waypoint.pose.position.x
                waypoint_y = closest_waypoint.pose.position.y
                self.update_error(current_x, waypoint_x, current_y, waypoint_y)

        if current_x != self.last_x or current_y != self.last_y:
            self.update_current_position(current_x, current_y)
            self.insert_data(current_x, current_y, min_error, self.linear_vel, self.angular_vel)

            # self.rmse += min_error**2
            # rmse_value = sqrt(self.rmse / self.index)
            # self.rmseLabel.setText(f"RMSE: {rmse_value:.4f}")
            # self.rmseLabel.adjustSize()

        self.last_x = current_x
        self.last_y = current_y

    def fault_callback(self, msg: Bool):
        """Handle fault messages and update indicator state."""
        if msg.data:
            self.set_indicator_state("error")

    # Publishing Methods
    def _publish_current_cmd(self):
        """Publish current velocity command."""
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_pub.publish(twist)

    def _start_publishing(self, direction: str):
        """Start publishing velocity commands for the given direction."""
        self.current_linear, self.current_angular = VELOCITIES[direction]
        if not self.cmd_timer.isActive():
            self.cmd_timer.start()

    def _stop_publishing(self):
        """Stop publishing velocity commands and send stop command."""
        if self.cmd_timer.isActive():
            self.cmd_timer.stop()

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.current_linear = 0.0
        self.current_angular = 0.0

    def _publish_path(self):
        """Publish the current path message."""
        if self.path_msg:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.nav_pub.publish(self.path_msg)
            self.get_logger().info(f"Published path ({len(self.path_msg.poses)} poses)")
        else:
            if self.path_timer.isActive():
                self.path_timer.stop()
                self.get_logger().error("Path_msg is None, stopping path_timer.")

    def publish_path_from_csv(self):
        """Load and publish a path from a CSV file."""
        file_path, _ = QFileDialog.getOpenFileName(self, "Open CSV File", "", "CSV Files (*.csv)")
        self.pathLabel.setText(file_path)
        self.pathLabel.adjustSize()

        if not file_path:
            self.get_logger().warn("No file selected for CSV.")
            self.path_msg = None
            return

        self.plot_setpoint(file_path)
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        poses = []

        try:
            with open(file_path, "r") as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = float(row["x"])
                    pose.pose.position.y = float(row["y"])
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    poses.append(pose)

                path.poses = poses
                self.path_msg = path
                self.get_logger().info(f"Loaded CSV: {file_path} ({len(path.poses)} poses)")
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV file: {e}")
            self.path_msg = None

    def _start_sending_path_loop(self):
        """Start the path publishing loop."""
        self.index = 0
        self.rmse = 0.0
        if self.path_msg and self.path_msg.poses:
            if not self.path_timer.isActive():
                self.path_timer.start()
                self.get_logger().info("Started sending path continuously.")
        else:
            self.get_logger().warn("No path available. Please load CSV first.")

    def toggle_pause_path(self):
        """Toggle pausing/resuming of path publishing."""
        if self.is_paused:
            if self.path_msg and self.path_msg.poses:
                if not self.path_timer.isActive():
                    self.path_timer.start()
                    self.get_logger().info("Resumed sending path.")
                    self.pauseButton.setText("Pause")
            else:
                self.get_logger().warn("No path available. Please load CSV first.")
            self.is_paused = False
        else:
            if self.path_timer.isActive():
                self.path_timer.stop()
                self.get_logger().info("Paused sending path.")
                self.pauseButton.setText("Resume")
            self.is_paused = True

def main(args=None):
    """Initialize and run the Robot UI Node."""
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = RobotUINode()
    node.show()

    # Use QTimer for ROS spin
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(SPIN_TIMER_INTERVAL)

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()