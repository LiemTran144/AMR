#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_ui import RobotUI
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox
from PyQt5.QtCore import QTimer
import sys
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import csv
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Bool
from math import sqrt
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np  # For vectorized distance calc

class RobotUINode(Node, RobotUI):
    def __init__(self):
        Node.__init__(self, 'robot_ui_node')
        self.get_logger().info("Robot UI Node Initialized")
        RobotUI.__init__(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.group_Reent_ = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, '/UI/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/liem/ui_path', 10)

        self.odom_sub = self.create_subscription(Odometry, '/liem_controller/odom', self.odom_callback, 10, callback_group=self.group_Reent_)
        self.fault_sub_ = self.create_subscription(Bool, '/liem_controller/fault', self.fault_callback, 10, callback_group=self.group_Reent_)
        self.amcl_sub_ = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10, callback_group=self.group_Reent_)

        self.cmd_timer = QTimer(self)
        self.cmd_timer.setInterval(100)  # 100ms ~ 10 Hz
        self.cmd_timer.timeout.connect(self._publish_current_cmd)

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.linear_vel = 0.0  # Latest from odom
        self.angular_vel = 0.0  # Latest from odom

        # Button connections
        self.forwardButton.pressed.connect(self._start_publishing_forward)
        self.backwardButton.pressed.connect(self._start_publishing_backward)
        self.leftButton.pressed.connect(self._start_publishing_left)
        self.rightButton.pressed.connect(self._start_publishing_right)
        self.forwardButton.released.connect(self._stop_publishing)
        self.backwardButton.released.connect(self._stop_publishing)
        self.leftButton.released.connect(self._stop_publishing)
        self.rightButton.released.connect(self._stop_publishing)
        self.stopButton.pressed.connect(self._stop_publishing)

        self.sendGoalButton.clicked.connect(self.publish_path)
        self.loadCSVButton.clicked.connect(self.load_path)
        self.pauseButton.clicked.connect(self.toggle_pause_path)
        self.path_msg = None
        self.last_x = None
        self.last_y = None
        self.is_paused = False

        # For RMSE
        self.index = 0
        self.rmse_sum = 0.0  # Sum of squared errors

        # Waypoints as numpy for optimization
        self.waypoints_np = None

    def toggle_pause_path(self):
        """Pause or resume path publishing."""
        if self.is_paused:
            if self.path_msg and self.path_msg.poses:
                if not self.path_timer.isActive():
                    self.path_timer.start()
                    self.get_logger().info("Resumed sending path.")
                    self.pauseButton.setText("Pause")
            else:
                self.get_logger().warn("No path available. Please load CSV first.")
                QMessageBox.warning(self, "Path Error", "No path loaded.")
            self.is_paused = False
        else:
            if self.path_timer.isActive():
                self.path_timer.stop()
                self.get_logger().info("Paused sending path.")
                self.pauseButton.setText("Resume")
            self.is_paused = True

    def _publish_current_cmd(self):
        """Publish current Twist command."""
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_pub.publish(twist)

    def _start_publishing_forward(self):
        self.current_linear = 0.1
        self.current_angular = 0.0
        if not self.cmd_timer.isActive():
            self.cmd_timer.start()

    def _start_publishing_backward(self):
        self.current_linear = -0.1
        self.current_angular = 0.0
        if not self.cmd_timer.isActive():
            self.cmd_timer.start()

    def _start_publishing_left(self):
        self.current_linear = 0.0
        self.current_angular = 0.3
        if not self.cmd_timer.isActive():
            self.cmd_timer.start()

    def _start_publishing_right(self):
        self.current_linear = 0.0
        self.current_angular = -0.3
        if not self.cmd_timer.isActive():
            self.cmd_timer.start()

    def _stop_publishing(self):
        if self.cmd_timer.isActive():
            self.cmd_timer.stop()
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info("Published STOP cmd_vel")
        self.current_linear = 0.0
        self.current_angular = 0.0

    def load_path(self):
        """Load path from CSV and prepare for publishing."""
        file_path, _ = QFileDialog.getOpenFileName(self, "Open CSV File", "", "CSV Files (*.csv)")
        self.pathLabel.setText(file_path)
        self.pathLabel.adjustSize()

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        poses = []
        waypoints = []

        if file_path:
            try:
                self.plot_setpoint(file_path)
                with open(file_path, 'r') as csvfile:
                    reader = csv.DictReader(csvfile)
                    if 'x' not in reader.fieldnames or 'y' not in reader.fieldnames:
                        raise ValueError("CSV must have 'x' and 'y' columns.")
                    for row in reader:
                        pose = PoseStamped()
                        pose.header.frame_id = "map"
                        pose.header.stamp = self.get_clock().now().to_msg()
                        pose.pose.position.x = float(row['x'])
                        pose.pose.position.y = float(row['y'])
                        pose.pose.position.z = 0.0
                        pose.pose.orientation.w = 1.0  # Default; add 'qx,qy,qz,qw' if in CSV
                        poses.append(pose)
                        waypoints.append([pose.pose.position.x, pose.pose.position.y])

                path.poses = poses
                self.path_msg = path
                self.waypoints_np = np.array(waypoints) if waypoints else None
                self.get_logger().info(f"Loaded CSV: {file_path} ({len(path.poses)} poses)")
                # Reset RMSE on new path
                self.index = 0
                self.rmse_sum = 0.0
                # self.rmseLabel.setText("RMSE: 0.0000")
            except Exception as e:
                self.get_logger().error(f"Failed to read CSV file: {e}")
                QMessageBox.warning(self, "CSV Error", str(e))
                self.path_msg = None
                self.waypoints_np = None
        else:
            self.get_logger().warn("No file selected for CSV.")
            self.path_msg = None
            self.waypoints_np = None

    def publish_path(self):
        """Start continuous path sending."""
        if self.path_msg and self.path_msg.poses:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path_msg)
        else:
            self.get_logger().warn("No path available. Please load CSV first.")
            QMessageBox.warning(self, "Path Error", "No path loaded.")

    def odom_callback(self, msg: Odometry):
        """Update velocities and UI lights."""
        try:
            self.linear_vel = msg.twist.twist.linear.x
            self.angular_vel = msg.twist.twist.angular.z
            self.update_Velocity(self.linear_vel, self.angular_vel)
            if abs(self.linear_vel) <= 0.01 and abs(self.angular_vel) <= 0.01:
                self.light_color_stop()
            else:
                self.light_color_run()
        except Exception as e:
            self.get_logger().error(f"Odom callback error: {e}")

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Process AMCL pose: update position, error, DB."""
        try:
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            self.index += 1

            min_error = float('inf')
            # closest_waypoint_x, closest_waypoint_y = None, None

            if self.waypoints_np is not None and len(self.waypoints_np) > 0:
                # Vectorized distance calculation for efficiency
                distances = np.sqrt(np.sum((self.waypoints_np - np.array([current_x, current_y]))**2, axis=1))
                min_idx = np.argmin(distances)
                min_error = distances[min_idx]
                # closest_waypoint_x, closest_waypoint_y = self.waypoints_np[min_idx]

                # self.update_error(current_x, closest_waypoint_x, current_y, closest_waypoint_y)

            if current_x != self.last_x or current_y != self.last_y:
                self.update_current_position(current_x, current_y)
                # Use latest velocities from odom
                self.insert_data(current_x, current_y, min_error, self.linear_vel, self.angular_vel)
                self.update_error_plot(min_error)
                # self.rmse_sum += min_error ** 2
                # rmse_value = sqrt(self.rmse_sum / self.index) if self.index > 0 else 0.0
                # self.rmseLabel.setText(f"RMSE: {rmse_value:.4f}")
                # self.rmseLabel.adjustSize()

            self.last_x = current_x
            self.last_y = current_y
        except Exception as e:
            self.get_logger().error(f"AMCL callback error: {e}")

    def fault_callback(self, msg: Bool):
        """Handle fault message."""
        if msg.data:
            self.light_color_error()
            self.get_logger().warn("Fault detected!")

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = RobotUINode()
    node.show()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(10)  # 100 Hz

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()