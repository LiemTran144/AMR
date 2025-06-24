#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_ui import RobotUI
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication, QFileDialog
from PyQt5.QtCore import QTimer
import sys
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import csv
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import Bool
from threading import Thread
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix, concatenate_matrices

# from rclpy.callback_groups import ReentrantCallbackGroup


class RobotUINode(Node, RobotUI):
    def __init__(self):
        Node.__init__(self, 'robot_ui_node')
        self.get_logger().info("Robot UI Node Initialized")
        RobotUI.__init__(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.group_Reent_ = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, "/liem_controller/cmd_vel", 10)  #'/UI/cmd_vel'
        self.nav_pub = self.create_publisher(Path, '/liem/ui_path', 10)

        self.odom_sub = self.create_subscription(Odometry,'/liem_controller/odom',self.odom_callback, 10) #, callback_group=self.group_Reent_
        self.fault_sub_ = self.create_subscription(Bool, '/liem_controller/fault', self.fault_callback, 10) #, callback_group=self.group_Reent_
        self.amcl_sub_ = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10) #, callback_group=self.group_Reent_
        self.cmd_timer = QTimer(self)
        self.cmd_timer.setInterval(100)  # 100 ms ~ 10 Hz
        self.cmd_timer.timeout.connect(self._publish_current_cmd)

        self.current_linear = 0.0
        self.current_angular = 0.0

        self.forwardButton.pressed.connect(self._start_publishing_forward)
        self.backwardButton.pressed.connect(self._start_publishing_backward)
        self.leftButton.pressed.connect(self._start_publishing_left)
        self.rightButton.pressed.connect(self._start_publishing_right)
        self.forwardButton.released.connect(self._stop_publishing)
        self.backwardButton.released.connect(self._stop_publishing)
        self.leftButton.released.connect(self._stop_publishing)
        self.rightButton.released.connect(self._stop_publishing)
        self.stopButton.pressed.connect(self._stop_publishing)

        self.path_timer = QTimer(self)
        self.path_timer.setInterval(500)   # 500 ms ~ 2 Hz
        self.path_timer.timeout.connect(self._publish_path)

        self.sendGoalButton.clicked.connect(self._start_sending_path_loop)
        self.loadCSVButton.clicked.connect(self.publish_path_from_csv)
        self.pauseButton.clicked.connect(self.toggle_pause_path)
        self.path_msg = None
        self.last_x = None
        self.last_y = None
        self.is_paused = False

    def toggle_pause_path(self):
        if self.is_paused:
            # Nếu đang tạm dừng, tiếp tục gửi đường đi
            if self.path_msg and self.path_msg.poses:
                if not self.path_timer.isActive():
                    self.path_timer.start()
                    self.get_logger().info("Resumed sending path.")
                    self.pauseButton.setText("Pause")
            else:
                self.get_logger().warn("No path available. Please load CSV first.")
            self.is_paused = False
        else:
            # Nếu đang gửi đường đi, tạm dừng
            if self.path_timer.isActive():
                self.path_timer.stop()
                self.get_logger().info("Paused sending path.")
                self.pauseButton.setText("Resume")
            self.is_paused = True

    def _publish_current_cmd(self):
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_pub.publish(twist)
        # self.get_logger().info(f"[TIMER] cmd_vel: linear={self.current_linear}, angular={self.current_angular}")

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

        # Gửi lệnh stop một lần
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info("Published STOP cmd_vel")

        self.current_linear = 0.0
        self.current_angular = 0.0

    def publish_path_from_csv(self):
        file_path, _ = QFileDialog.getOpenFileName(self,
                                                   "Open CSV File",
                                                   "",
                                                   "CSV Files (*.csv)")
                                                   
        self.pathLabel.setText(file_path)
        self.pathLabel.adjustSize()

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        poses = []

        if file_path:
            self.plot_setpoint(file_path)
            try:
                with open(file_path, 'r') as csvfile:
                    reader = csv.DictReader(csvfile)
                    for row in reader:
                        pose = PoseStamped()
                        pose.header.frame_id = "map"
                        pose.header.stamp = self.get_clock().now().to_msg()
                        pose.pose.position.x = float(row['x'])
                        pose.pose.position.y = float(row['y'])
                        pose.pose.position.z = 0.0
                        pose.pose.orientation.w = 1.0
                        poses.append(pose)

                    path.poses = poses    
                    self.path_msg = path

                    self.get_logger().info(f"Loaded CSV: {file_path} ({len(path.poses)} poses)")

            except Exception as e:
                self.get_logger().error(f"Failed to read CSV file: {e}")
                self.path_msg = None
        else:
            self.get_logger().warn("No file selected for CSV.")
            self.path_msg = None

    def _start_sending_path_loop(self):

        if self.path_msg and self.path_msg.poses:
            if not self.path_timer.isActive():
                self.path_timer.start()
                self.get_logger().info("Started sending path continuously.")
        else:
            self.get_logger().warn("No path available. Please load CSV first.")

    def _publish_path(self):

        if self.path_msg:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            self.nav_pub.publish(self.path_msg)
            self.get_logger().info(f"[TIMER] Published path ({len(self.path_msg.poses)} poses)")
        else:

            if self.path_timer.isActive():
                self.path_timer.stop()
                self.get_logger().error("Path_msg is None, stopping path_timer.")

    def odom_callback(self, msg: Odometry):

        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        self.update_Velocity(linear_vel, angular_vel)
        
        if abs(linear_vel) <= 0.01 and abs(angular_vel) <= 0.01:
            self.light_color_stop()
        else:
            self.light_color_run()

        # current_x = msg.pose.pose.position.x
        # current_y = msg.pose.pose.position.y

        # if current_x != self.last_x or current_y != self.last_y:
        #     # self.get_logger().info(f"[odom_callback] Updated Position: x={current_x}, y={current_y}")
        #     self.update_current_position(current_x, current_y)

        # self.last_x = current_x
        # self.last_y = current_y

    #     # self.get_logger().info(f"[odom_callback] Position: x={current_x}, y={current_y}, "
    #     #                        f"Linear Vel: {linear_vel}, Angular Vel: {angular_vel}")
        
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if current_x != self.last_x or current_y != self.last_y:
            # self.get_logger().info(f"[amcl_callback] Updated Position: x={current_x}, y={current_y}")
            self.update_current_position(current_x, current_y)

        self.last_x = current_x
        self.last_y = current_y


    def fault_callback(self, msg: Bool):
        if msg.data:
            self.light_color_error()



def main(args=None):
    rclpy.init(args=args)

    app = QApplication([])

    node = RobotUINode()
    node.show()

    # 💡 Tạo một thread để chạy rclpy.spin() song song với Qt GUI
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # ✅ Đây là vòng lặp GUI (Qt sẽ chạy ở thread chính)
    sys.exit(app.exec_())


# def main(args=None):
#     rclpy.init(args=args)

#     app = QApplication([])

#     node = RobotUINode()
#     node.show()

#     # Tạo một timer trong Qt để xử lý ROS 2
#     def ros_spin():
#         rclpy.spin_once(node, timeout_sec=0.1)

#     qt_timer = QTimer()
#     qt_timer.timeout.connect(ros_spin)
#     qt_timer.start(10)  # ROS 2 sẽ được xử lý mỗi 10ms

#     # Vòng lặp GUI (Qt sẽ chạy ở thread chính)
#     sys.exit(app.exec_())

#     # Đảm bảo ROS 2 được tắt khi thoát ứng dụng
#     rclpy.shutdown()



if __name__ == "__main__":
    main()
