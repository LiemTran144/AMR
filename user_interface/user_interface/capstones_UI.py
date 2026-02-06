#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from robot_ui import RobotUI
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox
# from PyQt5.QtCore import QTimer
# import sys
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
# import csv
# from tf2_ros import Buffer, TransformListener
# from std_msgs.msg import Bool
# from math import sqrt
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# import numpy as np  # For vectorized distance calc
# import signal

# class RobotUINode(Node, RobotUI):
#     def __init__(self):
#         Node.__init__(self, 'robot_ui_node')
#         self.get_logger().info("Robot UI Node Initialized")
#         RobotUI.__init__(self)

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.group_Reent_ = ReentrantCallbackGroup()

#         self.cmd_pub = self.create_publisher(Twist, '/UI/cmd_vel', 10)
#         # self.path_pub = self.create_publisher(Path, '/plan', 10)


#         self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10, callback_group=self.group_Reent_)
#         self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10, callback_group=self.group_Reent_)
#         # self.fault_sub_ = self.create_subscription(Bool, '/liem_controller/fault', self.fault_callback, 10, callback_group=self.group_Reent_)
#         self.amcl_sub_ = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10, callback_group=self.group_Reent_)
#         self.goal_sub_ = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10, callback_group=self.group_Reent_)

#         self.cmd_timer = QTimer(self)
#         self.cmd_timer.setInterval(100)  # 100ms ~ 10 Hz
#         self.cmd_timer.timeout.connect(self._publish_current_cmd)

#         self.current_linear = 0.0
#         self.current_angular = 0.0
#         self.linear_vel = 0.0  # Latest from odom
#         self.angular_vel = 0.0  # Latest from odom

#         # Button connections
#         self.forwardButton.pressed.connect(self._start_publishing_forward)
#         self.backwardButton.pressed.connect(self._start_publishing_backward)
#         self.leftButton.pressed.connect(self._start_publishing_left)
#         self.rightButton.pressed.connect(self._start_publishing_right)
#         self.forwardButton.released.connect(self._stop_publishing)
#         self.backwardButton.released.connect(self._stop_publishing)
#         self.leftButton.released.connect(self._stop_publishing)
#         self.rightButton.released.connect(self._stop_publishing)
#         self.stopButton.pressed.connect(self._stop_publishing)

#         # self.sendGoalButton.clicked.connect(self.publish_path)
#         # self.loadCSVButton.clicked.connect(self.load_path)
#         # self.pauseButton.clicked.connect(self.toggle_pause_path)
#         self.path_msg = None
#         self.last_x = None
#         self.last_y = None
#         self.is_paused = False

#         # For RMSE
#         self.index = 0
#         self.rmse_sum = 0.0  # Sum of squared errors

#         # Waypoints as numpy for optimization
#         self.waypoints_np = None
#         self.got_initial_path = False
    
#     def goal_callback(self, msg: PoseStamped):
#         """Clear error history when a new goal is received."""
#         self.rmse_sum = 0.0
#         self.index = 0
#         self.setpoint_x.clear()
#         self.setpoint_y.clear()
#         self.current_x.clear()
#         self.current_y.clear()
#         self.error_history.clear()
#         self.error_plot_curve.setData([], [])
#         self.curve_setpoint.setData([], [])
#         self.curve_current.setData([], [])
#         self.get_logger().info("New goal received, cleared error history and plots.")

#         # Clear đồ thị
#         self.replanned_Path.setData([], [])     # Xóa đường live cũ
        
#         # Mở khóa để nhận đường gốc mới
#         self.got_initial_path = False


#     def path_callback(self, msg):
#         if not msg.poses:
#             return

#         # Lấy dữ liệu tọa độ từ msg
#         temp_x = []
#         temp_y = []
#         for pose_stamped in msg.poses:
#             temp_x.append(pose_stamped.pose.position.x)
#             temp_y.append(pose_stamped.pose.position.y)

#         # --- PHẦN 1: LUÔN CẬP NHẬT ĐƯỜNG LIVE (REPLANNING) ---
#         # Vẽ ngay đường mới nhất lên curve_live để mắt người nhìn thấy robot tránh vật cản
#         self.replanned_Path.setData(temp_x, temp_y)

#         # --- PHẦN 2: CHỈ LƯU ĐƯỜNG GỐC 1 LẦN (ĐỂ TÍNH RMSE) ---
#         if not self.got_initial_path:
#             # Lưu vào biến setpoint chính để tính toán RMSE trong amcl_callback
#             self.setpoint_x = temp_x
#             self.setpoint_y = temp_y
            
#             self.num_points = len(self.setpoint_x)
#             if self.num_points > 0:
#                 self.waypoints_np = np.column_stack((self.setpoint_x, self.setpoint_y))

#                 # Vẽ đường gốc lên curve_setpoint (Màu xanh)
#                 self.curve_setpoint.setData(self.setpoint_x, self.setpoint_y)
                
#                 # Khóa lại, không update phần này nữa
#                 self.got_initial_path = True
#                 self.get_logger().info("Đã lưu đường tham chiếu gốc (Reference Path).")

#     # def path_callback(self, msg):
#     #     """
#     #     Nhận, xử lý và vẽ dữ liệu đường đi (setpoints) từ tin nhắn nav_msgs/Path.
#     #     Hàm này được gọi mỗi khi có tin nhắn mới trên topic '/plan'.
#     #     """
#     #     # Khởi tạo lại danh sách tọa độ

#     #     self.setpoint_x = []
#     #     self.setpoint_y = []
        
#     #     try:
#     #         # Lặp qua tất cả các điểm (poses) trong tin nhắn Path
#     #         # msg.poses là một danh sách các 'geometry_msgs/PoseStamped'
#     #         if msg.poses:
#     #             for pose_stamped in msg.poses:
#     #                 # Trích xuất tọa độ x và y từ vị trí (position) của mỗi pose
#     #                 x = pose_stamped.pose.position.x
#     #                 y = pose_stamped.pose.position.y
                    
#     #                 self.setpoint_x.append(x)
#     #                 self.setpoint_y.append(y)

#     #         # Cập nhật tổng số điểm
#     #         self.num_points = len(self.setpoint_x)

#     #         # Build numpy waypoints for efficient distance calc
#     #         if self.num_points > 0:
#     #             self.waypoints_np = np.column_stack((self.setpoint_x, self.setpoint_y))
#     #         else:
#     #             self.waypoints_np = None

#     #         self.curve_setpoint.setData(self.setpoint_x, self.setpoint_y)

#     #     except Exception as e:
#     #         if hasattr(self, 'get_logger'):
#     #             self.get_logger().error(f"Lỗi khi xử lý và vẽ đường đi: {e}")
#     #         else:
#     #             # Dự phòng nếu không tìm thấy logger
#     #             print(f"Lỗi khi xử lý và vẽ đường đi: {e}")

#     def _publish_current_cmd(self):
#         """Publish current Twist command."""
#         twist = Twist()
#         twist.linear.x = self.current_linear
#         twist.angular.z = self.current_angular
#         self.cmd_pub.publish(twist)

#     def _start_publishing_forward(self):
#         self.current_linear = 0.1
#         self.current_angular = 0.0
#         if not self.cmd_timer.isActive():
#             self.cmd_timer.start()

#     def _start_publishing_backward(self):
#         self.current_linear = -0.1
#         self.current_angular = 0.0
#         if not self.cmd_timer.isActive():
#             self.cmd_timer.start()

#     def _start_publishing_left(self):
#         self.current_linear = 0.0
#         self.current_angular = 0.3
#         if not self.cmd_timer.isActive():
#             self.cmd_timer.start()

#     def _start_publishing_right(self):
#         self.current_linear = 0.0
#         self.current_angular = -0.3
#         if not self.cmd_timer.isActive():
#             self.cmd_timer.start()

#     def _stop_publishing(self):
#         if self.cmd_timer.isActive():
#             self.cmd_timer.stop()
#         twist = Twist()
#         twist.linear.x = 0.0
#         twist.angular.z = 0.0
#         self.cmd_pub.publish(twist)
#         self.get_logger().info("Published STOP cmd_vel")
#         self.current_linear = 0.0
#         self.current_angular = 0.0

#     def odom_callback(self, msg: Odometry):
#         """Update velocities and UI lights."""
#         try:
#             self.linear_vel = msg.twist.twist.linear.x
#             self.angular_vel = msg.twist.twist.angular.z
            
#             if abs(self.linear_vel) <= 0.01 and abs(self.angular_vel) <= 0.01:
#                 self.light_color_stop()
#             else:
#                 self.light_color_run()
#                 self.update_Velocity(self.linear_vel, self.angular_vel)
#         except Exception as e:
#             self.get_logger().error(f"Odom callback error: {e}")

#     def amcl_callback(self, msg: PoseWithCovarianceStamped):
#         """Process AMCL pose: update position, error, DB."""
#         try:
#             current_x = msg.pose.pose.position.x
#             current_y = msg.pose.pose.position.y
#             self.index += 1

#             min_error = float('inf')
#             # closest_waypoint_x, closest_waypoint_y = None, None
#             # 
#             if self.waypoints_np is not None and len(self.waypoints_np) > 0:
#                 # Vectorized distance calculation for efficiency
#                 distances = np.sqrt(np.sum((self.waypoints_np - np.array([current_x, current_y]))**2, axis=1)) 
#                 min_idx = np.argmin(distances)   
#                 min_error = distances[min_idx]


#             if current_x != self.last_x or current_y != self.last_y:
#                 self.update_current_position(current_x, current_y)
#                 # Use latest velocities from odom
#                 self.insert_data(current_x, current_y, min_error, self.linear_vel, self.angular_vel)
#                 self.update_error_plot(min_error)


#                 self.rmse_sum += min_error ** 2
#                 rmse_value = sqrt(self.rmse_sum / self.index) if self.index > 0 else 0.0
#                 # self.rmseLabel.setText(f"RMSE: {rmse_value:.4f}")
#                 # self.rmseLabel.adjustSize()
#                 self.get_logger().info(f"Current RMSE: {rmse_value:.4f}")


#             self.last_x = current_x
#             self.last_y = current_y
#         except Exception as e:
#             self.get_logger().error(f"AMCL callback error: {e}")

#     # def fault_callback(self, msg: Bool):
#     #     """Handle fault message."""
#     #     if msg.data:
#     #         self.light_color_error()
#     #         self.get_logger().warn("Fault detected!")


# def main(args=None):
#     rclpy.init(args=args)
#     app = QApplication([])
#     node = RobotUINode()
#     node.show()
#     # Dùng QTimer để gọi spin_once định kỳ
#     spin_timer = QTimer()
#     spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
#     spin_timer.start(10)  # 10ms ~ 100Hz
#     sys.exit(app.exec_())

# if __name__ == "__main__":
#     main()


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
import signal

class RobotUINode(Node, RobotUI):
    def __init__(self):
        Node.__init__(self, 'robot_ui_node')
        self.get_logger().info("Robot UI Node Initialized")
        RobotUI.__init__(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.group_Reent_ = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(Twist, '/UI/cmd_vel', 10)
        # self.path_pub = self.create_publisher(Path, '/plan', 10)


        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10, callback_group=self.group_Reent_)
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10, callback_group=self.group_Reent_)
        # self.fault_sub_ = self.create_subscription(Bool, '/liem_controller/fault', self.fault_callback, 10, callback_group=self.group_Reent_)
        self.amcl_sub_ = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10, callback_group=self.group_Reent_)
        self.goal_sub_ = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10, callback_group=self.group_Reent_)

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

        # self.sendGoalButton.clicked.connect(self.publish_path)
        # self.loadCSVButton.clicked.connect(self.load_path)
        # self.pauseButton.clicked.connect(self.toggle_pause_path)
        self.path_msg = None
        self.last_x = None
        self.last_y = None
        self.is_paused = False

        # For RMSE
        self.index = 0
        self.rmse_sum = 0.0  # Sum of squared errors

        # Waypoints as numpy for optimization
        self.waypoints_np = None
        self.got_initial_path = False

    def goal_callback(self, msg: PoseStamped):
        """Clear error history when a new goal is received."""
        self.rmse_sum = 0.0
        self.index = 0
        self.setpoint_x.clear()
        self.setpoint_y.clear()
        self.current_x.clear()
        self.current_y.clear()
        self.error_history.clear()
        self.error_plot_curve.setData([], [])
        self.curve_setpoint.setData([], [])
        self.curve_current.setData([], [])
        self.get_logger().info("New goal received, cleared error history and plots.")

        # Clear đồ thị
        self.replanned_Path.setData([], [])     # Xóa đường live cũ
        
        # Mở khóa để nhận đường gốc mới
        self.got_initial_path = False

    def path_callback(self, msg):
        if not msg.poses:
            return

        # Lấy dữ liệu tọa độ từ msg
        temp_x = []
        temp_y = []
        for pose_stamped in msg.poses:
            temp_x.append(pose_stamped.pose.position.x)
            temp_y.append(pose_stamped.pose.position.y)

        # --- PHẦN 1: LUÔN CẬP NHẬT ĐƯỜNG LIVE (REPLANNING) ---
        # Vẽ ngay đường mới nhất lên curve_live để mắt người nhìn thấy robot tránh vật cản
        self.replanned_Path.setData(temp_x, temp_y)

        # --- PHẦN 2: CHỈ LƯU ĐƯỜNG GỐC 1 LẦN (ĐỂ TÍNH RMSE) ---
        if not self.got_initial_path:
            # Lưu vào biến setpoint chính để tính toán RMSE trong amcl_callback
            self.setpoint_x = temp_x
            self.setpoint_y = temp_y
            
            self.num_points = len(self.setpoint_x)
            if self.num_points > 0:
                self.waypoints_np = np.column_stack((self.setpoint_x, self.setpoint_y))

                # Vẽ đường gốc lên curve_setpoint (Màu xanh)
                self.curve_setpoint.setData(self.setpoint_x, self.setpoint_y)
                
                # Khóa lại, không update phần này nữa
                self.got_initial_path = True

    # def path_callback(self, msg):
    #     """
    #     Nhận, xử lý và vẽ dữ liệu đường đi (setpoints) từ tin nhắn nav_msgs/Path.
    #     Hàm này được gọi mỗi khi có tin nhắn mới trên topic '/plan'.
    #     """
    #     # Khởi tạo lại danh sách tọa độ
    #     self.rmse_sum = 0.0
    #     self.index = 0
    #     self.setpoint_x = []
    #     self.setpoint_y = []
        
    #     try:
    #         # Lặp qua tất cả các điểm (poses) trong tin nhắn Path
    #         # msg.poses là một danh sách các 'geometry_msgs/PoseStamped'
    #         if msg.poses:
    #             for pose_stamped in msg.poses:
    #                 # Trích xuất tọa độ x và y từ vị trí (position) của mỗi pose
    #                 x = pose_stamped.pose.position.x
    #                 y = pose_stamped.pose.position.y
                    
    #                 self.setpoint_x.append(x)
    #                 self.setpoint_y.append(y)

    #         # Cập nhật tổng số điểm
    #         self.num_points = len(self.setpoint_x)

    #         # Build numpy waypoints for efficient distance calc
    #         if self.num_points > 0:
    #             self.waypoints_np = np.column_stack((self.setpoint_x, self.setpoint_y))
    #         else:
    #             self.waypoints_np = None

    #         self.curve_setpoint.setData(self.setpoint_x, self.setpoint_y)
    #         self.error_history.clear()

    #     except Exception as e:
    #         if hasattr(self, 'get_logger'):
    #             self.get_logger().error(f"Lỗi khi xử lý và vẽ đường đi: {e}")
    #         else:
    #             # Dự phòng nếu không tìm thấy logger
    #             print(f"Lỗi khi xử lý và vẽ đường đi: {e}")

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

    def odom_callback(self, msg: Odometry):
        """Update velocities and UI lights."""
        try:
            self.linear_vel = msg.twist.twist.linear.x
            self.angular_vel = msg.twist.twist.angular.z
            
            if abs(self.linear_vel) <= 0.01 and abs(self.angular_vel) <= 0.01:
                self.light_color_stop()
            else:
                self.light_color_run()
                self.update_Velocity(self.linear_vel, self.angular_vel)
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
            # 
            if self.waypoints_np is not None and len(self.waypoints_np) > 0:
                # Vectorized distance calculation for efficiency
                distances = np.sqrt(np.sum((self.waypoints_np - np.array([current_x, current_y]))**2, axis=1)) 
                min_idx = np.argmin(distances)   
                min_error = distances[min_idx]


            if current_x != self.last_x or current_y != self.last_y:
                self.update_current_position(current_x, current_y)
                # Use latest velocities from odom
                self.insert_data(current_x, current_y, min_error, self.linear_vel, self.angular_vel)
                self.update_error_plot(min_error)


                self.rmse_sum += min_error ** 2
                rmse_value = sqrt(self.rmse_sum / self.index) if self.index > 0 else 0.0
                # self.rmseLabel.setText(f"RMSE: {rmse_value:.4f}")
                # self.rmseLabel.adjustSize()
                self.get_logger().info(f"Current RMSE: {rmse_value:.4f}")


            self.last_x = current_x
            self.last_y = current_y
        except Exception as e:
            self.get_logger().error(f"AMCL callback error: {e}")

    # def fault_callback(self, msg: Bool):
    #     """Handle fault message."""
    #     if msg.data:
    #         self.light_color_error()
    #         self.get_logger().warn("Fault detected!")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])
    node = RobotUINode()
    node.show()
    # Dùng QTimer để gọi spin_once định kỳ
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(10)  # 10ms ~ 100Hz
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()