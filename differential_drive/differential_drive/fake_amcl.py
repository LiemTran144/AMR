#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import pandas as pd
import numpy as np
from rclpy.qos import QoSProfile
import math

class FakeAMCLPublisher(Node):
    def __init__(self):
        super().__init__('fake_amcl_publisher')
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            QoSProfile(depth=10)
        )
        timer_period = 0.1  # 0.1s = 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load waypoints from CSV
        path = '/home/liemtran/liem_ws/src/user_interface/data/waypoint0509.csv'
        self.df = pd.read_csv(path)

        # Tạo thêm điểm nội suy để có nhiều điểm hơn
        self.interpolated_path = self.interpolate_path(self.df, step=0.02)  # 0.02m mỗi bước
        self.index = 0

    def interpolate_path(self, df, step=0.01):
        # Tạo danh sách điểm nội suy mượt hơn
        new_points = []
        for i in range(len(df) - 1):
            x0, y0, h0 = df.iloc[i]
            x1, y1, h1 = df.iloc[i+1]
            distance = math.hypot(x1 - x0, y1 - y0)
            steps = max(int(distance / step), 1)
            for j in range(steps):
                t = j / steps
                x = x0 + t * (x1 - x0)
                y = y0 + t * (y1 - y0)
                heading = h0 + t * (h1 - h0)
                # thêm nhiễu nhỏ
                noise_x = np.random.normal(0, 0.05)
                noise_y = np.random.normal(0, 0.05)
                noise_heading = np.random.normal(0, 0.01)
                new_points.append((x + noise_x, y + noise_y, heading + noise_heading))
        return new_points

    def timer_callback(self):
        if self.index >= len(self.interpolated_path):
            self.index = len(self.interpolated_path) - 1  # Dừng lại ở điểm cuối

        x, y, theta = self.interpolated_path[self.index]
        self.index += 1

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Optional: Add some covariance if needed
        msg.pose.covariance[0] = 0.01  # x
        msg.pose.covariance[7] = 0.01  # y
        msg.pose.covariance[35] = 0.05  # yaw

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published fake AMCL pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = FakeAMCLPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
