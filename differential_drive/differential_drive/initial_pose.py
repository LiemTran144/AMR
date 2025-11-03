#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')

        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Tạo message
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Pose giống với dữ liệu bạn đưa
        msg.pose.pose.position.x = 8.233
        msg.pose.pose.position.y = 2.659
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.9998774731132284
        msg.pose.pose.orientation.w = 0.015653713958831

        # Covariance: phải có đúng 36 phần tử float
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]

        self.pose_msg = msg

        # Publish 1 lần duy nhất
        self.timer = self.create_timer(1.0, self.publish_once)
        self.published = False

    def publish_once(self):
        if not self.published:
            self.pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.pose_msg)
            self.get_logger().info('✅ Published /initialpose successfully')
            self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
