#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import csv
import os
class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # self.declare_parameter('csv_path', '/home/liemtran/liem_ws/src/user_interface/data')
        self.declare_parameter('csv_path', '/home/liemtran/liem_ws/src/user_interface/user_interface')

        # self.declare_parameter('csv_file', 'waypoints265.csv')
        self.declare_parameter('csv_file', 'waypoints295.csv')
        csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        self.csv_full_path = os.path.join(csv_path, csv_file)

        # Tạo publisher cho topic /path
        self.publisher_ = self.create_publisher(Path, '/liem_path', 10)


        # path: Header: frame_id: 'map'
        #               stamped: 
        #       poses: position(point)
        #              orientation(quaternion)  



        self.timer = self.create_timer(1.0, self.publish_path)

        # with open(csv_file, 'r') as file:
        #     reader = csv.reader(file)
        #     next(reader) 
        self.path_published = False
    def publish_path(self):
        if self.path_published:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        poses = []

        try:
            with open(self.csv_full_path, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    # if len(row) >= 2:
                    pose = PoseStamped()
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.header.frame_id = 'map'

                    pose.pose.position.x = float(row['x'])
                    pose.pose.position.y = float(row['y'])
                    pose.pose.position.z = 0.0

                    pose.pose.orientation.w = 1.0

                    poses.append(pose)

            path_msg.poses = poses
            print(path_msg.poses)
            self.publisher_.publish(path_msg)
            self.get_logger().info(f'✅ Published path with {len(poses)} poses from CSV.')

            self.path_published = True

        except Exception as e:
            self.get_logger().error(f'❌ Failed to read CSV or publish: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
