#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
import csv
import os
import math

class ClickedWaypointNode(Node):
    def __init__(self):
        super().__init__('clicked_waypoint_node')
        # Declare parameters and retrieve their values
        self.declare_parameter('csv_path', '/home/liemtran/liem_ws/src/user_interface/user_interface')
        self.declare_parameter('csv_file', 'baocao.csv')
        self.declare_parameter('interpolation_precision', 10)
        self.declare_parameter('clicked_point_topic', '/clicked_point')

        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        self.precision = self.get_parameter('interpolation_precision').get_parameter_value().integer_value
        clicked_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value

        # Determine full CSV file path
        if os.path.isabs(csv_path):
            self.csv_full_path = os.path.join(csv_path, csv_file)
        else:
            self.csv_full_path = os.path.join(os.getcwd(), csv_path, csv_file)
        print(f"CSV file path: {self.csv_full_path}")



        # Initialize the CSV file with a header (overwrites any existing file)
        with open(self.csv_full_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y', 'heading'])

        # Initialize previous point storage
        self.prev_point = None

        # Initialize the line-strip marker for waypoints
        self.marker_pub = self.create_publisher(Marker, '/liem_waypoint_marker', 10)
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = 'waypoints'
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05  # line width
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0    # fully opaque
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        # Subscribe to the clicked point topic
        self.subscription = self.create_subscription(
            PointStamped,
            clicked_topic,
            self.clicked_point_callback,
            10
        )

    def clicked_point_callback(self, msg: PointStamped):
        """Handle a new clicked point: interpolate, save to CSV, and update marker."""
        new_point = msg.point

        # Open CSV in append mode to write new waypoints
        with open(self.csv_full_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if self.prev_point is not None:
                # Calculate differences and heading between previous and new point
                dx = new_point.x - self.prev_point.x
                dy = new_point.y - self.prev_point.y
                dz = new_point.z - self.prev_point.z
                heading = math.atan2(dy, dx)

                # Interpolate intermediate points
                for i in range(1, self.precision + 1):
                    fraction = i / (self.precision + 1)
                    x_interp = self.prev_point.x + fraction * dx
                    y_interp = self.prev_point.y + fraction * dy
                    z_interp = self.prev_point.z + fraction * dz

                    writer.writerow([x_interp, y_interp, heading])
                    # interp_point = Point(x=x_interp, y=y_interp, z=0.0)
                    interp_point = Point(x=x_interp, y=y_interp, z=z_interp)
                    self.marker.points.append(interp_point)

                # Write the new point
                writer.writerow([new_point.x, new_point.y, heading])
                #moi them
                marker_point = Point(x=new_point.x, y=new_point.y, z=new_point.z)
                self.marker.points.append(marker_point)

            else:
                # First point (no previous point to interpolate from)
                writer.writerow([new_point.x, new_point.y, 0.0])
                #   moi them
                marker_point = Point(x=new_point.x, y=new_point.y, z=new_point.z)
                self.marker.points.append(marker_point)

            # Add the new point to the marker (always do this)
            marker_point = Point(x=new_point.x, y=new_point.y, z=0.0)
            self.marker.points.append(marker_point)

        # Update previous point to current
        self.prev_point = new_point

        # Publish the updated marker (set current timestamp)
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = ClickedWaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()