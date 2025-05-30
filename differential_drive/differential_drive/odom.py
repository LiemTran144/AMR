#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster   
from rclpy.duration import Duration
from rclpy.time import Time
import numpy as np
import math
from tf_transformations import quaternion_from_euler

class odom(Node):
    def __init__(self): # constructor 
        super().__init__('odom')
        print("odom initialize")
        
        self.declare_parameter('wheel_radius', 0.0535)
        self.declare_parameter('wheel_separator', 0.45)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separator = self.get_parameter('wheel_separator').value

        self.vel_sub_ = self.create_subscription(JointState, '/JointState',self.jointCallback, 10)
        
        self.odom_pub_ = self.create_publisher(Odometry, "liem_controller/odom", 10)

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_link"

        self.prev_time_ = self.get_clock().now()
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.linear_filtered = 0.0
        self.angular_filtered = 0.0


    def jointCallback(self, msg):
        print("jointCallback: ", msg.position)
       
        if hasattr(self, "block_update_until") and self.get_clock().now() < self.block_update_until:
            return

        if not msg.position or len(msg.position) < 2:
            return

        if any(math.isnan(pos) or math.isinf(pos) for pos in msg.position):
            return

        # Tính dt (delta thời gian giữa hai lần cập nhật)
        current_time = Time.from_msg(msg.header.stamp)
        dt = max((current_time - self.prev_time_).nanoseconds / 1e9, 1e-6)  # Đảm bảo dt >= 1µs
        self.prev_time_ = current_time

        if msg.position[0] == 0.0 and msg.position[1] == 0.0:
            if self.left_wheel_prev_pos_ != 0.0 or self.right_wheel_prev_pos_ != 0.0:
                self.reset_odom()
            return 

        dp_left = msg.position[0] - self.left_wheel_prev_pos_
        dp_right = msg.position[1] - self.right_wheel_prev_pos_

        # Ngưỡng bỏ qua nhiễu nhỏ
        if abs(dp_left) < 0.0005 and abs(dp_right) < 0.0005:
            return 

        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]

        phi_left = dp_left / dt
        phi_right = dp_right / dt

        self.update_odometry(phi_left, phi_right, dt)

    def update_odometry(self, phi_left, phi_right, dt):

        self.linear_velocity = self.wheel_radius * (phi_left + phi_right) / 2.0
        self.angular_velocity = self.wheel_radius * (phi_right - phi_left) / self.wheel_separator

        self.alpha = 0.2
        self.linear_filtered = self.alpha * self.linear_velocity + (1 - self.alpha) * self.linear_filtered
        self.angular_filtered = self.alpha * self.angular_velocity + (1 - self.alpha) * self.angular_filtered

        if (abs(self.linear_filtered) < 0.004):
            self.linear_filtered = 0.0
        if (abs(self.angular_filtered) < 0.004):
            self.angular_filtered = 0.0

        self.x_ += self.linear_filtered * math.cos(self.theta_) * dt
        self.y_ += self.linear_filtered * math.sin(self.theta_) * dt

        self.theta_ -= self.angular_filtered * dt
        self.theta_ = math.atan2(math.sin(self.theta_), math.cos(self.theta_))

        self.publish_odom()

    def publish_odom(self):
        
        q = quaternion_from_euler(0, 0, self.theta_)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x_
        odom_msg.pose.pose.position.y = self.y_
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.linear_filtered
        odom_msg.twist.twist.angular.z = self.angular_filtered

        self.odom_pub_.publish(odom_msg)
        print("Odometry published")

        # #TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_

        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]

        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)
    
    def reset_odom(self):
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0

        self.block_update_until = self.get_clock().now() + Duration(seconds=0.1)
        self.get_logger().info("Odometry reset")
        self.publish_odom()

def main(args=None):
    try:
        rclpy.init(args=args)

        node = odom()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ =='__main__':
    main()
