#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np


class speed_controlNode(Node):
    def __init__(self): # constructor 
        super().__init__('speed_controlNode')
        print("speed_controlNode initialize")
        
        self.declare_parameter('wheel_radius', 0.0535)
        self.declare_parameter('wheel_separator', 0.45)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separator = self.get_parameter('wheel_separator').value

        self.vel_sub_ = self.create_subscription(Twist, "/liem_controller/cmd_vel", self.velCallback, 10)

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, '/liem_controller/wheel_rotational_vel' ,10)

    def velCallback(self, msg):

        # rad/s
        left = (msg.linear.x + (self.wheel_separator / 2) * (msg.angular.z)) / self.wheel_radius
        right = (msg.linear.x - (self.wheel_separator / 2) * (msg.angular.z)) / self.wheel_radius

        rpm_msgs = Float64MultiArray()
        rpm_msgs.data = [left, right]
        # print(f"left: {left}, right: {right}")

        self.wheel_cmd_pub_.publish(rpm_msgs)
        
def main(args=None):
    try:
        rclpy.init(args=args)

        node = speed_controlNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ =='__main__':
    main()
