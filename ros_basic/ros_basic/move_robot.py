#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import time

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

from rosbasic_move_msgs.srv import Control



class Ros_Publisher(Node):
    def __init__(self): # constructor 
        super().__init__('move_robot')
        print("Node initialize")

        self.pub_ = self.create_publisher(Twist, "/nhatbot_controller/cmd_vel", 10)
        self.sub_ = self.create_subscription(Odometry, "/nhatbot_controller/odom", self.msgCallback, 10)
        self.sub_ = self.create_subscription(LaserScan, "/scan", self.laserCallback, 10)
        


        # self.declare_parameter("quangduong", 3)
        # self.declare_parameter("khoangcach", 1.0)
        # self.quangduong = self.get_parameter("quangduong").value
        # self.khoangcach = self.get_parameter("khoangcach").value




        self.initialized_pose = None
        self.current_pose = None
        self.d = None
        self.kc = 0.0

    def responseCallback(self, future):
        self.quangduong = future.result().quangduong1
        self.khoangcach = future.result().khoangcach1
        self.get_logger().info("Service Response quangduong: %f" % self.quangduong)
        self.get_logger().info("Service Response khoangcach: %f" % self.khoangcach)

    def laserCallback(self,data):
        
        data_laser = data.ranges
        num_ranges = len(data.ranges)

        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        range_min = data.range_min
        range_max = data.range_max

        data_laser = data.ranges
        middle_angle = 0.0 
        target_num_points = 720
        total_angle = angle_max - angle_min
        fixed_angle_increment = total_angle / target_num_points
        # print(f"Fixed angle_increment: {fixed_angle_increment}")

        target_angle = 0.0  # Góc 0 độ
        index = int((target_angle - angle_min) / angle_increment)
        print(f"Khoang cach max: {self.khoangcach}")
        if 0 <= index < len(data_laser):
            distance_at_0_degree = data_laser[index]
            if distance_at_0_degree >=9:
                return
            self.kc = distance_at_0_degree
            print(f"Khoảng cách tại góc 0 độ: {distance_at_0_degree} m")
        else:
            print("Chỉ số không hợp lệ hoặc không có dữ liệu tại góc 0 độ.")



    def timerCallback(self):
        if self.initialized_pose is None or self.current_pose is None:
            return
        cmd = Twist()
        
        if self.kc < self.khoangcach:           
            if self.d < self.quangduong:
                cmd.linear.x = 0.1
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = -0.1
                cmd.angular.z = 0.0
            # print("d:", self.d)
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.pub_ .publish(cmd)

    def msgCallback(self,msg):
 
        if self.initialized_pose is None:
            self.initialized_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.current_pose  = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
        self.d = math.sqrt((self.current_pose[0] - self.initialized_pose[0]) ** 2 +(self.current_pose[1] - self.initialized_pose[1]) ** 2)


        # print("Init pose: ",self.initialized_pose)
        # print("Current pose: ",self.current_pose )
def main(args=None):
    try:
        rclpy.init(args=args)

        node = Ros_Publisher()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ =='__main__':
    main()


