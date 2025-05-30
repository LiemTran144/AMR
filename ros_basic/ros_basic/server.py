#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosbasic_move_msgs.srv import Control
import math
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
import sys

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")
        self.group_Reent_ = ReentrantCallbackGroup()


        self.service_ = self.create_service(Control, "control", self.serviceCallback, callback_group=self.group_Reent_)
        self.get_logger().info("Service control Ready")
        self.pub_ = self.create_publisher(Twist, "/nhatbot_controller/cmd_vel", 10)
        self.sub_ = self.create_subscription(Odometry, "/nhatbot_controller/odom", self.msgCallback, 10, callback_group=self.group_Reent_)
        self.sub_ = self.create_subscription(LaserScan, "/scan", self.laserCallback, 10, callback_group=self.group_Reent_)

        self.initialized_pose = None
        self.current_pose = None
        self.d = None
        self.kc = 0.0
        self.is_started = True

    def msgCallback(self,msg):
 
        if self.initialized_pose is None:
            self.initialized_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.current_pose  = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.d = math.sqrt((self.current_pose[0] - self.initialized_pose[0]) ** 2 +(self.current_pose[1] - self.initialized_pose[1]) ** 2)
        # print(f"Khoang cach: {self.d}")
    def laserCallback(self,data):
        
        data_laser = data.ranges
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        data_laser = data.ranges
        # print(f"Fixed angle_increment: {fixed_angle_increment}")

        target_angle = 0.0  # Góc 0 độ
        index = int((target_angle - angle_min) / angle_increment)
 
        if 0 <= index < len(data_laser):
            distance_at_0_degree = data_laser[index]
            if distance_at_0_degree >=9:
                return
            self.kc = distance_at_0_degree
            print(f"Khoảng cách tại góc 0 độ: {distance_at_0_degree} m")
        else:
            print("Chỉ số không hợp lệ hoặc không có dữ liệu tại góc 0 độ.")

    def serviceCallback(self, req, res):

        self.initialized_pose = None
        self.d = 0.0
        if self.initialized_pose is None or self.current_pose is None:
            return
        cmd = Twist()
        
        self.quangduong = req.quangduong
        self.khoangcach = req.khoangcach
        self.get_logger().info("Service Response quangduong: %f" % self.quangduong)
        self.get_logger().info("Service Response khoangcach: %f" % self.khoangcach)
        self.is_started = True

        while self.is_started:
            print(f"Khoang cach vat can: {self.kc}")
            if self.kc > self.khoangcach:
                if self.d < self.quangduong:  # Cho robot di chuyen 
                    cmd.linear.x = 0.1
                    cmd.angular.z = 0.0
                    print("Quang duong di duoc(service):", self.d)
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    print("Quang duong di duoc(service):", self.d)
                    self.is_started = False
                           
            else:  # Dung robot
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                print("Quang duong di duoc(service):", self.d)
                res.notification = "Robot gap vat can"
                res.success = False
               
            time.sleep(0.1)
            self.pub_.publish(cmd)

        res.notification = "Robot gap vat can"
        res.success = True
        return res

def main():
    try:
        rclpy.init()
        simple_service_server = SimpleServiceServer()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(simple_service_server)
        try:
            executor.spin()
        finally: 
            simple_service_server.destroy_node()
            executor.shutdown()
    except KeyboardInterrupt:
        pass
        simple_service_server.get_logger().info('Ctrl+C received - exiting...')
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        simple_service_server.get_logger().info('controller node is shutdowning')
        rclpy.shutdown()

if __name__ == '__main__':
    main()