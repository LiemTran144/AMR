#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class pub_control(Node):
    def __init__(self): # constructor 
        super().__init__('pub_control')
        print("pub_control Node initialize")

        self.pub_ = self.create_publisher(Float64MultiArray, '/wheel_rpm' ,10)
        self.timer_ = self.create_timer(1, self.timerCallback)

    
    def timerCallback(self):

        rpm = Float64MultiArray()
        left_rpm = 30.0
        right_rpm = 30.0
        rpm.data = [left_rpm, right_rpm]

        self.pub_ .publish(rpm)

def main(args=None):
    try:
        rclpy.init(args=args)

        node = pub_control()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ =='__main__':
    main()