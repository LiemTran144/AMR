#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class Ros_Publisher(Node):
    def __init__(self): # constructor 
        super().__init__('ros_publisher')
        print("Node initialize")

        self.pub_ = self.create_publisher(Int16, "count", 10)
        self.timer_ = self.create_timer(1, self.timerCallback)
        self.count = 0
    
    def timerCallback(self):
        self.count +=1 
        self.get_logger().info(f'count: {self.count}')
        value = Int16()
        value.data = self.count
        self.pub_ .publish(value)


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





