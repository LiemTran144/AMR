#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class Ros_Subscriber(Node):
    def __init__(self): # constructor 
        super().__init__('ros_subscriber')
        print("Node initialize")
        self.sub_ = self.create_subscription(Int16, "count", self.msgCallback, 10)

    def msgCallback(self, msg):
        print("msg: ", msg)
        
def main(args=None):
    try:
        rclpy.init(args=args)

        node = Ros_Subscriber()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ =='__main__':
    main()
