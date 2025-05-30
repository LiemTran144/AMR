#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from zlac8015d import ZLAC8015D_API
from sensor_msgs.msg import JointState

class positionNode(Node):
    def __init__(self): # constructor 
        super().__init__('positionNode')
        print("positionNode initialize")
        self.sub_ = self.create_subscription(JointState,'JointState' , self.msgCallback, 10)

    def msgCallback(self, msg:JointState):

        print("position:", msg.position)       
        # print("msg: ", msg)
        
def main(args=None):
    try:
        rclpy.init(args=args)

        node = positionNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ =='__main__':
    main()
