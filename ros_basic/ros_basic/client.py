#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosbasic_msgs.srv import AddTwoInts
from rosbasic_move_msgs.srv import Control
import sys 



class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__("simple_service_client")
        self.client_ = self.create_client(Control, "control")
        
        # verify that actually this server is running and is available to receive new requests
        # while not self.client_.wait_for_service(timeout_sec=1.0) doesn't return true -> we are going to proceed to the following instructions And so every time that this function
        # returns false here let's wait for the service a total amount

        while not self.client_.wait_for_service(timeout_sec=1.0):  # wait for service function, 

            self.get_logger().info("Service not available, waiting again...")
        # out loop if wait_for_service = true
        
        self.req_ = AddTwoInts.Request() # message type
        self.req_.quangduong = 1.0
        self.req_.khoangcach = 0.6

        self.future_ = self.client_.call_async(self.req_) # send the request meg to service server and immediately returns a result 
        # future variable implements a mechanism to store the output of asynchronous functions like this one

        self.future_.add_done_callback(self.responseCallback)
   
    def responseCallback(self, future):
        self.get_logger().info("Service Response %s" % future.result().notification)

def main():
    rclpy.init()
    # use sys lib to access to the parameter of the main. 
    # if the script has been started correctly, so with the correct number of arguments 
    simple_service_client = SimpleServiceClient() # argv[1] only name of the script
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()