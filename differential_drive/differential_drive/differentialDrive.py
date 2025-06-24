#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from zlac8015d import ZLAC8015D_API
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Bool
# from drive_msgs import zlacStatus

class differentialDriveNode(Node):

    def __init__(self):
        super().__init__('differentialDriveNode')

        self.declare_parameter('port', '/dev/zlac_8015d')
        port = self.get_parameter('port').get_parameter_value().string_value
        print("Port:", port)

        # Khởi tạo driver ZLAC8015D
        self.driver = ZLAC8015D_API(port,self)
        self.driver.clear_position(3) # clear both

        # print("travel:", self.driver.get_wheels_travelled())

        # Thiết lập chế độ vận tốc và bật động cơ (nếu cần thiết)
        self.driver.set_mode(self.driver.VEL_CONTROL)
        self.driver.enable_motor()
        self.driver.set_rpm(0, 0)
        self.driver.set_accel_time(1000,1000)
        self.driver.set_decel_time(1000,1000) 

        self.sub_rpm = self.create_subscription(Float64MultiArray, '/liem_controller/wheel_rotational_vel',self.wheel_callback,10)
        self.wheel_JointState_pub_ = self.create_publisher(JointState, 'JointState',10)
        self.timer_JointState_ = self.create_timer(0.1, self.pub_JointState_Callback)

        # self.fault_pub_ = self.create_publisher(Bool, '/liem_controller/fault', 10)
        # self.timer_Fault_ = self.create_timer(0.1, self.pub_Fault_Callback)

        # self.driverStatus_pub_ = self.create_publisher(zlacStatus, '/liem_controller/driver_status', 10)
        # self.timer_driverStatus_ = self.create_timer(0.1, self.pub_driverStatus_Callback)

    def pub_Fault_Callback(self):
        fault_msg = Bool()
        left_fault, right_fault = self.driver.get_fault_code()
        left_fault_flag, left_fault_code = left_fault
        right_fault_flag, right_fault_code = right_fault
        if left_fault_flag or right_fault_flag:
            self.get_logger().error(f"Left Wheel Fault: {left_fault_code}, Right Wheel Fault: {right_fault_code}")
            fault_msg.data = True
        else:
            fault_msg.data = False
        self.fault_pub_.publish(fault_msg)



    def pub_JointState_Callback(self):

        msg_wheel_JointState_ = JointState()   
        msg_wheel_JointState_.velocity = list(self.driver.get_angular_velocity())  # rad/s
        msg_wheel_JointState_.position = list(self.driver.get_wheels_travelled())  # rad
        msg_wheel_JointState_.header.stamp = self.get_clock().now().to_msg()
        self.wheel_JointState_pub_.publish(msg_wheel_JointState_)
        
    def wheel_callback(self, msg: Float64MultiArray):
        """
        Callback khi nhận lệnh vận tốc (Twist).
        linear.x = vận tốc bánh trái (m/s),
        linear.y = vận tốc bánh phải (m/s).
        """

        if len(msg.data) != 2:
            self.get_logger().error("Invalid RPM data received. Expected 2 values.")

            return
        left_rpm  = msg.data[0] * (60 / (2*np.pi))
        right_rpm = msg.data[1] * (60 / (2*np.pi))

        # Gửi lệnh điều khiển RPM tới driver
        self.driver.set_rpm(-left_rpm,right_rpm)

    def exit_driver(self):

        """Stop the motor safely when ROS shuts down"""
        if self.driver:
            try:
                print("🛑 Stopping motor immediately!")
                self.driver.set_rpm(0, 0)  
                self.driver.disable_motor()
                self.driver.close_connect()
            except Exception as e:
               print(f"❌ Failed to stop motor: {str(e)}")

    # def pub_driverStatus_Callback(self):
    #     # zlac_status = zlacStatus()
    #     # motor_temperature = self.bldcMotor.get_motor_temperature()
    #     zlac_status.battery_voltage = float(self.driver.get_battery_voltage())
    #     zlac_status.brake_state = str(self.driver.get_brake_state())
    #     zlac_status.control_mode = int(self.driver.get_mode())
    #     zlac_status.driver_temp = float(self.driver.get_driver_temperature())
    #     # zlac_status.vehicle_state = str(self.motor_states)

    #     self.driverStatus_pub_.publish(zlac_status)

def main(args=None):
    try:
        rclpy.init(args=args)

        node = differentialDriveNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        node.exit_driver()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Driver Node has been shut down.")

if __name__ == '__main__':
    main()
