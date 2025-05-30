#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from zlac8015d import ZLAC8015D_API
from sensor_msgs.msg import JointState

class FeedbackNode(Node):

    def __init__(self):
        super().__init__('velocity_feedback')
        # Đọc tham số cổng kết nối từ launch file hoặc tham số node
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value
        print("Port:", port)

        # Khởi tạo driver ZLAC8015D
        self.driver = ZLAC8015D_API('/dev/ttyUSB0')
        self.driver.clear_position(3) # clear both

        print("travel:", self.driver.get_wheels_travelled())

        # Thiết lập chế độ vận tốc và bật động cơ (nếu cần thiết)
        self.driver.set_mode(self.driver.VEL_CONTROL)
        self.driver.enable_motor()
        self.driver.set_rpm(0, 0)
        self.driver.set_accel_time(1000,1000)
        self.driver.set_decel_time(1000,1000) 

        self.sub_rpm = self.create_subscription(Float64MultiArray, '/wheel_rpm',self.wheel_callback,10)

        self.wheel_JointState_pub_ = self.create_publisher(JointState, 'JointState',10)
        self.timer_JointState_ = self.create_timer(0.1, self.pub_JointState_Callback)
        # # Publisher cho feedback vận tốc bánh xe
        # self.publisher = self.create_publisher(Twist, '/wheel_velocity_feedback', 10)
        # # Timer: mỗi 0.1s đọc và publish dữ liệu
        # self.timer = self.create_timer(0.1, self.publish_feedback)

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
        left_rpm  = msg.data[0]
        right_rpm = msg.data[1]

        # Gửi lệnh điều khiển RPM tới driver
        self.driver.set_rpm(-left_rpm, right_rpm)

    # def publish_feedback(self):
    #     # Lấy vận tốc thực tế (m/s) từ driver
    #     vl, vr = self.driver.get_linear_velocities()
    #     # Tạo và publish Twist message
    #     msg = Twist()
    #     msg.linear.x = vl if vl is not None else 0.0
    #     msg.linear.y = vr if vr is not None else 0.0
    #     self.publisher.publish(msg)

def main(args=None):
    try:
        rclpy.init(args=args)

        node = FeedbackNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
