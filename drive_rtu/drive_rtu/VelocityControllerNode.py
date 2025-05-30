#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from zlac8015d import ZLAC8015D_API
import math

class VelocityControllerNode(Node):
    """
    Node này điều khiển vận tốc bánh xe trái/phải dựa trên Twist message.
    """
    def __init__(self):
        super().__init__('velocity_controller')
        # Đọc tham số cổng kết nối từ launch file hoặc tham số node
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value

        # Khởi tạo driver ZLAC8015D
        self.driver = ZLAC8015D_API(port)
        # Thiết lập chế độ vận tốc và bật động cơ
        self.driver.set_mode(self.driver.VEL_CONTROL)
        self.driver.enable_motor()

        # Subscriber cho lệnh vận tốc bánh xe
        self.subscription = self.create_subscription(
            Twist,
            '/wheel_velocity_command',
            self.cmd_callback,
            10)

    def cmd_callback(self, msg: Twist):
        """
        Callback khi nhận lệnh vận tốc (Twist).
        linear.x = vận tốc bánh trái (m/s),
        linear.y = vận tốc bánh phải (m/s).
        """
        left_vel = msg.linear.x
        right_vel = msg.linear.y

        # Chuyển đổi tốc độ từ m/s sang RPM
        # Công thức: rpm = V * 60 / (2*pi*R)
        left_rpm = int(round(left_vel * 60.0 / (2 * math.pi * self.driver.R_Wheel)))
        # Đổi dấu cho bánh phải (cấu hình xoay ngược chiều so với bánh trái)
        right_rpm = int(round(-right_vel * 60.0 / (2 * math.pi * self.driver.R_Wheel)))

        # Gửi lệnh điều khiển RPM tới driver
        self.driver.set_rpm(left_rpm, right_rpm)

def main(args=None):
    try:
        rclpy.init(args=args)

        node = VelocityControllerNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ =='__main__':
    main()
