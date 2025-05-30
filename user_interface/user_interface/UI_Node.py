#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from robot_ui import RobotUI
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from PyQt5.QtWidgets import QApplication

# class UI_Node(Node):
#     def __init__(self): # constructor 
#         super().__init__('UI_Node')
#         print("UI_Node initialize")

#         self.ui = RobotUI()
#         print("RobotUI initialized")

#         self.odom_sub_ = self.create_subscription(Odometry, "liem_controller/odom", self.msgCallback, 10)
#         self.vel_pub_ = self.create_publisher(Twist, '/UI/cmd_vel', 10)

#     def msgCallback(self, msg:Odometry):

#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
        
#         self.ui.xCurrentPos.display(x)
#         self.ui.yCurrentPos.display(y)

#         self.ui.curve_current.setData(self.current_x, self.current_y)
        
# def main(args=None):
#     try:
#         rclpy.init(args=args)
#         app = QApplication([])

#         node = UI_Node()
#         node.ui.show()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     except Exception as e:
#         print(e)

# if __name__ =='__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_ui import RobotUI
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication

class RobotUINode(Node, RobotUI):
    def __init__(self):
        # Khởi tạo ROS 2 Node
        super().__init__('robot_ui_node')
        self.get_logger().info("Robot UI Node Initialized")

        # Khởi tạo giao diện PyQt5
        self.ui = RobotUI(self)

        # ROS 2 Publisher
        self.cmd_pub = self.create_publisher(Twist, '/UI/cmd_vel', 10)

        # ROS 2 Subscriber
        self.odom_sub = self.create_subscription(Odometry, '/liem_controller/odom', self.odom_callback, 10)

        # Kết nối các nút bấm với hành động ROS 2


def main(args=None):
    try:
        rclpy.init(args=args)
        node = RobotUINode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()


