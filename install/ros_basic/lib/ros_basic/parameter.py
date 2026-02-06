#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class SimpleParameter(Node):
    
    def __init__(self):
        super().__init__("simple_parameter")
        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "Antonio")

        self.simple_int_param = self.get_parameter("simple_int_param").value
        self.simple_string_param = self.get_parameter("simple_string_param").value
        print("Simple int param: %d" % self.simple_int_param)
        print("Simple string param: %s" % self.simple_string_param)


        self.timer_ = self.create_timer(1, self.timerCallback)
        self.int = 0

        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def timerCallback(self):
      
        self.get_logger().info("Simple int self.int : %d" % self.int )

    def paramChangeCallback(self, params):
        result = SetParametersResult()
        print("Param change callback called", params)

        for param in params:

            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info("Param simple_int_param changed! New value is %d" % param.value)
                self.int = param.value
                result.successful = True

            if param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("Param simple_string_param changed! New value is %s" % param.value)
                result.successful = True 

        return result


def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()