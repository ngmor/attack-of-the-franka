import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient


class Cartesian(Node):
    def __init__(self):
        super().__init__('testing_cartesian')

        self.get_logger().info("In testing_cartesian")
        

def main(args=None):
    rclpy.init(args=args)
    testing = Cartesian()
    rclpy.spin(testing)
    rclpy.shutdown()