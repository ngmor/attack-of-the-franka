import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from moveit_msgs.srv import GetCartesianPath

## RESOURCES
# service type for cartesian path: http://docs.ros.org/en/api/moveit_msgs/html/srv/GetCartesianPath.html

class Cartesian(Node):
    def __init__(self):
        super().__init__('testing_cartesian')

        self.get_logger().info("In testing_cartesian")
        self.check_client = self.create_timer(1/200, self.timer_callback)
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path',)
        
        self.req = GetCartesianPath.Request()
        self.req.header.stamp = self.get_clock().now()
        self.req.header.frame_id = "panda_link0"
        self.req.start_state.joint_state

        self.req_future = self.cartesian_client.call_async(self.req)

        def timer_callback(self):
            if self.req_future.done():
                self.get_logger().info("future finished")

def main(args=None):
    rclpy.init(args=args)
    testing = Cartesian()
    rclpy.spin(testing)
    rclpy.shutdown()