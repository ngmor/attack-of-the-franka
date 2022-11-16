import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import sensor_msgs.msg

class CameraProcessor(Node):
    """TODO"""

    def __init__(self):
        """Class constructor."""
        super().__init__('camera_processor')

        self.interval = 1.0 / 100.0
        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.sub_image = self.create_subscription(sensor_msgs.msg.Image,'/camera/color/image_raw',self.image_callback,10)
        
        self.bridge = CvBridge()

        self.get_logger().info("camera node started")

    def image_callback(self, msg):

        current_frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        cv2.imshow("camera",current_frame)

        cv2.waitKey(1)

    def timer_callback(self):
        pass

def entry(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    rclpy.spin(camera_processor)
    rclpy.shutdown()