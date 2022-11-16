import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import sensor_msgs.msg
import numpy as np
import copy

class CameraProcessor(Node):
    """TODO"""

    def __init__(self):
        """Class constructor."""
        super().__init__('camera_processor')

        self.interval = 1.0 / 100.0
        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.sub_image = self.create_subscription(sensor_msgs.msg.Image,'/camera/color/image_raw',self.image_callback,10)
        
        self.bridge = CvBridge()

        # Define HSV color limits
        self.lower_H = 119#117
        self.lower_S = 74#110
        self.lower_V = 0#0

        # B = 69, G = 27, R = 26
        # H = 119, S = 159, V = 69
        self.upper_H = 164#139
        self.upper_S = 255
        self.upper_V = 255

        self.lower_H_name = 'H Low'

        self.window_name = 'Pen Tracker'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.createTrackbar(self.lower_H_name, self.window_name,self.lower_H,180,self.trackbar_lower_H)

        self.get_logger().info("camera node started")
    
    def timer_callback(self):
        pass
    
        
    def trackbar_lower_H(self,val):
        self.lower_H = val
        self.lower_H = min(self.upper_H-1, self.lower_H)
        cv2.setTrackbarPos(self.lower_H_name,self.window_name,self.lower_H)
    

    def image_callback(self, msg):
        

        # HSV values
        pen_lower = np.array([self.lower_H,self.lower_S,self.lower_V])
        pen_upper = np.array([self.upper_H,self.upper_S,self.upper_V])

        color_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        color_image_with_tracking = copy.deepcopy(color_image)
        # Threshold HSV image to get only purple
        mask = cv2.inRange(hsv_image,pen_lower,pen_upper)

        # Get contours
        ret, thresh = cv2.threshold(mask, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        validContour = True

        # Make sure we have contours
        if len(contours) <= 0:
            validContour = False

        else:
            # Get largest contours
            largest_contour = max(contours, key = cv2.contourArea)

            # calculate moments
            moments = cv2.moments(largest_contour)

            # Not valid if this moment is 0
            if moments['m00'] == 0:
                validContour = False
            else:  
                # Calculate centroids
                centroid_x = int(moments['m10']/moments['m00'])
                centroid_y = int(moments['m01']/moments['m00'])

                # Add contours to image
                color_image_with_tracking = cv2.drawContours(color_image_with_tracking, [largest_contour], 0, (0,255,0), 3)

                # Add centroid to color image
                color_image_with_tracking = cv2.circle(color_image_with_tracking, (centroid_x,centroid_y), radius=10, color=(0, 0, 255), thickness=-1)

        cv2.imshow(self.window_name,color_image_with_tracking)

        cv2.waitKey(1)

    

def entry(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    rclpy.spin(camera_processor)
    rclpy.shutdown()