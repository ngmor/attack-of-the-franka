import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import sensor_msgs.msg
import numpy as np
import copy
from rcl_interfaces.msg import ParameterDescriptor

class HSV():
    def __init__(self,H,S,V):
        self.H = H
        self.S = S
        self.V = V

    def to_array(self):
        return [self.H,self.S,self.V]

    def to_np_array(self):
        return np.array(self.to_array())

class HSVLimits():
    
    def __init__(self,name,window_name,lower_bounds,upper_bounds,use_trackbar=False):
        self.lower = HSV(lower_bounds[0],lower_bounds[1],lower_bounds[2])
        self.upper = HSV(upper_bounds[0],upper_bounds[1],upper_bounds[2])
        self.name = name
        self.window_name = window_name
        self.lower_names = HSV(self.name + ' H lo', self.name + ' S lo', self.name + ' V lo')
        self.upper_names = HSV(self.name + ' H hi', self.name + ' S hi', self.name + ' V hi')

        if use_trackbar:
            cv2.createTrackbar(self.lower_names.H, self.window_name,self.lower.H,180,self.trackbar_lower_H)
            cv2.createTrackbar(self.upper_names.H, self.window_name,self.upper.H,180,self.trackbar_upper_H)
            cv2.createTrackbar(self.lower_names.S, self.window_name,self.lower.S,255,self.trackbar_lower_S)
            cv2.createTrackbar(self.upper_names.S, self.window_name,self.upper.S,255,self.trackbar_upper_S)
            cv2.createTrackbar(self.lower_names.V, self.window_name,self.lower.V,255,self.trackbar_lower_V)
            cv2.createTrackbar(self.upper_names.V, self.window_name,self.upper.V,255,self.trackbar_upper_V)

    def trackbar_lower_H(self,val):
        self.lower.H = val
        self.lower.H = min(self.upper.H-1, self.lower.H)
        cv2.setTrackbarPos(self.lower_names.H,self.window_name,self.lower.H)

    def trackbar_upper_H(self,val):
        self.upper.H = val
        self.upper.H = max(self.lower.H+1, self.upper.H)
        cv2.setTrackbarPos(self.upper_names.H,self.window_name,self.upper.H)

    def trackbar_lower_S(self,val):
        self.lower.S = val
        self.lower.S = min(self.upper.S-1, self.lower.S)
        cv2.setTrackbarPos(self.lower_names.S,self.window_name,self.lower.S)

    def trackbar_upper_S(self,val):
        self.upper.S = val
        self.upper.S = max(self.lower.S+1, self.upper.S)
        cv2.setTrackbarPos(self.upper_names.S,self.window_name,self.upper.S)

    def trackbar_lower_V(self,val):
        self.lower.V = val
        self.lower.V = min(self.upper.V-1, self.lower.V)
        cv2.setTrackbarPos(self.lower_names.V,self.window_name,self.lower.V)

    def trackbar_upper_V(self,val):
        self.upper.V = val
        self.upper.V = max(self.lower.V+1, self.upper.V)
        cv2.setTrackbarPos(self.upper_names.V,self.window_name,self.upper.V)

class CameraProcessor(Node):
    """TODO"""

    def __init__(self):
        """Class constructor."""
        super().__init__('camera_processor')

        self.interval = 1.0 / 100.0
        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.sub_image = self.create_subscription(sensor_msgs.msg.Image,'/camera/color/image_raw',self.image_callback,10)

        self.declare_parameter("enable_ally_sliders", False,
                               ParameterDescriptor(description="Enable Ally HSV sliders"))
        self.enable_ally_sliders = self.get_parameter("enable_ally_sliders").get_parameter_value().bool_value
        self.declare_parameter("enable_enemy_sliders", False,
                               ParameterDescriptor(description="Enable Enemy HSV sliders"))
        self.enable_enemy_sliders = self.get_parameter("enable_enemy_sliders").get_parameter_value().bool_value

        self.bridge = CvBridge()

        self.window_name = 'Color Image'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.ally_hsv = HSVLimits('Ally',self.window_name,[119,74,0],[164,255,255],self.enable_ally_sliders)
        self.enemy_hsv = HSVLimits('Enemy',self.window_name,[119,74,0],[164,255,255],self.enable_enemy_sliders)

        self.get_logger().info("camera node started")
    
    def timer_callback(self):
        pass

    def image_callback(self, msg):

        color_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        color_image_with_tracking = copy.deepcopy(color_image)
        # Threshold HSV image to get only purple
        mask = cv2.inRange(hsv_image,self.ally_hsv.lower.to_np_array(),self.ally_hsv.upper.to_np_array())

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