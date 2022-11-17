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
        self.sub_color_image = self.create_subscription(sensor_msgs.msg.Image,'/camera/color/image_raw',self.color_image_callback,10)
        self.sub_color_camera_info = self.create_subscription(sensor_msgs.msg.CameraInfo,'/camera/color/camera_info',self.color_info_callback,10)
        self.sub_aligned_depth_image = self.create_subscription(sensor_msgs.msg.Image,'/camera/aligned_depth_to_color/image_raw',self.aligned_depth_image_callback,10)

        self.declare_parameter("enable_ally_sliders", False,
                               ParameterDescriptor(description="Enable Ally HSV sliders"))
        self.enable_ally_sliders = self.get_parameter("enable_ally_sliders").get_parameter_value().bool_value
        self.declare_parameter("enable_enemy_sliders", False,
                               ParameterDescriptor(description="Enable Enemy HSV sliders"))
        self.enable_enemy_sliders = self.get_parameter("enable_enemy_sliders").get_parameter_value().bool_value

        self.bridge = CvBridge()

        self.color_window_name = 'Color Image'
        self.ally_mask_window_name = 'Ally Mask'
        self.enemy_mask_window_name = 'Enemy Mask'
        cv2.namedWindow(self.color_window_name, cv2.WINDOW_NORMAL)
        if self.enable_ally_sliders:
            cv2.namedWindow(self.ally_mask_window_name, cv2.WINDOW_NORMAL)

        if self.enable_enemy_sliders:
            cv2.namedWindow(self.enemy_mask_window_name, cv2.WINDOW_NORMAL)

        self.filter_kernel = 5
        cv2.namedWindow(self.color_window_name, cv2.WINDOW_NORMAL)
        cv2.createTrackbar('Kernel', self.color_window_name,self.filter_kernel,50,self.trackbar_filter_kernel)

        self.ally_hsv = HSVLimits('Ally',self.ally_mask_window_name,[100,57,43],[142,255,255],self.enable_ally_sliders)
        self.enemy_hsv = HSVLimits('Enemy',self.enemy_mask_window_name,[0,193,90],[9,255,206],self.enable_enemy_sliders)

        self.get_logger().info("camera node started")

    def trackbar_filter_kernel(self,val):
        self.filter_kernel = val
    
    def timer_callback(self):
        pass
    
    def color_image_callback(self, data):

        color_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        color_image_with_tracking = copy.deepcopy(color_image)

        kernel = np.ones((self.filter_kernel,self.filter_kernel),np.uint8)

        # Threshold HSV image to get only ally color
        mask_ally = cv2.inRange(hsv_image,self.ally_hsv.lower.to_np_array(),self.ally_hsv.upper.to_np_array())
        
        # Get contours of ally
        ret_ally, thresh_ally = cv2.threshold(mask_ally, 127, 255, 0)
        opening_ally = cv2.morphologyEx(thresh_ally, cv2.MORPH_OPEN, kernel)
        opening_then_closing_ally = cv2.morphologyEx(opening_ally, cv2.MORPH_CLOSE, kernel)
        contours_ally, hierarchy_ally = cv2.findContours(opening_then_closing_ally, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        valid_contour_ally = True

        # Make sure we have contours
        if len(contours_ally) <= 0:
            valid_contour_ally = False

        else:
            # Get largest contours
            largest_contour_ally = max(contours_ally, key = cv2.contourArea)

            # calculate moments
            moments_ally = cv2.moments(largest_contour_ally)

            # Not valid if this moment is 0
            if moments_ally['m00'] == 0:
                valid_contour_ally = False
            else:  
                # Calculate centroids
                centroid_x_ally = int(moments_ally['m10']/moments_ally['m00'])
                centroid_y_ally = int(moments_ally['m01']/moments_ally['m00'])

                # Add contours to image
                color_image_with_tracking = cv2.drawContours(color_image_with_tracking, contours_ally, -1, (255,0,0), 3)

                # Add centroid to color image
                color_image_with_tracking = cv2.circle(color_image_with_tracking, (centroid_x_ally,centroid_y_ally), radius=10, color=(255, 0, 0), thickness=-1)

        # Threshold HSV image to get only enemy color
        mask_enemy = cv2.inRange(hsv_image,self.enemy_hsv.lower.to_np_array(),self.enemy_hsv.upper.to_np_array())

        # Get contours of ally
        ret_enemy, thresh_enemy = cv2.threshold(mask_enemy, 127, 255, 0)
        opening_enemy = cv2.morphologyEx(thresh_enemy, cv2.MORPH_OPEN, kernel)
        opening_then_closing_enemy = cv2.morphologyEx(opening_enemy, cv2.MORPH_CLOSE, kernel)
        contours_enemy, hierarchy_enemy = cv2.findContours(opening_then_closing_enemy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        valid_contour_enemy = True

        # Make sure we have contours
        if len(contours_enemy) <= 0:
            valid_contour_enemy = False

        else:
            # Get largest contours
            largest_contour_enemy = max(contours_enemy, key = cv2.contourArea)

            # calculate moments
            moments_enemy = cv2.moments(largest_contour_enemy)

            # Not valid if this moment is 0
            if moments_enemy['m00'] == 0:
                valid_contour_enemy = False
            else:  
                # Calculate centroids
                centroid_x_enemy = int(moments_enemy['m10']/moments_enemy['m00'])
                centroid_y_enemy = int(moments_enemy['m01']/moments_enemy['m00'])

                # Add contours to image
                color_image_with_tracking = cv2.drawContours(color_image_with_tracking, contours_enemy, -1, (0,0,255), 3)

                # Add centroid to color image
                color_image_with_tracking = cv2.circle(color_image_with_tracking, (centroid_x_enemy,centroid_y_enemy), radius=10, color=(0, 0, 255), thickness=-1)

        if self.enable_ally_sliders:
            cv2.imshow(self.ally_mask_window_name,mask_ally)
        if self.enable_enemy_sliders:
            cv2.imshow(self.enemy_mask_window_name, mask_enemy)
        cv2.imshow(self.color_window_name,color_image_with_tracking)

        cv2.waitKey(1)

    def aligned_depth_image_callback(self,data):
        aligned_depth_image = self.bridge.imgmsg_to_cv2(data)

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(aligned_depth_image, alpha=0.3), cv2.COLORMAP_JET)
        cv2.imshow('Depth Colormap',depth_colormap)
        # self.get_logger().info(f'{aligned_depth_image[0][0]}')

    def color_info_callback(self,info):
        pass
        # self.get_logger().info(f'Color: {info}')
        # TODO - get intrinsics


def entry(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    rclpy.spin(camera_processor)
    rclpy.shutdown()