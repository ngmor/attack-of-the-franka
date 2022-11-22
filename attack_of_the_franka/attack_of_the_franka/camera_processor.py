import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs.msg
import numpy as np
import copy
from rcl_interfaces.msg import ParameterDescriptor
import pyrealsense2 as rs2
import geometry_msgs.msg
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer

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

class Pixel():
    x = 0
    y = 0

class ContourData():
    def __init__(self,contour):
        self.contour = contour

        self.moments = cv2.moments(self.contour)
        self.centroid = Pixel()
        self.coord = None

        # valid contour if area is not 0
        self.valid = self.moments['m00'] != 0

        if self.valid:
            self.centroid.x = int(self.moments['m10']/self.moments['m00'])
            self.centroid.y = int(self.moments['m01']/self.moments['m00'])

    def calc_coord(self,depth_image,intrinsics):
        """Calculate coordinates from depth image and intrinsics."""
        if (not self.valid) or not intrinsics:
            self.coord = None
            return

        try:
            depth = depth_image[self.centroid.y, self.centroid.x]
            pixel = [self.centroid.x, self.centroid.y]
            position = rs2.rs2_deproject_pixel_to_point(intrinsics,pixel,depth)

            self.coord = geometry_msgs.msg.Point()
            self.coord.x = position[0]
            self.coord.y = position[1]
            self.coord.z = position[2]

        except ValueError as e:
            self.coord = None
            return


class CameraProcessor(Node):
    """TODO"""

    def __init__(self):
        """Class constructor."""
        super().__init__('camera_processor')

        self.interval = 1.0 / 30.0 #30 fps
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
        self.intrinsics = None

        self.color_image = None
        self.aligned_depth_image = None
        self.color_window_name = 'Color Image'
        self.ally_mask_window_name = 'Ally Mask'
        self.enemy_mask_window_name = 'Enemy Mask'
        cv2.namedWindow(self.color_window_name, cv2.WINDOW_NORMAL)
        if self.enable_ally_sliders:
            cv2.namedWindow(self.ally_mask_window_name, cv2.WINDOW_NORMAL)

        if self.enable_enemy_sliders:
            cv2.namedWindow(self.enemy_mask_window_name, cv2.WINDOW_NORMAL)

        self.filter_kernel = 5
        cv2.createTrackbar('Kernel', self.color_window_name,self.filter_kernel,50,self.trackbar_filter_kernel)

        self.area_threshold = 8000
        cv2.createTrackbar('Area Threshold', self.color_window_name,self.area_threshold,10000,self.trackbar_area_threshold)

        self.contours_filtered_ally = []
        self.contours_filtered_enemy = []

        self.ally_hsv = HSVLimits('Ally',self.ally_mask_window_name,[100,57,43],[142,255,255],self.enable_ally_sliders)
        self.enemy_hsv = HSVLimits('Enemy',self.enemy_mask_window_name,[0,193,90],[9,255,206],self.enable_enemy_sliders)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)

        # TODO put frame names into common reference file
        self.apriltag_base = TransformStamped()
        self.apriltag_base.header.frame_id = "robot_table_reference"
        self.apriltag_base.child_frame_id = "panda_link0"

        # Measurements from table:
        tag_size = 0.173 # m
        table_width = 0.605 # m
        x_edge_to_table_edge = 0.023 # m
        y_edge_to_base = 0.3 # m


        self.apriltag_base.transform.translation.x = -(table_width / 2.0 - x_edge_to_table_edge - tag_size / 2.0)
        self.apriltag_base.transform.translation.y = y_edge_to_base + tag_size / 2.0
        self.apriltag_base.transform.translation.z = 0.0

        time = self.get_clock().now().to_msg()
        self.apriltag_base.header.stamp = time
        self.static_broadcaster.sendTransform(self.apriltag_base)

        self.get_logger().info("camera_processor node started")

    def trackbar_filter_kernel(self,val):
        self.filter_kernel = val
    def trackbar_area_threshold(self,val):
        self.area_threshold = val
    
    def timer_callback(self):
        
        # Can't execute if there isn't a color image yet
        if self.color_image is None:
            return

        hsv_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        color_image_with_tracking = copy.deepcopy(self.color_image)

        kernel = np.ones((self.filter_kernel,self.filter_kernel),np.uint8)

        # Threshold HSV image to get only ally color
        mask_ally = cv2.inRange(hsv_image,self.ally_hsv.lower.to_np_array(),self.ally_hsv.upper.to_np_array())
        
        # Get contours of ally
        ret_ally, thresh_ally = cv2.threshold(mask_ally, 127, 255, 0)
        opening_ally = cv2.morphologyEx(thresh_ally, cv2.MORPH_OPEN, kernel)
        opening_then_closing_ally = cv2.morphologyEx(opening_ally, cv2.MORPH_CLOSE, kernel)
        contours_ally, hierarchy_ally = cv2.findContours(opening_then_closing_ally, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        # Make sure we have contours
        if len(contours_ally) > 0:
            self.contours_filtered_ally = []

            for contour in contours_ally:
                if cv2.contourArea(contour) > self.area_threshold:
                    contour_data = ContourData(contour)
                    self.contours_filtered_ally.append(contour_data)

                    # Only add contour to image if it is valid
                    if contour_data.valid:
                        # Add contours to image
                        color_image_with_tracking = cv2.drawContours(color_image_with_tracking, [contour_data.contour], 0, (255,0,0), 3)

                        # Add centroid to color image
                        color_image_with_tracking = cv2.circle(color_image_with_tracking, (contour_data.centroid.x,contour_data.centroid.y), radius=10, color=(255, 0, 0), thickness=-1)


        # Threshold HSV image to get only enemy color
        mask_enemy = cv2.inRange(hsv_image,self.enemy_hsv.lower.to_np_array(),self.enemy_hsv.upper.to_np_array())

        # Get contours of ally
        ret_enemy, thresh_enemy = cv2.threshold(mask_enemy, 127, 255, 0)
        opening_enemy = cv2.morphologyEx(thresh_enemy, cv2.MORPH_OPEN, kernel)
        opening_then_closing_enemy = cv2.morphologyEx(opening_enemy, cv2.MORPH_CLOSE, kernel)
        contours_enemy, hierarchy_enemy = cv2.findContours(opening_then_closing_enemy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Make sure we have contours
        if len(contours_enemy) > 0:
            self.contours_filtered_enemy = []

            for contour in contours_enemy:
                if cv2.contourArea(contour) > self.area_threshold:
                    contour_data = ContourData(contour)
                    self.contours_filtered_enemy.append(contour_data)

                    # Only add contour to image if it is valid
                    if contour_data.valid:
                        # Add contours to image
                        color_image_with_tracking = cv2.drawContours(color_image_with_tracking, [contour_data.contour], 0, (0,0,255), 3)

                        # Add centroid to color image
                        color_image_with_tracking = cv2.circle(color_image_with_tracking, (contour_data.centroid.x,contour_data.centroid.y), radius=10, color=(0, 0, 255), thickness=-1)

        if self.enable_ally_sliders:
            cv2.imshow(self.ally_mask_window_name,mask_ally)
        if self.enable_enemy_sliders:
            cv2.imshow(self.enemy_mask_window_name, mask_enemy)

        color_image_with_tracking = cv2.putText(color_image_with_tracking, f'Allies: {len(self.contours_filtered_ally)}',(50,50),cv2.FONT_HERSHEY_DUPLEX,1,(255,0,0))
        color_image_with_tracking = cv2.putText(color_image_with_tracking, f'Enemies: {len(self.contours_filtered_enemy)}',(50,100),cv2.FONT_HERSHEY_DUPLEX,1,(0,0,255))

        cv2.imshow(self.color_window_name,color_image_with_tracking)

        # Find real world coordinates of all contours
        if (self.aligned_depth_image is not None) and (self.intrinsics is not None):
            for contour in self.contours_filtered_ally:
                contour.calc_coord(self.aligned_depth_image,self.intrinsics)

            for contour in self.contours_filtered_enemy:
                contour.calc_coord(self.aligned_depth_image,self.intrinsics)

            # TODO - publish transforms for these coordinates to the camera frame
        
        
        cv2.waitKey(1)
    
    def color_image_callback(self, data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
            return

    def aligned_depth_image_callback(self,data):
        try:
            self.aligned_depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
            return

    def color_info_callback(self,info):
        # https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_camera/scripts/show_center_depth.py
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = info.width
            self.intrinsics.height = info.height
            self.intrinsics.ppx = info.k[2]
            self.intrinsics.ppy = info.k[5]
            self.intrinsics.fx = info.k[0]
            self.intrinsics.fy = info.k[4]
            if info.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif info.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in info.d]
        except CvBridgeError as e:
            print(e)
            return


def entry(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    rclpy.spin(camera_processor)
    rclpy.shutdown()