# Copyright 2022 Attack of the Franka.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Performs image processing for ally and enemy detection based on color.

Gets workspace area transforms and the robot transform from AprilTags.

Publisher:
    object_detections (Detections) : publishes the detected objects

Subscribers:
    /camera/color/image_raw (sensor_msgs.msg.Image) : gets the color image_raw data
    /camera/color/camera_info (sensor_msgs.msg.CameraInfo) : gets meta information for the
        camera
    /camera/aligned_depth_to_color/image_raw (sensor_msgs.msg.Image) : gets the
        aligned_depth_to_color raw image
    enemy_dead_count (std_msgs.msg.Int16) : gets the dead enemy count

Services:
    start_apriltag_calibration (std_srvs.srv.Empty) :   starts the AprilTag calibrations
    update_calibration_continuously (std_srvs.srv.Empty) :  updates the AprilTag calibrations

Authors:
    Sushma Chandra
    Vaishnavi Dornadula
    Nick Morales
    Meg Sindelar

Last Update: December 8th, 2022
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs.msg
import std_srvs.srv
import numpy as np
import copy
import std_msgs.msg
from rcl_interfaces.msg import ParameterDescriptor
import pyrealsense2 as rs2
import geometry_msgs.msg
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from attack_of_the_franka.common import FRAMES, angle_axis_to_quaternion,\
    ObjectType, get_average_transformation
from attack_of_the_franka_interfaces.msg import DetectedObject, Detections

camera_scale_factor = 1.0 / 1000.0


class Pixel():
    """Camera image pixel information."""

    x = 0
    y = 0

    def get_from_camera_coordinates(self, intrinsics, point):
        """
        Convert 3D real world coordinates into pixel coordinates from the input point.

        Args:
        ----
            intrinsics (rs2.intrinsics): camera intrinsic information
            point (Vector3 or Point): point in 3D real world coordinates of the camera frame.

        """
        if intrinsics is None:
            return

        # Project point to pixel, changing frames from the camera frame
        pixel = rs2.rs2_project_point_to_pixel(intrinsics, [-point.y, -point.z, point.x])
        self.x = int(pixel[0])
        self.y = int(pixel[1])


class Limits():
    """General use lower and upper limits structure."""

    lower = 0.
    upper = 0.


class HSV():
    """HSV structure."""

    def __init__(self, H, S, V):
        self.H = H
        self.S = S
        self.V = V

    def to_array(self):
        """Return HSV data as an array."""
        return [self.H, self.S, self.V]

    def to_np_array(self):
        """Return HSV data as a np array."""
        return np.array(self.to_array())


class TrackbarLimits():
    """General use class for creating trackbars to adjust two sided limit values through OpenCV."""

    def __init__(self, name, window_name, initial_values, limits, use_trackbar=False):
        """Initialize trackbars and limits for the values."""
        self.name = Limits()
        self.name.lower = name + ' lo'
        self.name.upper = name + ' hi'
        self.window_name = window_name
        self.value = Limits()
        self.value.lower = initial_values[0]
        self.value.upper = initial_values[1]
        self.limits = Limits()
        self.limits.lower = limits[0]
        self.limits.upper = limits[1]

        if use_trackbar:
            cv2.createTrackbar(self.name.lower, self.window_name, self.value.lower,
                               self.limits.upper, self.trackbar_lower)
            cv2.createTrackbar(self.name.upper, self.window_name, self.value.upper,
                               self.limits.upper, self.trackbar_upper)

    def trackbar_lower(self, val):
        """Adjust lower limit when trackbar is moved."""
        self.value.lower = val
        self.value.lower = min(self.value.upper-1, self.value.lower)
        self.value.lower = max(self.value.lower, self.limits.lower)
        cv2.setTrackbarPos(self.name.lower, self.window_name, self.value.lower)

    def trackbar_upper(self, val):
        """Adjust upper limit when trackbar is moved."""
        self.value.upper = val
        self.value.upper = max(self.value.lower+1, self.value.upper)
        self.value.upper = min(self.value.upper, self.limits.upper)
        cv2.setTrackbarPos(self.name.upper, self.window_name, self.value.upper)


class HSVLimits():
    """Class for creating trackbars to adjust HSV bound limits through OpenCV."""

    # TODO Refactor to include several trackbar limits classes
    def __init__(self, name, window_name, lower_bounds, upper_bounds, use_trackbar=False):
        """Create trackbars to help set HSV lower and upper bounds."""
        self.lower = HSV(lower_bounds[0], lower_bounds[1], lower_bounds[2])
        self.upper = HSV(upper_bounds[0], upper_bounds[1], upper_bounds[2])
        self.name = name
        self.window_name = window_name
        self.lower_names = HSV(self.name + ' H lo', self.name + ' S lo', self.name + ' V lo')
        self.upper_names = HSV(self.name + ' H hi', self.name + ' S hi', self.name + ' V hi')

        if use_trackbar:
            cv2.createTrackbar(self.lower_names.H, self.window_name, self.lower.H, 180,
                               self.trackbar_lower_H)
            cv2.createTrackbar(self.upper_names.H, self.window_name, self.upper.H, 180,
                               self.trackbar_upper_H)
            cv2.createTrackbar(self.lower_names.S, self.window_name, self.lower.S, 255,
                               self.trackbar_lower_S)
            cv2.createTrackbar(self.upper_names.S, self.window_name, self.upper.S, 255,
                               self.trackbar_upper_S)
            cv2.createTrackbar(self.lower_names.V, self.window_name, self.lower.V, 255,
                               self.trackbar_lower_V)
            cv2.createTrackbar(self.upper_names.V, self.window_name, self.upper.V, 255,
                               self.trackbar_upper_V)

    def trackbar_lower_H(self, val):
        """Set the H lower bound when the trackbar is moved."""
        self.lower.H = val
        self.lower.H = min(self.upper.H-1, self.lower.H)
        cv2.setTrackbarPos(self.lower_names.H, self.window_name, self.lower.H)

    def trackbar_upper_H(self, val):
        """Set the H upper bound when the trackbar is moved."""
        self.upper.H = val
        self.upper.H = max(self.lower.H+1, self.upper.H)
        cv2.setTrackbarPos(self.upper_names.H, self.window_name, self.upper.H)

    def trackbar_lower_S(self, val):
        """Set the S lower bound when the trackbar is moved."""
        self.lower.S = val
        self.lower.S = min(self.upper.S-1, self.lower.S)
        cv2.setTrackbarPos(self.lower_names.S, self.window_name, self.lower.S)

    def trackbar_upper_S(self, val):
        """Set the S upper bound when the trackbar is moved."""
        self.upper.S = val
        self.upper.S = max(self.lower.S+1, self.upper.S)
        cv2.setTrackbarPos(self.upper_names.S, self.window_name, self.upper.S)

    def trackbar_lower_V(self, val):
        """Set the V lower bound when the trackbar is moved."""
        self.lower.V = val
        self.lower.V = min(self.upper.V-1, self.lower.V)
        cv2.setTrackbarPos(self.lower_names.V, self.window_name, self.lower.V)

    def trackbar_upper_V(self, val):
        """Set V upper bound when the trackbar is moved."""
        self.upper.V = val
        self.upper.V = max(self.lower.V+1, self.upper.V)
        cv2.setTrackbarPos(self.upper_names.V, self.window_name, self.upper.V)


class ContourData():
    """Information and methods to handle contours found in a masked image."""

    def __init__(self, contour, object_type):
        """Initialize the data by finding area and centroid data of the contour, if valid."""
        self.contour = contour
        self.object_type = object_type

        self.moments = cv2.moments(self.contour)
        self.centroid = Pixel()
        self.coord = None

        # valid contour if area is not 0
        self.valid = self.moments['m00'] != 0

        if self.valid:
            self.centroid.x = int(self.moments['m10']/self.moments['m00'])
            self.centroid.y = int(self.moments['m01']/self.moments['m00'])

    def calc_coord(self, depth_image, intrinsics):
        """Calculate camera frame coordinates from depth image and intrinsics."""
        if (not self.valid) or (depth_image is None) or (intrinsics is None):
            self.coord = None
            return

        try:
            depth = depth_image[self.centroid.y, self.centroid.x]
            pixel = [self.centroid.x, self.centroid.y]
            position = rs2.rs2_deproject_pixel_to_point(intrinsics, pixel, depth)

            self.coord = geometry_msgs.msg.Point()
            self.coord.x = position[2] * camera_scale_factor
            self.coord.y = -position[0] * camera_scale_factor
            self.coord.z = -position[1] * camera_scale_factor

        except ValueError:
            self.coord = None
            return

    def centroid_within_bounds(self, x_limits, y_limits):
        """Return if the centroid is within the input pixel limits."""
        return ((self.centroid.x >= x_limits.lower) and
                (self.centroid.x <= x_limits.upper) and
                (self.centroid.y >= y_limits.lower) and
                (self.centroid.y <= y_limits.upper))

    def coord_within_bounds(self, y_limits, z_limits):
        """Return if the camera coordinates are within the input limits."""
        if self.coord is None:
            return False
        else:
            return ((self.coord.y >= y_limits.lower) and
                    (self.coord.y <= y_limits.upper) and
                    (self.coord.z >= z_limits.lower) and
                    (self.coord.z <= z_limits.upper))

    def depth_within_bounds(self, limits):
        """Return if the camera coordinate depth is within the input limits."""
        if self.coord is None:
            return False
        else:
            if self.coord.x == 0:  # invalid depth
                return False
            else:
                return ((self.coord.x >= limits.lower) and
                        (self.coord.x <= limits.upper))

    def get_frame_name(self, number):
        """Get the frame name of the object by inputting which number it is."""
        if self.object_type == ObjectType.ALLY:
            name = FRAMES().ALLY
        elif self.object_type == ObjectType.ENEMY:
            name = FRAMES().ENEMY
        else:
            name = ''

        name += f'{number:02d}'

        return name

    def broadcast(self, broadcaster, number, time):
        """Broadcast the transform to the frame at the location of the centroid of the contour."""
        if self.coord is None:
            return

        transform = TransformStamped()
        transform.header.frame_id = FRAMES().CAMERA_COLOR
        transform.child_frame_id = self.get_frame_name(number)
        transform.transform.translation.x = self.coord.x
        transform.transform.translation.y = self.coord.y
        transform.transform.translation.z = self.coord.z
        transform.transform.rotation = angle_axis_to_quaternion(-np.pi/2., [0., 1., 0.])
        transform.header.stamp = time

        broadcaster.sendTransform(transform)

    def get_as_detection_object(self, number):
        """Return information about the contour as a DetectedObject type."""
        return DetectedObject(
            name=self.get_frame_name(number)
            # TODO add more information?
        )


class CameraProcessor(Node):
    """
    Performs image processing for ally and enemy detection based on color.

    Gets workspace area transforms and the robot transform from AprilTags.

    Publisher:
        object_detections (Detections) : publishes the detected objects

    Subscribers:
        /camera/color/image_raw (sensor_msgs.msg.Image) : gets the color image_raw data
        /camera/color/camera_info (sensor_msgs.msg.CameraInfo) : gets meta information for the
            camera
        /camera/aligned_depth_to_color/image_raw (sensor_msgs.msg.Image) : gets the
            aligned_depth_to_color raw image
        enemy_dead_count (std_msgs.msg.Int16) : gets the dead enemy count

    Services:
        start_apriltag_calibration (std_srvs.srv.Empty) :   starts the AprilTag calibrations
        update_calibration_continuously (std_srvs.srv.Empty) :  updates the AprilTag calibrations

    """

    def __init__(self):
        """Class constructor."""
        super().__init__('camera_processor')

        self.interval = 1.0 / 30.0  # 30 fps
        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.pub_object_detections = self.create_publisher(Detections, 'object_detections', 10)
        self.sub_color_image = self.create_subscription(
            sensor_msgs.msg.Image,
            '/camera/color/image_raw',
            self.color_image_callback,
            10
        )
        self.sub_color_camera_info = self.create_subscription(
            sensor_msgs.msg.CameraInfo,
            '/camera/color/camera_info',
            self.color_info_callback,
            10
        )
        self.sub_aligned_depth_image = self.create_subscription(
            sensor_msgs.msg.Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.aligned_depth_image_callback,
            10
        )
        self.sub_enemy_dead_count = self.create_subscription(
            std_msgs.msg.Int16,
            'enemy_dead_count',
            self.enemy_deadcount_callback,
            10
        )
        self.srv_start_apriltag_calibration = self.create_service(
            std_srvs.srv.Empty,
            'start_apriltag_calibration',
            self.start_apriltag_calibration_callback
        )
        self.srv_update_calibration_continuously = self.create_service(
            std_srvs.srv.Empty,
            'update_calibration_continuously',
            self.update_calibration_continuously_callback
        )

        # Parameters
        self.declare_parameter(
            "enable_ally_sliders", False,
            ParameterDescriptor(description="Enable Ally HSV sliders")
        )
        self.enable_ally_sliders = self.get_parameter(
            "enable_ally_sliders").get_parameter_value().bool_value
        self.declare_parameter(
            "enable_enemy_sliders", False,
            ParameterDescriptor(description="Enable Enemy HSV sliders")
        )
        self.enable_enemy_sliders = self.get_parameter(
            "enable_enemy_sliders").get_parameter_value().bool_value
        self.declare_parameter(
            "invert_ally_hue", False,
            ParameterDescriptor(description="Inverts ally hue range")
        )
        self.invert_ally_hue = self.get_parameter(
            "invert_ally_hue").get_parameter_value().bool_value
        self.declare_parameter(
            "invert_enemy_hue", True,
            ParameterDescriptor(description="Inverts enemy hue range")
        )
        self.invert_enemy_hue = self.get_parameter(
            "invert_enemy_hue").get_parameter_value().bool_value
        self.declare_parameter(
            "enable_filtering_sliders", False,
            ParameterDescriptor(description="Enable filtering sliders")
        )
        self.enable_filtering_sliders = self.get_parameter(
            "enable_filtering_sliders").get_parameter_value().bool_value
        self.declare_parameter(
            "enable_work_area_sliders", False,
            ParameterDescriptor(description="Enable work area bounds by sliders")
        )
        self.enable_work_area_sliders = self.get_parameter(
            "enable_work_area_sliders").get_parameter_value().bool_value
        self.declare_parameter(
            "enable_depth_filter", True,
            ParameterDescriptor(description="Activate depth filter")
        )
        self.enable_depth_filter = self.get_parameter(
            "enable_depth_filter").get_parameter_value().bool_value
        self.declare_parameter(
            "enable_depth_filter_sliders", False,
            ParameterDescriptor(description="Enable depth filter sliders")
        )
        self.enable_depth_filter_sliders = self.get_parameter(
            "enable_depth_filter_sliders").get_parameter_value().bool_value
        self.declare_parameter(
            "enable_work_area_apriltags", True,
            ParameterDescriptor(description="Enable work area bounds by AprilTags")
        )
        self.enable_work_area_apriltags = self.get_parameter(
            "enable_work_area_apriltags").get_parameter_value().bool_value
        self.declare_parameter(
            "broadcast_transforms_directly", True,
            ParameterDescriptor(
                description="Enable broadcasting of transforms for enemies/allies from this node.")
        )
        self.broadcast_transforms_directly = self.get_parameter(
            "broadcast_transforms_directly").get_parameter_value().bool_value
        self.declare_parameter(
            "sort_by_x", False,
            ParameterDescriptor(description="sorts the allies and enemies in x")
        )
        self.sort_by_x = self.get_parameter("sort_by_x").get_parameter_value().bool_value
        self.declare_parameter(
            "update_calibration_continuously", False,
            ParameterDescriptor(description="Update AprilTag calibration continuously.")
        )
        self.update_calibration_continuously = self.get_parameter(
            "update_calibration_continuously").get_parameter_value().bool_value

        # Dimension parameters
        self.declare_parameter(
            "apriltags.robot_table_tag_size", 0.173,
            ParameterDescriptor(description="Size of AprilTag on robot table")
        )
        self.robot_table_tag_size = self.get_parameter(
            "apriltags.robot_table_tag_size").get_parameter_value().double_value
        self.declare_parameter(
            "apriltags.work_area_tag_size", 0.055,
            ParameterDescriptor(description="Size of AprilTags at work area bounds")
        )
        self.work_area_tag_size = self.get_parameter(
            "apriltags.work_area_tag_size").get_parameter_value().double_value
        self.declare_parameter(
            "robot_table.width", 0.605,
            ParameterDescriptor(description="Robot table width")
        )
        self.robot_table_width = self.get_parameter(
            "robot_table.width").get_parameter_value().double_value
        self.declare_parameter(
            "robot_table.y_tag_edge_to_base", 0.3,
            ParameterDescriptor(description="Y edge of AprilTag to base of robot")
        )
        self.robot_table_y_tag_edge_to_base = self.get_parameter(
            "robot_table.y_tag_edge_to_base").get_parameter_value().double_value
        self.declare_parameter(
            "robot_table.x_tag_edge_to_table_edge", 0.023,
            ParameterDescriptor(description="X edge of AprilTag to edge of robot table")
        )
        self.robot_table_x_tag_edge_to_table_edge = self.get_parameter(
            "robot_table.x_tag_edge_to_table_edge").get_parameter_value().double_value

        # sliders get precedence over AprilTags
        if self.enable_work_area_sliders and self.enable_work_area_apriltags:
            self.enable_work_area_apriltags = False

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

        self.ally_hsv = HSVLimits('Ally', self.ally_mask_window_name,
                                  [100, 57, 120], [142, 255, 255], self.enable_ally_sliders)
        self.enemy_hsv = HSVLimits('Enemy', self.enemy_mask_window_name,
                                   [20, 81, 167], [160, 255, 255], self.enable_enemy_sliders)

        self.filter_kernel = 5
        self.area_threshold = 1250

        if self.enable_filtering_sliders:
            cv2.createTrackbar('Kernel', self.color_window_name, self.filter_kernel, 50,
                               self.trackbar_filter_kernel)
            cv2.createTrackbar('Area Threshold', self.color_window_name, self.area_threshold,
                               10000, self.trackbar_area_threshold)

        self.x_limits = TrackbarLimits('X', self.color_window_name,
                                       [558, 808], [0, 1280], self.enable_work_area_sliders)
        self.y_limits = TrackbarLimits('Y', self.color_window_name,
                                       [261, 642], [0, 720], self.enable_work_area_sliders)

        self.depth_limits = TrackbarLimits('Depth', self.color_window_name,
                                           [0, 1670], [0, 3000], self.enable_depth_filter_sliders)

        self.contours_filtered_ally = []
        self.contours_filtered_enemy = []

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_camera_to_robot_table_raw = None
        self.tf_camera_to_workspace1_raw = None
        self.tf_camera_to_workspace2_raw = None
        self.tf_workspace_min = None
        self.tf_workspace_max = None
        self.workspace_min_pixel = Pixel()
        self.workspace_max_pixel = Pixel()
        self.work_area_limits_x = Limits()  # color camera frame coordinates, from AprilTags(depth)
        self.work_area_limits_y = Limits()  # color camera frame coordinates, from AprilTags
        self.work_area_limits_z = Limits()  # color camera frame coordinates, from AprilTags

        # Publish static transform between AprilTag on robot table and robot base
        tf_table_apriltag_to_base = TransformStamped()
        tf_table_apriltag_to_base.header.frame_id = FRAMES().PANDA_TABLE
        tf_table_apriltag_to_base.child_frame_id = FRAMES().PANDA_BASE

        tf_table_apriltag_to_base.transform.translation.x = \
            -(self.robot_table_width / 2.0 - self.robot_table_x_tag_edge_to_table_edge -
              self.robot_table_tag_size / 2.0)
        tf_table_apriltag_to_base.transform.translation.y = \
            self.robot_table_y_tag_edge_to_base + self.robot_table_tag_size / 2.0
        tf_table_apriltag_to_base.transform.translation.z = 0.0
        tf_table_apriltag_to_base.transform.rotation = \
            angle_axis_to_quaternion(-np.pi/2, [0., 0., 1.])

        time = self.get_clock().now().to_msg()
        tf_table_apriltag_to_base.header.stamp = time
        self.static_broadcaster.sendTransform(tf_table_apriltag_to_base)

        self.calibration_data_points = 100  # TODO make parameter?

        self.calibrating_robot_table = not self.update_calibration_continuously
        self.robot_table_calibration_array = []
        self.tf_camera_to_robot_table_calibrated = None

        self.calibrating_workspace1 = not self.update_calibration_continuously
        self.workspace1_calibration_array = []
        self.tf_camera_to_workspace1_calibrated = None

        self.calibrating_workspace2 = not self.update_calibration_continuously
        self.workspace2_calibration_array = []
        self.tf_camera_to_workspace2_calibrated = None

        self.dead_enemies_count = 0

        self.get_logger().info("camera_processor node started")

    def timer_callback(self):
        """Execute cyclic camera processing."""
        self.get_transforms()

        # Can't execute if there isn't a color image yet
        if self.color_image is None:
            return

        hsv_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        color_image_with_tracking = copy.deepcopy(self.color_image)

        kernel = np.ones((self.filter_kernel, self.filter_kernel), np.uint8)

        # Threshold HSV image to get only ally color
        if not self.invert_ally_hue:
            mask_ally = cv2.inRange(
                hsv_image,
                self.ally_hsv.lower.to_np_array(),
                self.ally_hsv.upper.to_np_array()
            )
        else:
            # Split into 2 inRange calls and logically and them
            temp_lower = self.ally_hsv.lower.to_np_array()
            temp_upper = self.ally_hsv.upper.to_np_array()

            temp_lower[0] = 0
            temp_upper[0] = self.ally_hsv.lower.H

            mask_ally = cv2.inRange(hsv_image, temp_lower, temp_upper)

            temp_lower[0] = self.ally_hsv.upper.H
            temp_upper[0] = 255

            mask_ally += cv2.inRange(hsv_image, temp_lower, temp_upper)

        # Get contours of ally
        ret_ally, thresh_ally = cv2.threshold(mask_ally, 127, 255, 0)
        opening_ally = cv2.morphologyEx(thresh_ally, cv2.MORPH_OPEN, kernel)
        opening_then_closing_ally = cv2.morphologyEx(opening_ally, cv2.MORPH_CLOSE, kernel)
        contours_ally, hierarchy_ally = cv2.findContours(
            opening_then_closing_ally,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        self.contours_filtered_ally = []

        # Make sure we have contours
        if len(contours_ally) > 0:

            # Iterate through contours to process and filter them
            for contour in contours_ally:
                if cv2.contourArea(contour) > self.area_threshold:
                    contour_data = ContourData(contour, ObjectType.ALLY)

                    # Only add contour to list if it is valid
                    if contour_data.valid:
                        include_contour = True

                        # Find real world coordinates of all contours
                        contour_data.calc_coord(self.aligned_depth_image, self.intrinsics)

                        if self.enable_work_area_sliders:
                            if not contour_data.centroid_within_bounds(self.x_limits.value,
                                                                       self.y_limits.value):
                                include_contour = False
                        elif self.enable_work_area_apriltags and self.work_area_apriltags_detected:
                            if not contour_data.coord_within_bounds(self.work_area_limits_y,
                                                                    self.work_area_limits_z):
                                include_contour = False

                        if self.enable_depth_filter:
                            if not contour_data.depth_within_bounds(self.get_depth_limits()):
                                include_contour = False

                        if include_contour:
                            self.contours_filtered_ally.append(contour_data)

        # sort contours by x or y coordinate in image
        if self.sort_by_x:
            self.contours_filtered_ally.sort(key=lambda contour: contour.centroid.x)
        else:
            self.contours_filtered_ally.sort(key=lambda contour: contour.centroid.y)

        # Threshold HSV image to get only enemy color
        if not self.invert_enemy_hue:
            mask_enemy = cv2.inRange(
                hsv_image,
                self.enemy_hsv.lower.to_np_array(),
                self.enemy_hsv.upper.to_np_array()
            )
        else:
            # Split into 2 inRange calls and logically and them
            temp_lower = self.enemy_hsv.lower.to_np_array()
            temp_upper = self.enemy_hsv.upper.to_np_array()

            temp_lower[0] = 0
            temp_upper[0] = self.enemy_hsv.lower.H

            mask_enemy = cv2.inRange(hsv_image, temp_lower, temp_upper)

            temp_lower[0] = self.enemy_hsv.upper.H
            temp_upper[0] = 255

            mask_enemy += cv2.inRange(hsv_image, temp_lower, temp_upper)

        # Get contours of ally
        ret_enemy, thresh_enemy = cv2.threshold(mask_enemy, 127, 255, 0)
        opening_enemy = cv2.morphologyEx(thresh_enemy, cv2.MORPH_OPEN, kernel)
        opening_then_closing_enemy = cv2.morphologyEx(opening_enemy, cv2.MORPH_CLOSE, kernel)
        contours_enemy, hierarchy_enemy = cv2.findContours(
            opening_then_closing_enemy,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        self.contours_filtered_enemy = []

        # Make sure we have contours
        if len(contours_enemy) > 0:

            for contour in contours_enemy:
                if cv2.contourArea(contour) > self.area_threshold:
                    contour_data = ContourData(contour, ObjectType.ENEMY)

                    # Only add contour to list if it is valid
                    if contour_data.valid:
                        include_contour = True

                        # Find real world coordinates of all contours
                        contour_data.calc_coord(self.aligned_depth_image, self.intrinsics)

                        if self.enable_work_area_sliders:
                            if not contour_data.centroid_within_bounds(self.x_limits.value,
                                                                       self.y_limits.value):
                                include_contour = False
                        elif self.enable_work_area_apriltags and self.work_area_apriltags_detected:
                            if not contour_data.coord_within_bounds(self.work_area_limits_y,
                                                                    self.work_area_limits_z):
                                include_contour = False

                        if self.enable_depth_filter:
                            if not contour_data.depth_within_bounds(self.get_depth_limits()):
                                include_contour = False

                        if include_contour:
                            self.contours_filtered_enemy.append(contour_data)

        # sort contours by x or y coordinate in image
        if self.sort_by_x:
            self.contours_filtered_enemy.sort(key=lambda contour: contour.centroid.x)
        else:
            self.contours_filtered_enemy.sort(key=lambda contour: contour.centroid.y)

        if self.enable_ally_sliders:
            cv2.imshow(self.ally_mask_window_name, mask_ally)
        if self.enable_enemy_sliders:
            cv2.imshow(self.enemy_mask_window_name, mask_enemy)

        time = self.get_clock().now().to_msg()

        detections = Detections()

        # Contour output to image, other places
        for i, contour in enumerate(self.contours_filtered_ally):
            # Add contours to image
            color_image_with_tracking = cv2.drawContours(
                color_image_with_tracking,
                [contour.contour],
                0,
                (255, 0, 0),
                3
            )

            # Add centroid to color image
            color_image_with_tracking = cv2.circle(
                color_image_with_tracking,
                (contour.centroid.x, contour.centroid.y),
                radius=10,
                color=(255, 0, 0),
                thickness=-1
            )

            color_image_with_tracking = cv2.putText(
                color_image_with_tracking, f'{i}',
                (contour.centroid.x - 5, contour.centroid.y + 5),
                cv2.FONT_HERSHEY_DUPLEX,
                0.5,
                (255, 255, 255)
            )

            # Broadcast transforms directly from this node
            if self.broadcast_transforms_directly:
                contour.broadcast(self.broadcaster, i, time)

            # Store for publishing
            detections.allies.append(contour.get_as_detection_object(i))

        # Contour output to image, other places
        for i, contour in enumerate(self.contours_filtered_enemy):
            # Add contours to image
            color_image_with_tracking = cv2.drawContours(
                color_image_with_tracking,
                [contour.contour],
                0,
                (0, 0, 255),
                3
            )

            # Add centroid to color image
            color_image_with_tracking = cv2.circle(
                color_image_with_tracking,
                (contour.centroid.x, contour.centroid.y),
                radius=10,
                color=(0, 0, 255),
                thickness=-1
            )

            color_image_with_tracking = cv2.putText(
                color_image_with_tracking, f'{i}',
                (contour.centroid.x - 5, contour.centroid.y + 5),
                cv2.FONT_HERSHEY_DUPLEX,
                0.5,
                (255, 255, 255)
            )

            # Broadcast transforms directly from this node
            if self.broadcast_transforms_directly:
                contour.broadcast(self.broadcaster, i, time)

            # Store for publishing
            detections.enemies.append(contour.get_as_detection_object(i))

        # Publish detections
        self.pub_object_detections.publish(detections)

        # Add work area bounds to image
        if self.enable_work_area_sliders:
            color_image_with_tracking = cv2.rectangle(
                color_image_with_tracking,
                (self.x_limits.value.lower, self.y_limits.value.lower),
                (self.x_limits.value.upper, self.y_limits.value.upper),
                color=(0, 255, 0),
                thickness=1
            )
        elif self.enable_work_area_apriltags:
            color_image_with_tracking = cv2.rectangle(
                color_image_with_tracking,
                (self.workspace_min_pixel.x, self.workspace_min_pixel.y),
                (self.workspace_max_pixel.x, self.workspace_max_pixel.y),
                color=(0, 255, 0),
                thickness=1
            )

        color_image_with_tracking = cv2.putText(
            color_image_with_tracking,
            f'Allies: {len(self.contours_filtered_ally)}',
            (50, 50),
            cv2.FONT_HERSHEY_DUPLEX,
            1,
            (255, 0, 0)
        )
        color_image_with_tracking = cv2.putText(
            color_image_with_tracking,
            f'Enemies: {len(self.contours_filtered_enemy)}',
            (50, 100),
            cv2.FONT_HERSHEY_DUPLEX,
            1,
            (0, 0, 255)
        )
        color_image_with_tracking = cv2.putText(
            color_image_with_tracking,
            f'Enemies Vanquished!: {self.dead_enemies_count}',
            (50, 150),
            cv2.FONT_HERSHEY_DUPLEX,
            1,
            (0, 0, 128)
        )

        cv2.imshow(self.color_window_name, color_image_with_tracking)
        cv2.waitKey(1)

    def get_transforms(self):
        """
        Obtain necessary transformations.

        Get transforms  between camera and panda table and broadcast them.
        Get transforms between camera and workspace diagonal ends and broadcast them.
        Calculate the workspace from the transforms and make a bounding box depicting it.
        Fix frames to latest transform in case we stop receiving transforms from AprilTags.
        Calibrate AprilTag locations over multiple received transformations.
        """
        time = self.get_clock().now().to_msg()

        # Get panda table transformations if possible
        try:
            self.tf_camera_to_robot_table_raw = self.buffer.lookup_transform(
                FRAMES().CAMERA_COLOR,
                FRAMES().PANDA_TABLE_RAW,
                rclpy.time.Time()
            )
            robot_table_detected = True
        except Exception:
            robot_table_detected = False

        # Get workspace transformations if possible
        if self.enable_work_area_apriltags:
            try:
                self.tf_camera_to_workspace1_raw = self.buffer.lookup_transform(
                    FRAMES().CAMERA_COLOR,
                    FRAMES().WORK_TABLE1_RAW,
                    rclpy.time.Time()
                )
                workspace1_detected = True
            except Exception:
                workspace1_detected = False

            try:
                self.tf_camera_to_workspace2_raw = self.buffer.lookup_transform(
                    FRAMES().CAMERA_COLOR,
                    FRAMES().WORK_TABLE2_RAW,
                    rclpy.time.Time()
                )
                workspace2_detected = True
            except Exception:
                workspace2_detected = False

        robot_table_transform = None

        # Broadcast last known transforms even if we lose AprilTags
        if self.tf_camera_to_robot_table_raw is not None:

            # If updating continuously, always send most recent available transform
            if self.update_calibration_continuously:
                robot_table_transform = copy.deepcopy(self.tf_camera_to_robot_table_raw)

            # Otherwise, send calibrated transform
            else:
                # collect data if performing a calibration
                if self.calibrating_robot_table:
                    # If all data is collected, update the calibrated transform
                    if len(self.robot_table_calibration_array) >= self.calibration_data_points:
                        self.tf_camera_to_robot_table_calibrated = \
                            copy.deepcopy(self.tf_camera_to_robot_table_raw)

                        # Calculate average translation
                        self.tf_camera_to_robot_table_calibrated.transform = \
                            get_average_transformation(self.robot_table_calibration_array)

                        self.get_logger().info('Robot table AprilTag calibration complete')

                        self.calibrating_robot_table = False

                    # Collect data
                    elif robot_table_detected:
                        self.robot_table_calibration_array.append(
                            copy.deepcopy(self.tf_camera_to_robot_table_raw.transform))

                # if a calibration transform has been created, send it.
                # Otherwise send latest transform
                if self.tf_camera_to_robot_table_calibrated is not None:
                    robot_table_transform = copy.deepcopy(self.tf_camera_to_robot_table_calibrated)
                else:
                    robot_table_transform = copy.deepcopy(self.tf_camera_to_robot_table_raw)

            robot_table_transform.child_frame_id = FRAMES().PANDA_TABLE
            robot_table_transform.header.stamp = time
            self.broadcaster.sendTransform(robot_table_transform)

        workspace1_transform = None

        # Broadcast last known transforms even if we lose AprilTags
        if self.tf_camera_to_workspace1_raw is not None:

            # If updating continuously, always send most recent available transform
            if self.update_calibration_continuously:
                workspace1_transform = copy.deepcopy(self.tf_camera_to_workspace1_raw)

            # Otherwise, send calibrated transform
            else:
                # collect data if performing a calibration
                if self.calibrating_workspace1:
                    # If all data is collected, update the calibrated transform
                    if len(self.workspace1_calibration_array) >= self.calibration_data_points:
                        self.tf_camera_to_workspace1_calibrated = \
                            copy.deepcopy(self.tf_camera_to_workspace1_raw)

                        # Calculate average translation
                        self.tf_camera_to_workspace1_calibrated.transform = \
                            get_average_transformation(self.workspace1_calibration_array)

                        self.get_logger().info('Workspace 1 AprilTag calibration complete')

                        self.calibrating_workspace1 = False

                    # Collect data
                    elif workspace1_detected:
                        self.workspace1_calibration_array.append(
                            copy.deepcopy(self.tf_camera_to_workspace1_raw.transform))

                # if a calibration transform has been created, send it.
                # Otherwise send latest transform
                if self.tf_camera_to_workspace1_calibrated is not None:
                    workspace1_transform = copy.deepcopy(self.tf_camera_to_workspace1_calibrated)
                else:
                    workspace1_transform = copy.deepcopy(self.tf_camera_to_workspace1_raw)

            workspace1_transform.child_frame_id = FRAMES().WORK_TABLE1
            workspace1_transform.header.stamp = time
            self.broadcaster.sendTransform(workspace1_transform)

        workspace2_transform = None

        # Broadcast last known transforms even if we lose AprilTags
        if self.tf_camera_to_workspace2_raw is not None:

            # If updating continuously, always send most recent available transform
            if self.update_calibration_continuously:
                workspace2_transform = copy.deepcopy(self.tf_camera_to_workspace2_raw)

            # Otherwise, send calibrated transform
            else:
                # collect data if performing a calibration
                if self.calibrating_workspace2:
                    # If all data is collected, update the calibrated transform
                    if len(self.workspace2_calibration_array) >= self.calibration_data_points:
                        self.tf_camera_to_workspace2_calibrated = \
                            copy.deepcopy(self.tf_camera_to_workspace2_raw)

                        # Calculate average translation
                        self.tf_camera_to_workspace2_calibrated.transform = \
                            get_average_transformation(self.workspace2_calibration_array)

                        self.get_logger().info('Workspace 2 AprilTag calibration complete')

                        self.calibrating_workspace2 = False

                    # Collect data
                    elif workspace2_detected:
                        self.workspace2_calibration_array.append(
                            copy.deepcopy(self.tf_camera_to_workspace2_raw.transform))

                # if a calibration transform has been created, send it.
                # Otherwise send latest transform
                if self.tf_camera_to_workspace2_calibrated is not None:
                    workspace2_transform = copy.deepcopy(self.tf_camera_to_workspace2_calibrated)
                else:
                    workspace2_transform = copy.deepcopy(self.tf_camera_to_workspace2_raw)

            workspace2_transform.child_frame_id = FRAMES().WORK_TABLE2
            workspace2_transform.header.stamp = time
            self.broadcaster.sendTransform(workspace2_transform)

        self.work_area_apriltags_detected = (
            (workspace1_transform is not None)
            and (workspace2_transform is not None)
        )

        if self.work_area_apriltags_detected:
            # Determine bounds from work area AprilTags
            self.work_area_limits_y.lower = min(
                workspace1_transform.transform.translation.y,
                workspace2_transform.transform.translation.y
            ) - self.work_area_tag_size / 2.
            self.work_area_limits_y.upper = max(
                workspace1_transform.transform.translation.y,
                workspace2_transform.transform.translation.y
            ) + self.work_area_tag_size / 2.
            self.work_area_limits_z.lower = min(
                workspace1_transform.transform.translation.z,
                workspace2_transform.transform.translation.z
            ) - self.work_area_tag_size / 2.
            self.work_area_limits_z.upper = max(
                workspace1_transform.transform.translation.z,
                workspace2_transform.transform.translation.z
            ) + self.work_area_tag_size / 2.
            table_depth = min(
                workspace1_transform.transform.translation.x,
                workspace2_transform.transform.translation.x
            )

            # Get pixel coordinates to draw on picture
            self.tf_workspace_min = TransformStamped()
            self.tf_workspace_min.transform.translation.x = table_depth
            self.tf_workspace_min.transform.translation.y = self.work_area_limits_y.lower
            self.tf_workspace_min.transform.translation.z = self.work_area_limits_z.lower

            self.tf_workspace_max = TransformStamped()
            self.tf_workspace_max.transform.translation.x = table_depth
            self.tf_workspace_max.transform.translation.y = self.work_area_limits_y.upper
            self.tf_workspace_max.transform.translation.z = self.work_area_limits_z.upper

            self.workspace_min_pixel.get_from_camera_coordinates(
                self.intrinsics,
                self.tf_workspace_min.transform.translation
            )
            self.workspace_max_pixel.get_from_camera_coordinates(
                self.intrinsics,
                self.tf_workspace_max.transform.translation
            )

    def trackbar_filter_kernel(self, val):
        """Adjust filter kernel value when trackbar is moved through OpenCV."""
        self.filter_kernel = val

    def trackbar_area_threshold(self, val):
        """Adjust area threshold value when trackbar is moved through OpenCV."""
        self.area_threshold = val

    def color_image_callback(self, data):
        """Use cvbridge to get color_image data from Intel RealSense in ROS2."""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
            return

    def aligned_depth_image_callback(self, data):
        """Use cvbridge to get aligned_depth_image data from Intel RealSense in ROS2."""
        try:
            self.aligned_depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
            return

    def color_info_callback(self, info):
        """Collect camera information published by the ROS2 RealSense nodes."""
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

    def enemy_deadcount_callback(self, msg):
        """Get published dead enemy count."""
        self.dead_enemies_count = msg.data

    def start_apriltag_calibration_callback(self, request, response):
        """Start AprilTag calibration when the start_apriltag_calibration service is called."""
        self.calibrating_robot_table = True
        self.robot_table_calibration_array = []

        self.calibrating_workspace1 = True
        self.workspace1_calibration_array = []

        self.calibrating_workspace2 = True
        self.workspace2_calibration_array = []

        self.update_calibration_continuously = False

        return response

    def update_calibration_continuously_callback(self, request, response):
        """Switch to updating AprilTag positions continuously."""
        self.update_calibration_continuously = True

        return response

    def get_depth_limits(self):
        """Convert stored depth limits from mm to m."""
        limits = Limits()
        limits.lower = self.depth_limits.value.lower / 1000.
        limits.upper = self.depth_limits.value.upper / 1000.

        return limits


def entry(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    rclpy.spin(camera_processor)
    rclpy.shutdown()
