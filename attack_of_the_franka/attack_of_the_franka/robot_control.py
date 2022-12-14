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
Robot Control Node.

Description: Robot Control node contains a state machine and services to allow our Franka robot
                to detect allies and enemies. Once it is commanded to start attacking enemies,
                the robot will attempt to attack each enemy in each style with checks for if it
                will knock over an enemy. After knocking down all enemies it can without hitting
                allies, it returns to idle.

Parameters passed into the Node:
    parameters: the custom parameters contained within the robot_control node that have a default
            but can be changed by the user

Publishers:
    dead_count_pub (Int16): publishes the number of enemies that have been killed
    pub_joint_traj (trajectory_msgs.msg.JointTrajectory): publishes robot's joint state trajectory

Clients:
    gripper_action_client (control_msgs.action.GripperCommand): an action client controlling
                                                                gripper movement
    test_followjoints (control_msgs.action.FollowJointTrajectory): action client for
                                                                    joint trajectory
    grip_lightsaber_client (franka_msgs.action.Grasp): action client for commanding panda gripper
                                                        to hold lightsaber

Authors:
    Megan Sindelar
    Nick Morales
    Sushma Chandra
    Vaishnavi Dornadula

Last Updated: December 8th, 2022
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_testing.moveit_interface import MoveIt, MoveConfig, MoveItApiErrors
import geometry_msgs.msg
import moveit_msgs.action
import moveit_msgs.srv
import moveit_msgs.msg
import control_msgs.action
import std_srvs.srv
from enum import Enum, auto
import shape_msgs.msg
import moveit_testing_interfaces.srv
import trajectory_msgs.msg
import copy
import franka_msgs.action
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from attack_of_the_franka.common import FRAMES, ObjectType
from rcl_interfaces.msg import ParameterDescriptor
from attack_of_the_franka_interfaces.msg import Detections
from std_msgs.msg import Int16
import time


class State(Enum):
    """Top level states for main state machine."""

    IDLE = auto(),
    MOVE_TO_HOME_START = auto(),
    MOVE_TO_HOME_WAIT = auto(),
    PLAN_TO_POSE_START = auto(),
    PLAN_TO_POSITION_START = auto(),
    PLAN_TO_ORIENTATION_START = auto(),
    PLAN_WAIT = auto(),
    EXECUTE_START = auto(),
    EXECUTE_WAIT = auto(),
    WAYPOINTS = auto(),
    WAYPOINTS_WAIT = auto(),
    NEXT_WAYPOINT = auto(),
    LOOK_FOR_ENEMY = auto(),
    CHECK_FOR_ENEMY_REMAINING = auto(),
    RESET_ALLIES = auto(),
    RESET_ALLIES_WAIT = auto(),
    FIND_ALLIES = auto(),
    FIND_ALLIES_WAIT = auto(),
    DYNAMIC_MOTION = auto(),
    LEFT_DYNAMIC_MOTION = auto(),
    RIGHT_DYNAMIC_MOTION = auto(),
    STAB_MOTION = auto(),
    PICKUP_LIGHTSABER = auto(),


class PickupLightsaberState(Enum):
    """State machine for lightsaber pickup subsequence."""

    ADD_WALLS_START = auto(),
    ADD_WALLS_WAIT = auto(),
    REMOVE_ATTACHED_COLLISION_START = auto(),
    REMOVE_ATTACHED_COLLISION_WAIT = auto()
    ADD_SEPARATE_COLLISION_START = auto(),
    ADD_SEPARATE_COLLISION_WAIT = auto(),
    MOVE_TO_HOME_START = auto(),
    MOVE_TO_HOME_WAIT = auto(),
    OPEN_START = auto(),
    OPEN_WAIT = auto(),
    MOVE_TO_LIGHTSABER_STANDOFF_START = auto(),
    MOVE_TO_LIGHTSABER_STANDOFF_WAIT = auto(),
    REMOVE_SEPARATE_COLLISION_START = auto(),
    REMOVE_SEPARATE_COLLISION_WAIT = auto(),
    MOVE_TO_LIGHTSABER_PICK_START = auto(),
    MOVE_TO_LIGHTSABER_PICK_WAIT = auto(),
    GRASP_START = auto(),
    GRASP_WAIT = auto(),
    ADD_ATTACHED_COLLISION_START = auto(),
    ADD_ATTACHED_COLLISION_WAIT = auto()
    DRAW_START = auto(),
    DRAW_WAIT = auto(),
    RETURN_TO_HOME_START = auto(),
    RETURN_TO_HOME_WAIT = auto(),


class DetectedObjectData():

    def __init__(self, obj):
        self.obj = obj
        self.tf = None


class RobotControl(Node):
    """
    Control robot and planning scene using the moveit_interface API.

    Services:
        move_to_home (std_srvs.srv.Empty): moves the robot to a predetermined home position
        gripper_open (std_srvs.srv.Empty): moves the robot's end-effector position to open
        gripper_close (std_srvs.srv.Empty): moves the robot's end-effector position to close
        gripper_grasp (std_srvs.srv.Empty): moves the robot's end-effector together until
                                            it's grasping an object between them
        waypoints (std_srvs.srv.Empty): commands the robot to move to a predetermined waypoint
        test_joint_trajectory (std_srvs.srv.Empty): commands robot to a predetermined waypoint
                                                        using the joint angles
        move_to_pose (moveit_testing_interfaces/srv/MoveToPose): move robot to specific position
                                                                 and orientation
        joint_waypoint (std_srvs.srv.Empty): command specific joint positions
        move_to_position (moveit_testing_interfaces/srv/MoveToPosition): move robot to specific
                                                                         position
        move_to_orientation (moveit_testing_interfaces/srv/MoveToOrientation): move robot to
                                                                               specific orientation
        update_obstacles (moveit_testing_interfaces/srv/UpdateObstacles): add obstacles to scene
        pickup_lightsaber (std_srvs.srv.Empty): commands the robot to follow waypoints to pick up
                                                pick up the fixed lightsaber with the end-effector
        update_persistent_obstacles (moveit_testing_interfaces.srv.UpdateObstacles): adds obstacle
                                                                                     to scene
                                                                                     permanently
        update_attached_obstacles (moveit_testing_interfaces.srv.UpdateAttachedObstacles): adds
                                                                                           obstacle
                                                                                           attached
                                                                                           to robot
                                                                                           links
        home_waypoint (std_srvs.srv.Empty): plans to a hard coded value in the callback
        add_walls (std_srvs.srv.Empty): adds walls and ceiling collision objects
                                        to the planning scene
        remove_separate_lightsaber (std_srvs.srv.Empty): removes the lightsaber as collision object
        add_separate_lightsaber (std_srvs.srv.Empty): add lightsaber as a separate collision object
        remove_attached_lightsaber (std_srvs.srv.Empty): remove the lightsaber attached to the
                                                         end-effector in the plannin scene in rviz
        add_attached_lightsaber (std_srvs.srv.Empty): add the lightsaber attached to end-effector
                                                      to the planning scene as an attached
                                                      collision object.
        look_for_enemy (std_srvs.srv.Empty): check for any enemies detected in the planning scene
                                             and begins to calculate how to attack if possible
        reset_allies (std_srvs.srv.Empty): updates the position of the allies in the planning
                                            scene to current

    Timers:
        timer: main timer that runs at 100 Hz
    """

    def __init__(self):
        """Class constructor."""
        super().__init__('robot_control')

        self.interval = 1.0 / 100.0
        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.pub_joint_traj = self.create_publisher(trajectory_msgs.msg.JointTrajectory,
                                                    'panda_arm_controller/joint_trajectory', 10)
        self.sub_obj_detections = \
            self.create_subscription(Detections,
                                     'object_detections', self.obj_detection_callback, 10)
        self.srv_move_to_home = self.create_service(std_srvs.srv.Empty,
                                                    'move_to_home', self.move_to_home_callback)
        self.srv_gripper_open = self.create_service(std_srvs.srv.Empty,
                                                    'gripper_open',
                                                    self.gripper_open_callback)
        self.srv_gripper_close = self.create_service(std_srvs.srv.Empty,
                                                     'gripper_close',
                                                     self.gripper_close_callback)
        self.gripper_grasp = self.create_service(std_srvs.srv.Empty,
                                                 'gripper_grasp', self.gripper_grasp_callback)
        self.waypoints = self.create_service(std_srvs.srv.Empty,
                                             'waypoints', self.waypoint_callback)
        self.srv_move_to_pose = self.create_service(moveit_testing_interfaces.srv.MoveToPose,
                                                    'move_to_pose', self.move_to_pose_callback)
        self.srv_joint_waypoint = self.create_service(std_srvs.srv.Empty, 'joint_waypoint',
                                                            self.joint_waypoint_callback)
        self.srv_move_to_position = \
            self.create_service(moveit_testing_interfaces.srv.MoveToPosition,
                                'move_to_position', self.move_to_position_callback)
        self.srv_move_to_orientation = \
            self.create_service(moveit_testing_interfaces.srv.MoveToOrientation,
                                'move_to_orientation', self.move_to_orientation_callback)
        self.srv_update_obstacles = \
            self.create_service(moveit_testing_interfaces.srv.UpdateObstacles,
                                'update_obstacles', self.obstacles_callback)
        self.srv_pickup_lightsaber = self.create_service(std_srvs.srv.Empty, 'pickup_lightsaber',
                                                         self.pickup_lightsaber_callback)
        self.gripper_action_client = ActionClient(self, control_msgs.action.GripperCommand,
                                                  'panda_gripper/gripper_action')
        self.test_followjoints = ActionClient(self, control_msgs.action.FollowJointTrajectory,
                                              'panda_gripper/follow_joint_trajectory')
        self.grip_lightsaber_client = ActionClient(self, franka_msgs.action.Grasp,
                                                   'panda_gripper/grasp')
        self.srv_update_persistent_obstacles = \
            self.create_service(moveit_testing_interfaces.srv.UpdateObstacles,
                                'update_persistent_obstacles', self.persistent_obstacles_callback)
        self.srv_update_attached_obstacles = \
            self.create_service(moveit_testing_interfaces.srv.UpdateAttachedObstacles,
                                'update_attached_obstacles', self.attached_obstacles_callback)
        self.srv_home_waypoint = self.create_service(std_srvs.srv.Empty, 'home_waypoint',
                                                     self.home_waypoint_callback)
        self.srv_add_walls = self.create_service(std_srvs.srv.Empty,
                                                 'add_walls', self.add_walls_callback)
        self.srv_remove_separate_lightsaber = \
            self.create_service(std_srvs.srv.Empty,
                                'remove_separate_lightsaber',
                                self.remove_separate_lightsaber_callback)
        self.srv_add_separate_lightsaber = \
            self.create_service(std_srvs.srv.Empty,
                                'add_separate_lightsaber',
                                self.add_separate_lightsaber_callback)
        self.srv_remove_attached_lightsaber = \
            self.create_service(std_srvs.srv.Empty,
                                'remove_attached_lightsaber',
                                self.remove_attached_lightsaber_callback)
        self.srv_add_attached_lightsaber = \
            self.create_service(std_srvs.srv.Empty,
                                'add_attached_lightsaber',
                                self.add_attached_lightsaber_callback)
        self.look_for_enemy_srv = self.create_service(std_srvs.srv.Empty,
                                                      'look_for_enemy',
                                                      self.look_for_enemy_callback)
        self.reset_allies = self.create_service(std_srvs.srv.Empty,
                                                'reset_allies',
                                                self.reset_allies_callback)

        self.tf_buffer = Buffer()
        self.tf_obj_listener = TransformListener(self.tf_buffer, self)

        # Control parameters
        self.declare_parameter("simulation", False,
                               ParameterDescriptor(description="If the robot is in simulation"))
        self.simulation = self.get_parameter("simulation").get_parameter_value().bool_value

        # Dimension parameters
        self.declare_parameter("robot_table.width", 0.605,
                               ParameterDescriptor(description="Robot table width"))
        self.robot_table_width = \
            self.get_parameter("robot_table.width").get_parameter_value().double_value
        self.declare_parameter("robot_table.length", 0.911,
                               ParameterDescriptor(description="Robot table length"))
        self.robot_table_length = \
            self.get_parameter("robot_table.length").get_parameter_value().double_value
        self.declare_parameter("robot_table.height", 0.827,
                               ParameterDescriptor(description="Robot table height"))
        self.robot_table_height = \
            self.get_parameter("robot_table.height").get_parameter_value().double_value
        self.declare_parameter("ceiling_height", 2.4,
                               ParameterDescriptor(description="Ceiling height from floor"))
        self.ceiling_height = \
            self.get_parameter("ceiling_height").get_parameter_value().double_value
        self.declare_parameter("side_wall.distance", 1.0,
                               ParameterDescriptor(description="Side wall distance from base"))
        self.side_wall_distance = \
            self.get_parameter("side_wall.distance").get_parameter_value().double_value
        self.declare_parameter("side_wall.height", 2.4,
                               ParameterDescriptor(description="Side wall height from floor"))
        self.side_wall_height = \
            self.get_parameter("side_wall.height").get_parameter_value().double_value
        self.declare_parameter("back_wall.distance", 0.75,
                               ParameterDescriptor(description="Back wall distance from base"))
        self.back_wall_distance = \
            self.get_parameter("back_wall.distance").get_parameter_value().double_value
        self.declare_parameter("back_wall.height", 0.46,
                               ParameterDescriptor(description="Back wall height from floor"))
        self.back_wall_height = \
            self.get_parameter("back_wall.height").get_parameter_value().double_value
        self.declare_parameter("lightsaber.diameter", 0.033,
                               ParameterDescriptor(description="Lightsaber diameter"))
        self.lightsaber_diameter = \
            self.get_parameter("lightsaber.diameter").get_parameter_value().double_value
        self.declare_parameter("lightsaber.full_length", 1.122,
                               ParameterDescriptor(description="Lightsaber full length"))
        self.lightsaber_full_length = \
            self.get_parameter("lightsaber.full_length").get_parameter_value().double_value
        self.declare_parameter("lightsaber.grip_offset", 0.125,
                               ParameterDescriptor(description="Lightsaber grip offset"))
        self.lightsaber_grip_offset = \
            self.get_parameter("lightsaber.grip_offset").get_parameter_value().double_value
        self.declare_parameter("lightsaber.start_location.x", 0.5715,
                               ParameterDescriptor(description="Lightsaber initial location x"))
        self.declare_parameter("lightsaber.start_location.y", -0.263525,
                               ParameterDescriptor(description="Lightsaber initial location y"))
        self.declare_parameter("lightsaber.start_location.z", -0.175,
                               ParameterDescriptor(description="Lightsaber initial location z"))
        self.lightsaber_start_location = geometry_msgs.msg.Point()
        self.lightsaber_start_location.x = \
            self.get_parameter("lightsaber.start_location.x").get_parameter_value().double_value
        self.lightsaber_start_location.y = \
            self.get_parameter("lightsaber.start_location.y").get_parameter_value().double_value
        self.lightsaber_start_location.z = \
            self.get_parameter("lightsaber.start_location.z").get_parameter_value().double_value
        self.declare_parameter("lightsaber.lift_height", 0.25,
                               ParameterDescriptor(description="Height to lift lightsaber up"))
        self.lightsaber_lift_height = \
            self.get_parameter("lightsaber.lift_height").get_parameter_value().double_value
        self.declare_parameter("gripper.height", 0.08,
                               ParameterDescriptor(description="height of gripper for collisions"))
        self.gripper_height = \
            self.get_parameter("gripper.height").get_parameter_value().double_value
        self.declare_parameter("gripper.tcp_offset", 0.036,
                               ParameterDescriptor(description="grip offset panda_hand_tcp_frame"))
        self.gripper_tcp_offset = \
            self.get_parameter("gripper.tcp_offset").get_parameter_value().double_value
        self.declare_parameter("speeds.default", 0.3,
                               ParameterDescriptor(description="Default speed limit multiplier"))
        self.default_speed = \
            self.get_parameter("speeds.default").get_parameter_value().double_value
        self.declare_parameter("speeds.pickup_lightsaber_speed_slow", 0.1,
                               ParameterDescriptor(description="Pickup lightsaber slow multiple"))
        self.pickup_lightsaber_speed_slow = \
            self.get_parameter("speeds.pickup_lightsaber_speed_slow"
                               ).get_parameter_value().double_value
        self.declare_parameter("speeds.pickup_lightsaber_speed_fast", 0.3,
                               ParameterDescriptor(description="Pickup lightsaber fast multiple"))
        self.pickup_lightsaber_speed_fast = \
            self.get_parameter("speeds.pickup_lightsaber_speed_fast"
                               ).get_parameter_value().double_value

        self.table_offset = 0.091

        # Initialize API class
        self.config = MoveConfig()
        self.config.base_frame_id = 'panda_link0'
        self.config.workspace_min_corner = geometry_msgs.msg.Vector3(
            x=-3.0,
            y=-3.0,
            z=-3.0
        )
        self.config.workspace_max_corner = geometry_msgs.msg.Vector3(
            x=3.0,
            y=3.0,
            z=3.0
        )
        self.config.tolerance = 0.01
        self.config.max_velocity_scaling_factor = self.default_speed
        self.config.group_name = 'panda_manipulator'

        self.home_pose = geometry_msgs.msg.Pose()
        self.waypoint_joints = []

        # Select end effector attributes based on group name
        if self.config.group_name == 'panda_arm':
            self.config.ee_frame_id = 'panda_link8'  # end effector frame for panda_arm
            self.home_pose.position.x = 0.30691
            self.home_pose.position.y = 0.0
            self.home_pose.position.z = 0.590282
            self.home_pose.orientation.x = 0.92388
            self.home_pose.orientation.y = -0.382683
            self.home_pose.orientation.z = 8.32667e-17
            self.home_pose.orientation.w = 8.32667e-17
        elif self.config.group_name == 'panda_manipulator':
            self.config.ee_frame_id = 'panda_hand_tcp'  # end effector frame for panda_manipulator
            self.home_pose.position.x = 0.306891
            self.home_pose.position.y = -8.32667e-17
            self.home_pose.position.z = 0.486882
            self.home_pose.orientation.x = 1.0
            self.home_pose.orientation.y = 1.38778e-16
            self.home_pose.orientation.z = 2.22045e-16
            self.home_pose.orientation.w = -6.93889e-17

        self.config.home_joint_positions = [
            0.0,                    # panda_joint1
            -0.7853981633974483,    # panda_joint2
            0.0,                    # panda_joint3
            -2.356194490192345,     # panda_joint4
            0.0,                    # panda_joint5
            (math.pi*5)/6,     # panda_joint6
            0.7853981633974483,     # panda_joint7
                                    # TODO - This might open the gripper when we try to move home
                                    # CAREFUL!
            0.0,                  # 0.035, 0.0 panda_finger_joint1
            0.0                 # 0.035, 0.0 panda_finger_joint2
        ]

        self.waypoint_joints = []

        self.moveit = MoveIt(self, self.config)

        self.goal_pose = geometry_msgs.msg.Pose()
        self.goal_waypoint = geometry_msgs.msg.Pose()
        self.plan = None

        self.state = State.IDLE
        self.state_last = None
        self.pickup_lightsaber_state = PickupLightsaberState.ADD_WALLS_START
        self.pickup_lightsaber_state_last = None

        self.joint_traj = trajectory_msgs.msg.JointTrajectory()

        self.ready = 0

        self.trajectory = control_msgs.action.FollowJointTrajectory.Goal()

        self.waypoints = 0

        self.flag = 0

        self.ind = 0

        self.home_waypoint = False

        self.update_allies = False

        self.count = 2

        self.table1_x = 1.22
        self.table1_z = self.table_offset
        self.table_len_x = 0.5
        self.table_len_y = 0.8
        self.table_center_x = 1.443
        self.table_center_y = -0.052

        self.obstacles_added = 0

        self.is_waypoint = True
        self.detected_objects = None
        self.detected_allies = []
        self.detected_enemies = []
        self.unreachable_enemies = []
        self.current_enemy = None

        self.num_movements = 0
        self.num_moves_completed = 0
        self.is_stab_motion = False

        self.block_height = 0.23
        self.block_width = 0.11
        self.block_length = 0.08
        self.ally_height_buffer = 1.5
        self.movement_direction_sign = 1
        self.dead_count_pub = self.create_publisher(Int16, 'enemy_dead_count', 10)
        self.dead_enemy_count = 0
        self.enemies_after = 0
        self.enemies_before = 0

        self.num_waypoints_completed = 0
        self.looking_for_enemies = 0

        self.open_count = 0

        self.add_walls()

        self.allies_reset_flag = False

        self.get_logger().info("robot_control node started")

    def obstacle_info(self):
        """
        Get robot transformation from base frame to end-effector frame.

        Args:
            none

        Returns
        -------
            GetPlanningScene.result (moveit_msgs/srv/GetPlanningScene)

        """
        self.obstacle_future = \
            self.obstacle_client.call_async(moveit_msgs.srv.GetPlanningScene.Request())
        print(f"Obstacle Info: {self.obstacle_future.result()}")
        self.x = 1
        return self.obstacle_future.result()

    def close_gripper(self):
        """Move the end-effector to the close position."""
        grip_msg = control_msgs.action.GripperCommand.Goal()
        grip_msg.command.position = 0.0
        grip_msg.command.max_effort = 60.0
        if self.gripper_action_client.server_is_ready():
            self.gripper_action_client.send_goal_async(grip_msg)
        else:
            return None

    def open_gripper(self):
        """Move the end-effector to the open position."""
        grip_msg = control_msgs.action.GripperCommand.Goal()
        grip_msg.command.position = 0.03
        if self.gripper_action_client.server_is_ready():
            return self.gripper_action_client.send_goal_async(grip_msg)
        else:
            return None

    def grasp(self):
        """Close the end-effector until it grasps an object."""
        if self.grip_lightsaber_client.server_is_ready():
            grip_goal = franka_msgs.action.Grasp.Goal()
            grip_goal.width = 0.00
            grip_goal.epsilon.inner = 0.005
            grip_goal.epsilon.outer = 0.005
            grip_goal.speed = 0.03
            grip_goal.force = 80.0
            return self.grip_lightsaber_client.send_goal_async(grip_goal)
        else:
            return None

    def find_allies(self):
        """
        Update the allies from the detected obstacles list and creates collision objects.

        Returns
        -------
            all_transforms_found: a boolean representing if any TransformExceptions were found
                                    while looking for transforms

        """
        all_transforms_found = self.update_detected_objects(ObjectType.ALLY)
        obstacle_list = []
        if all_transforms_found:
            for i in range(len(self.detected_allies)):
                obstacle = moveit_msgs.msg.CollisionObject()
                obstacle.id = self.detected_allies[i].obj.name

                shape = shape_msgs.msg.SolidPrimitive()
                shape.type = 1  # Box
                length = 0.078
                width = 0.105
                height = 0.2286
                shape.dimensions = [length, width, height * self.ally_height_buffer]
                obstacle.primitives = [shape]

                pose = geometry_msgs.msg.Pose()
                pose.position.x = self.detected_allies[i].tf.transform.translation.x
                pose.position.y = self.detected_allies[i].tf.transform.translation.y
                pose.position.z = -0.091 + (height/2)
                obstacle.primitive_poses = [pose]

                obstacle.header.frame_id = self.moveit.config.base_frame_id
                obstacle_list.append(obstacle)

            self.moveit.update_obstacles(obstacle_list, delete=False)

        return all_transforms_found

    def timer_callback(self):
        """Check states and call the specific plan or execute from MoveIt API accordingly."""
        # call MoveIt handler
        self.moveit.handle()

        if self.allies_reset_flag:
            if self.find_allies():
                self.allies_reset_flag = False

        new_state = self.state != self.state_last

        if new_state:
            self.get_logger().info(
                f"robot_control main sequence changed to {self.state.name}")
            self.state_last = self.state

        # State machine
        if self.state == State.IDLE:
            self.unreachable_enemies = []
            self.looking_for_enemies = 0
            self.is_stab_motion = False

        elif self.state == State.MOVE_TO_HOME_START:

            if self.moveit.planning:
                self.state = State.MOVE_TO_HOME_WAIT
            else:
                self.moveit.move_to_home()

        elif self.state == State.MOVE_TO_HOME_WAIT:
            if not self.moveit.busy:
                if self.movement_direction_sign == -1 and not self.is_stab_motion:
                    self.state = State.RIGHT_DYNAMIC_MOTION
                elif self.is_stab_motion and self.num_moves_completed != self.num_movements:
                    self.state = State.STAB_MOTION
                else:
                    if self.looking_for_enemies:
                        self.state = State.CHECK_FOR_ENEMY_REMAINING
                    else:
                        self.state = State.IDLE

        elif self.state == State.RESET_ALLIES:
            if self.moveit.busy_updating_obstacles:
                self.state = State.RESET_ALLIES_WAIT
            else:
                self.moveit.reset_obstacles(FRAMES.ALLY)

        elif self.state == State.RESET_ALLIES_WAIT:
            if not self.moveit.busy_updating_obstacles:
                self.state = State.FIND_ALLIES

        elif self.state == State.FIND_ALLIES:
            try:
                table1 = self.tf_buffer.lookup_transform(FRAMES.PANDA_BASE,
                                                         FRAMES.WORK_TABLE1,
                                                         rclpy.time.Time())
                table1_here = True
            except TransformException:
                table1_here = False
                return
            try:
                table2 = self.tf_buffer.lookup_transform(FRAMES.PANDA_BASE,
                                                         FRAMES.WORK_TABLE2,
                                                         rclpy.time.Time())
                table2_here = True
            except TransformException:
                table2_here = False
                return

            if table1_here and table2_here:
                self.table1_x = table1.transform.translation.x
                self.table1_z = table1.transform.translation.z
                self.table_len_x = \
                    table2.transform.translation.x - table1.transform.translation.x + 0.1
                self.table_len_y = \
                    abs(table1.transform.translation.y) + \
                    abs(table2.transform.translation.y) + 0.06
                self.table_center_x = \
                    ((abs(table1.transform.translation.x - table2.transform.translation.x))/2) + \
                    table1.transform.translation.x
                self.table_center_y = \
                    (table1.transform.translation.y + table2.transform.translation.y)/2

            # Wait for all ally transforms to be detected
            if self.update_detected_objects(ObjectType.ALLY):
                obstacle_list = []
                for i in range(len(self.detected_allies)):
                    obstacle = moveit_msgs.msg.CollisionObject()
                    obstacle.id = self.detected_allies[i].obj.name

                    shape = shape_msgs.msg.SolidPrimitive()
                    shape.type = 1  # Box
                    length = 0.078
                    width = 0.105
                    height = 0.2286
                    shape.dimensions = [length, width, height * self.ally_height_buffer]
                    obstacle.primitives = [shape]

                    pose = geometry_msgs.msg.Pose()
                    pose.position.x = self.detected_allies[i].tf.transform.translation.x
                    pose.position.y = self.detected_allies[i].tf.transform.translation.y
                    pose.position.z = -0.091 + (height/2)
                    obstacle.primitive_poses = [pose]

                    obstacle.header.frame_id = self.moveit.config.base_frame_id
                    obstacle_list.append(obstacle)

                self.moveit.update_obstacles(obstacle_list, delete=False)
                self.state = State.FIND_ALLIES_WAIT

        elif self.state == State.FIND_ALLIES_WAIT:
            if not self.moveit.busy_updating_obstacles:
                time.sleep(1)
                self.state = State.LOOK_FOR_ENEMY

        elif self.state == State.LOOK_FOR_ENEMY:
            self.looking_for_enemies = 1
            self.waypoints = 0
            self.num_moves_completed = 0
            self.num_waypoints_completed = 0
            all_transforms_found = self.update_detected_objects(ObjectType.ENEMY)
            self.get_logger().info(f'number of enemies on field: {len(self.detected_enemies)}')
            self.x_disp = []
            self.rotate = []
            self.waypoint_movements = []
            self.right_waypoint_movements = []
            self.left_goal_waypoint = geometry_msgs.msg.Pose()
            self.right_goal_waypoint = geometry_msgs.msg.Pose()
            self.right_knock_enemy_waypoint = geometry_msgs.msg.Pose()
            if len(self.detected_enemies) == 0:
                self.state = State.CHECK_FOR_ENEMY_REMAINING
            if not self.moveit.busy:
                if all_transforms_found:
                    self.enemies_before = len(self.detected_enemies)
                    if len(self.detected_enemies) > 0:

                        for i in range(len(self.detected_enemies)):
                            # select first enemy not in unreachable enemies list
                            enemy_reachable = True
                            for enemy in self.unreachable_enemies:
                                if enemy.obj.name == self.detected_enemies[i].obj.name:
                                    enemy_reachable = False
                                    break

                            if enemy_reachable:
                                break

                        # prevent array bounds error
                        if not enemy_reachable:
                            self.state = State.CHECK_FOR_ENEMY_REMAINING
                            return

                        self.current_enemy = self.detected_enemies[i]

                        self.x_disp.append(self.detected_enemies[i].tf.transform.translation.x -
                                           self.table1_x + 0.1)
                        self.rotate.append(math.atan2(
                            self.detected_enemies[i].tf.transform.translation.y,
                            self.detected_enemies[i].tf.transform.translation.x))

                        x_pos = self.detected_enemies[i].tf.transform.translation.x
                        y_pos = self.detected_enemies[i].tf.transform.translation.y
                        if self.detected_enemies[i].tf.transform.translation.z < 0.1:
                            height = 3*self.block_height/4
                            waypoint_height_correction = 0.16
                        else:
                            height = self.detected_enemies[i].tf.transform.translation.z
                            waypoint_height_correction = 0.2

                        waypoint_angle = -math.radians(9.)

                        # left waypoint
                        self.left_goal_waypoint.position.x = \
                            x_pos - (self.lightsaber_full_length*0.75)
                        self.left_goal_waypoint.position.y = y_pos + 0.12
                        self.left_goal_waypoint.position.z = \
                            -self.table_offset + height + waypoint_height_correction
                        self.get_logger().info(f'goal z: {self.goal_waypoint.position.z}')
                        self.left_goal_waypoint.orientation.x = math.pi
                        self.left_goal_waypoint.orientation.z = waypoint_angle

                        self.left_knock_enemy_waypoint = geometry_msgs.msg.Pose()
                        self.left_knock_enemy_waypoint.position.x = \
                            x_pos - (self.lightsaber_full_length*0.75)
                        self.left_knock_enemy_waypoint.position.y = y_pos - 0.05
                        self.left_knock_enemy_waypoint.position.z = \
                            -self.table_offset + height + waypoint_height_correction
                        self.left_knock_enemy_waypoint.orientation.x = math.pi
                        self.left_knock_enemy_waypoint.orientation.z = waypoint_angle

                        self.left_reverse_enemy_waypoint = geometry_msgs.msg.Pose()
                        self.left_reverse_enemy_waypoint.position.x = \
                            x_pos - (self.lightsaber_full_length*0.75)
                        self.left_reverse_enemy_waypoint.position.y = y_pos + 0.12
                        self.left_reverse_enemy_waypoint.position.z = \
                            -self.table_offset + height + waypoint_height_correction
                        self.left_reverse_enemy_waypoint.orientation.x = math.pi
                        self.left_reverse_enemy_waypoint.orientation.z = waypoint_angle

                        # right waypoint
                        self.right_goal_waypoint.position.x = \
                            x_pos - (self.lightsaber_full_length*0.75)
                        self.right_goal_waypoint.position.y = y_pos - 0.12
                        self.right_goal_waypoint.position.z = \
                            -self.table_offset + height + waypoint_height_correction
                        self.get_logger().info(f'goal z: {self.right_goal_waypoint.position.z}')
                        self.right_goal_waypoint.orientation.x = math.pi
                        self.right_goal_waypoint.orientation.z = waypoint_angle

                        self.right_knock_enemy_waypoint = geometry_msgs.msg.Pose()
                        self.right_knock_enemy_waypoint.position.x = \
                            x_pos - (self.lightsaber_full_length*0.75)
                        self.right_knock_enemy_waypoint.position.y = y_pos + 0.05
                        self.right_knock_enemy_waypoint.position.z = \
                            -self.table_offset + height + waypoint_height_correction
                        self.right_knock_enemy_waypoint.orientation.x = math.pi
                        self.right_knock_enemy_waypoint.orientation.z = waypoint_angle

                        self.right_reverse_enemy_waypoint = geometry_msgs.msg.Pose()
                        self.right_reverse_enemy_waypoint.position.x = \
                            x_pos - (self.lightsaber_full_length*0.75)
                        self.right_reverse_enemy_waypoint.position.y = y_pos - 0.12
                        self.right_reverse_enemy_waypoint.position.z = \
                            -self.table_offset + height + waypoint_height_correction
                        self.right_reverse_enemy_waypoint.orientation.x = math.pi
                        self.right_reverse_enemy_waypoint.orientation.z = waypoint_angle

                        self.get_logger().info("adding waypoints")
                        self.waypoint_movements.append([self.left_goal_waypoint,
                                                        self.left_knock_enemy_waypoint,
                                                        self.left_reverse_enemy_waypoint])
                        self.right_waypoint_movements.append([self.right_goal_waypoint,
                                                              self.right_knock_enemy_waypoint,
                                                              self.right_reverse_enemy_waypoint])
                        self.num_movements = len(self.waypoint_movements[0])
                        ####################
                        # Testing check for if ally would be hit by falling enemy block
                        ####################
                        if self.check_ally_danger_fall(self.detected_enemies[i], 0):
                            self.state = State.LEFT_DYNAMIC_MOTION
                        elif self.check_ally_danger_fall(self.detected_enemies[i], 1):
                            self.state = State.RIGHT_DYNAMIC_MOTION
                            self.movement_direction_sign = -1
                        elif self.check_ally_danger_fall(self.detected_enemies[i], 2):
                            self.state = State.STAB_MOTION
                            self.movement_direction_sign = -1
                            self.is_stab_motion = True
                        else:
                            self.get_logger().error("ALLY IN DANGER, CAN'T ATTACK ENEMY")
                            self.unreachable_enemies.append(copy.deepcopy(self.current_enemy))
                            self.state = State.CHECK_FOR_ENEMY_REMAINING
                    else:
                        self.state = State.CHECK_FOR_ENEMY_REMAINING

        elif self.state == State.LEFT_DYNAMIC_MOTION:
            self.get_logger().info("Left Dyanmic Motion")
            if self.moveit.planning:
                self.state = State.WAYPOINTS_WAIT
            else:
                if self.waypoint_movements:
                    try:
                        self.moveit.plan_traj_to_pose(
                            self.waypoint_movements[
                                self.num_waypoints_completed][self.num_moves_completed])
                        self.num_moves_completed += 1
                        if self.num_moves_completed % self.num_movements == 0:
                            self.num_waypoints_completed += 1
                        if self.num_moves_completed == 1:
                            self.config.max_velocity_scaling_factor = self.default_speed + 0.4
                        else:
                            self.config.max_velocity_scaling_factor = self.default_speed
                    except Exception:
                        self.get_logger().info("back to idle!")
                        self.state = State.IDLE
                        pass

        elif self.state == State.RIGHT_DYNAMIC_MOTION:
            self.movement_direction_sign = -1
            self.get_logger().info(f'Moveit State: {self.moveit._plan_state}')
            self.get_logger().info("Right Dyanmic Motion")
            self.get_logger().info(f'num_waypoints completed: {self.num_waypoints_completed}')
            self.get_logger().info(f'num moves completed: {self.num_moves_completed}')
            if self.moveit.planning:
                self.state = State.WAYPOINTS_WAIT
            else:
                if self.waypoint_movements:
                    try:
                        self.moveit.plan_traj_to_pose(
                            self.right_waypoint_movements[
                                self.num_waypoints_completed][self.num_moves_completed])
                        self.num_moves_completed += 1
                        if self.num_moves_completed % self.num_movements == 0:
                            self.num_waypoints_completed += 1
                        if self.num_moves_completed == 1:
                            self.config.max_velocity_scaling_factor = self.default_speed + 0.4
                        else:
                            self.config.max_velocity_scaling_factor = self.default_speed
                    except Exception:
                        self.get_logger().info("back to idle!")
                        self.state = State.IDLE
                        pass

        elif self.state == State.STAB_MOTION:
            self.get_logger().info("Stab Motion")
            joint_waypoints = []
            if self.moveit.planning:
                self.state = State.WAYPOINTS_WAIT
            else:
                #####################################
                # Come in from center
                #####################################
                # goal waypoint

                joint1_start = math.radians(0)
                joint2_start = math.radians(-31)
                joint3_start = math.radians(-1)
                joint4_start = math.radians(-156)
                joint5_start = math.radians(0)
                joint6_start = math.radians(122)
                joint7_start = math.radians(45)

                joint1_end = math.radians(0)
                joint2_end = math.radians(49)
                joint3_end = math.radians(-1)
                joint4_end = math.radians(-76)
                joint5_end = math.radians(0)
                joint6_end = math.radians(116)
                joint7_end = math.radians(45)

                joint1_range = joint1_end - joint1_start
                joint2_range = joint2_end - joint2_start
                joint3_range = joint3_end - joint3_start
                joint4_range = joint4_end - joint4_start
                joint5_range = joint5_end - joint5_start
                joint6_range = joint6_end - joint6_start
                joint7_range = joint7_end - joint7_start

                stab_adjust_start = math.radians(0)
                stab_adjust_end = math.radians(8)
                stab_adjust_range = stab_adjust_end - stab_adjust_start

                table_length = 0.4826

                self.is_waypoint = False
                for i in range(len(self.x_disp)):
                    self.waypoint_joints1 = [joint1_start + self.rotate[i],     # panda_joint1
                                             -0.7853981633974483,       # panda_joint2
                                             0.0,                       # panda_joint3
                                             -2.356194490192345,        # panda_joint4
                                             0.0,                       # panda_joint5
                                             (math.pi*5)/6,             # panda_joint6
                                             0.7853981633974483,        # panda_joint7
                                             0.0,                       # panda_finger_jointl
                                             0.0                        # panda_finger_jointr
                                             ]

                    self.waypoint_joints2 = [joint1_start + self.rotate[i],     # panda_joint1
                                             joint2_start,      # panda_joint2
                                             joint3_start,      # panda_joint3
                                             joint4_start,      # panda_joint4
                                             joint5_start,      # panda_joint5
                                             joint6_start,      # panda_joint6
                                             joint7_start,      # panda_joint7
                                             0.0,                # panda_finger_jointl
                                             0.0                 # panda_finger_jointr
                                             ]

                    self.waypoint_joints3 = [joint1_start + joint1_range / table_length
                                             * self.x_disp[i] + self.rotate[i],
                                             joint2_start
                                             + joint2_range / table_length * self.x_disp[i],
                                             joint3_start
                                             + joint3_range / table_length * self.x_disp[i],
                                             joint4_start
                                             + joint4_range / table_length * self.x_disp[i],
                                             joint5_start
                                             + joint5_range / table_length * self.x_disp[i],
                                             joint6_start
                                             + joint6_range / table_length * self.x_disp[i]
                                             + stab_adjust_range / table_length
                                             * self.x_disp[i],
                                             joint7_start
                                             + joint7_range / table_length * self.x_disp[i],
                                             0.0,
                                             0.0
                                             ]

                    self.waypoint_joints4 = [joint1_start + self.rotate[i],     # panda_joint1
                                             joint2_start,         # panda_joint2
                                             joint3_start,         # panda_joint3
                                             joint4_start,         # panda_joint4
                                             joint5_start,         # panda_joint5
                                             joint6_start,         # panda_joint6
                                             joint7_start,         # panda_joint7
                                             0.0,                  # panda_finger_jointl
                                             0.0                   # panda_finger_jointr
                                             ]

                    joint_movements = [self.waypoint_joints1,
                                       self.waypoint_joints2,
                                       self.waypoint_joints3,
                                       self.waypoint_joints4]
                    self.num_movements = len(joint_movements)
                    joint_waypoints.append(joint_movements)

                self.moveit.joint_waypoints(
                    joint_waypoints[self.num_waypoints_completed][self.num_moves_completed])
                self.num_moves_completed += 1
                if self.num_moves_completed % 4 == 0:
                    self.num_waypoints_completed += 1

        elif self.state == State.WAYPOINTS:
            if self.moveit.planning:
                self.state = State.WAYPOINTS_WAIT
            else:
                # add wait for collision object to be in planning scene before plan! (new state?)
                if self.is_waypoint:
                    self.moveit.plan_traj_to_pose(self.goal_waypoint)
                else:
                    self.moveit.joint_waypoints(self.waypoint_joints)
                self.waypoints += 1

        elif self.state == State.WAYPOINTS_WAIT:
            # once we're not planning anymore, get the plan and move on to execute stage
            if not self.moveit.busy:
                # This is optional. We don't have to get the plan but we can if we
                # want to store it for later for whatever reason.
                # then we can pass it back into the trajectory function (which
                # will be demonstrated here)
                # we could also just call the execution method, which would
                # execute the last planned trajectory
                self.plan = self.moveit.get_plan()
                # if self.moveit.get_last_error() == MoveItApiErrors.CONTROL_ERROR:
                #     if self.sign == 1:
                #         self.state = State.LEFT_DYNAMIC_MOTION
                #     elif self.sign == -1 and not self.is_stab_motion:
                #         self.state = State.RIGHT_DYNAMIC_MOTION
                #     else:
                #         self.state = State.STAB_MOTION
                if self.moveit.get_last_error() == MoveItApiErrors.NO_ERROR:
                    self.state = State.EXECUTE_START
                    self.get_logger().info("start execute!")
                elif self.movement_direction_sign == 1 and self.num_moves_completed == 0:
                    self.movement_direction_sign = -1
                    self.num_moves_completed = 0
                    self.num_waypoints_completed = 0
                    self.state = State.RIGHT_DYNAMIC_MOTION
                    self.get_logger().info("other dynamic motion!")
                elif self.movement_direction_sign == 1:
                    self.movement_direction_sign = -1
                    self.num_moves_completed = 0
                    self.num_waypoints_completed = 0
                    self.state = State.MOVE_TO_HOME_START
                    # self.state = State.RIGHT_DYNAMIC_MOTION
                    self.get_logger().info("back to home!")
                elif not self.is_stab_motion and self.num_moves_completed == 0:
                    self.is_stab_motion = True
                    self.num_moves_completed = 0
                    self.num_waypoints_completed = 0
                    self.state = State.STAB_MOTION
                    self.get_logger().info("stab motion!")
                elif not self.is_stab_motion:
                    self.is_stab_motion = True
                    self.num_moves_completed = 0
                    self.num_waypoints_completed = 0
                    self.state = State.MOVE_TO_HOME_START
                    self.get_logger().info("back to home!")
                else:
                    self.get_logger().info("no solution!")
                    # Enemy is unreachable
                    self.unreachable_enemies.append(self.current_enemy)
                    self.state = State.MOVE_TO_HOME_START

        elif self.state == State.CHECK_FOR_ENEMY_REMAINING:
            # Update objects, wait for all transforms to be found
            if self.update_detected_objects(ObjectType.ENEMY):
                enemies_left = len(self.detected_enemies) > len(self.unreachable_enemies)
                if enemies_left:
                    self.get_logger().info("Attacking next enemy")
                    self.state = State.RESET_ALLIES
                else:
                    self.get_logger().info("All enemies vanquished")
                    self.looking_for_enemies = 0
                    self.state = State.IDLE

        elif self.state == State.NEXT_WAYPOINT:
            # if self.home_waypoint == True:
            #     self.state = State.IDLE
            #     self.home_waypoint = False
            # elif self.ind < self.i:
            #     self.ind += 1
            #     self.waypoint_poses()
            #     self.state = State.WAYPOINTS
            # else:
            #     self.state = State.IDLE
            if self.is_waypoint:
                self.goal_waypoint = self.knock_enemy_waypoint
            else:
                if self.waypoints == 1:
                    self.waypoint_joints = self.waypoint_joints2
                else:
                    self.waypoint_joints = self.waypoint_joints3
            self.get_logger().info("next waypoint!")
            self.state = State.WAYPOINTS

        # Draw waypoints
        elif self.state == State.PICKUP_LIGHTSABER:

            # reset subsequence on first callback of PICKUP_LIGHTSABER state
            if new_state:
                self.pickup_lightsaber_state = PickupLightsaberState.ADD_WALLS_START

            # Execute subsequence to pickup the lightsaber
            # this method returns true if the subsequence is complete, false if not
            if self.pickup_lightsaber_sequence():
                self.config.max_velocity_scaling_factor = self.default_speed

                # Return to IDLE
                self.state = State.IDLE

        elif self.state == State.PLAN_TO_POSE_START:

            # if self.waypoints == 1:
            #     self.waypoint_poses()
            # If we've started planning move to next step
            if self.moveit.planning:
                self.state = State.PLAN_WAIT

            # If we haven't started planning, initiate plan
            # TODO - would probably want to error handle here or we'll get stuck
            else:
                self.moveit.plan_traj_to_pose(self.goal_pose)

        elif self.state == State.PLAN_TO_POSITION_START:

            # If we've started planning move to next step
            if self.moveit.planning:
                self.state = State.PLAN_WAIT

            # If we haven't started planning, initiate plan
            # TODO - would probably want to error handle here or we'll get stuck
            else:
                self.moveit.plan_traj_to_position(self.goal_pose.position)

        elif self.state == State.PLAN_TO_ORIENTATION_START:

            # If we've started planning move to next step
            if self.moveit.planning:
                self.state = State.PLAN_WAIT

            # If we haven't started planning, initiate plan
            # TODO - would probably want to error handle here or we'll get stuck
            else:
                self.moveit.plan_traj_to_orientation(self.goal_pose.orientation)

        elif self.state == State.PLAN_WAIT:

            # once we're not planning anymore, get the plan and move on to execute stage
            if not self.moveit.busy:
                # This is optional. We don't have to get the plan but we can if we
                # want to store it for later for whatever reason.
                # then we can pass it back into the trajectory function (which
                # will be demonstrated here)
                # we could also just call the execution method, which would
                # execute the last planned trajectory
                self.plan = self.moveit.get_plan()

                if self.moveit.get_last_error() == MoveItApiErrors.NO_ERROR:
                    self.state = State.EXECUTE_START
                    self.get_logger().info("start execute!")
                else:
                    self.state = State.IDLE

        elif self.state == State.EXECUTE_START:

            # If we've started executing move to next step
            if self.moveit.executing:
                self.state = State.EXECUTE_WAIT

            # If we haven't started executing, initiate execute
            # TODO - would probably want to error handle here or we'll get stuck
            else:
                # again, if we're executing the last planned trajectory, we don't
                # Need to pass a plan in. This is just an example of doing that
                # (if you wanted to pass an arbitrary plan)
                self.moveit.exec_traj(self.plan)

        elif self.state == State.EXECUTE_WAIT:

            # once we're not executing anymore, return to IDLE
            if not self.moveit.busy:
                self.get_logger().info(f'num movements: {self.num_movements}')
                self.get_logger().info(f'num moves completed: {self.num_moves_completed}')
                if self.num_moves_completed < self.num_movements:
                    if self.moveit.get_last_error() == \
                            MoveItApiErrors.NO_ERROR and self.movement_direction_sign == 1:
                        self.get_logger().info("next waypoint!")
                        self.state = State.LEFT_DYNAMIC_MOTION
                    elif not self.is_stab_motion:
                        self.state = State.RIGHT_DYNAMIC_MOTION
                    else:
                        self.state = State.STAB_MOTION
                else:
                    self.get_logger().info("done!")
                    all_transforms_found = self.update_detected_objects(ObjectType.ENEMY)
                    self.is_stab_motion = False
                    self.movement_direction_sign = 1
                    if all_transforms_found:
                        self.enemies_after = len(self.detected_enemies)
                        self.state = State.MOVE_TO_HOME_START
                        self.dead_enemy_count += self.enemies_before - self.enemies_after

        self.dead_count_pub.publish(Int16(data=self.dead_enemy_count))

    def pickup_lightsaber_sequence(self):
        """
        Sequence to pickup the lightsaber.

        Adds environment collision obstacles, handles attached collision objects vs
        regular collision objects, and follows waypoints to bring it to the correct spot
        to grab the lightsaber.
        """
        done = False

        new_state = self.pickup_lightsaber_state != self.pickup_lightsaber_state_last

        if new_state:
            self.get_logger().info(
                f"Pickup lightsaber sequence changed to {self.pickup_lightsaber_state.name}")
            self.pickup_lightsaber_state_last = self.pickup_lightsaber_state

        if self.pickup_lightsaber_state == PickupLightsaberState.ADD_WALLS_START:
            if self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = PickupLightsaberState.ADD_WALLS_WAIT
            else:
                self.add_walls()

        elif self.pickup_lightsaber_state == PickupLightsaberState.ADD_WALLS_WAIT:
            if not self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = \
                    PickupLightsaberState.REMOVE_ATTACHED_COLLISION_START

        elif self.pickup_lightsaber_state == PickupLightsaberState.REMOVE_ATTACHED_COLLISION_START:
            if self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = PickupLightsaberState.REMOVE_ATTACHED_COLLISION_WAIT
            else:
                self.remove_attached_lightsaber()

        elif self.pickup_lightsaber_state == PickupLightsaberState.REMOVE_ATTACHED_COLLISION_WAIT:
            if not self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = PickupLightsaberState.ADD_SEPARATE_COLLISION_START

        elif self.pickup_lightsaber_state == PickupLightsaberState.ADD_SEPARATE_COLLISION_START:

            self.config.max_velocity_scaling_factor = self.pickup_lightsaber_speed_fast

            if self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = PickupLightsaberState.ADD_SEPARATE_COLLISION_WAIT
            else:
                self.add_separate_lightsaber()

        if self.pickup_lightsaber_state == PickupLightsaberState.ADD_SEPARATE_COLLISION_WAIT:
            if not self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = PickupLightsaberState.MOVE_TO_HOME_START

        elif self.pickup_lightsaber_state == PickupLightsaberState.MOVE_TO_HOME_START:
            if self.moveit.planning:
                self.pickup_lightsaber_state = PickupLightsaberState.MOVE_TO_HOME_WAIT
            else:
                self.moveit.move_to_home()

        elif self.pickup_lightsaber_state == PickupLightsaberState.MOVE_TO_HOME_WAIT:
            if not self.moveit.busy:

                # Skip gripper actions in simulation since action server is not available
                if self.simulation:
                    self.pickup_lightsaber_state = \
                        PickupLightsaberState.MOVE_TO_LIGHTSABER_STANDOFF_START
                else:
                    self.pickup_lightsaber_state = PickupLightsaberState.OPEN_START
                    self.open_count = 0

        elif self.pickup_lightsaber_state == PickupLightsaberState.OPEN_START:
            if new_state:
                self.pickup_lightsaber_future = None

            if self.pickup_lightsaber_future is None:
                self.pickup_lightsaber_future = self.open_gripper()

            elif self.pickup_lightsaber_future is not None:
                if self.pickup_lightsaber_future.done():
                    self.pickup_lightsaber_future = \
                        self.pickup_lightsaber_future.result().get_result_async()
                    self.pickup_lightsaber_state = PickupLightsaberState.OPEN_WAIT

        elif self.pickup_lightsaber_state == PickupLightsaberState.OPEN_WAIT:
            if self.pickup_lightsaber_future.done():
                self.open_count += 1

                if self.open_count >= 2:
                    self.pickup_lightsaber_state = \
                        PickupLightsaberState.MOVE_TO_LIGHTSABER_STANDOFF_START
                else:
                    self.pickup_lightsaber_state = PickupLightsaberState.OPEN_START

        elif self.pickup_lightsaber_state == \
                PickupLightsaberState.MOVE_TO_LIGHTSABER_STANDOFF_START:
            if self.moveit.planning:
                self.pickup_lightsaber_state = \
                    PickupLightsaberState.MOVE_TO_LIGHTSABER_STANDOFF_WAIT
            else:
                pose = geometry_msgs.msg.Pose()
                pose.position.x = self.lightsaber_start_location.x
                pose.position.y = self.lightsaber_start_location.y + \
                    self.gripper_height * 2.
                pose.position.z = self.lightsaber_start_location.z + \
                    self.lightsaber_full_length / 2. - \
                    self.lightsaber_grip_offset
                pose.orientation.x = 0.5
                pose.orientation.y = 0.5
                pose.orientation.z = -0.5
                pose.orientation.w = 0.5
                self.moveit.plan_traj_to_pose(pose, execute=True)

        elif self.pickup_lightsaber_state == \
                PickupLightsaberState.MOVE_TO_LIGHTSABER_STANDOFF_WAIT:
            # wait for collision object to be in scene
            if not self.moveit.busy:
                self.pickup_lightsaber_state = \
                    PickupLightsaberState.REMOVE_SEPARATE_COLLISION_START

        elif self.pickup_lightsaber_state == PickupLightsaberState.REMOVE_SEPARATE_COLLISION_START:
            if self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = PickupLightsaberState.REMOVE_SEPARATE_COLLISION_WAIT
            else:
                self.remove_separate_lightsaber()

        elif self.pickup_lightsaber_state == PickupLightsaberState.REMOVE_SEPARATE_COLLISION_WAIT:
            if not self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = PickupLightsaberState.MOVE_TO_LIGHTSABER_PICK_START

        elif self.pickup_lightsaber_state == PickupLightsaberState.MOVE_TO_LIGHTSABER_PICK_START:
            self.config.max_velocity_scaling_factor = self.pickup_lightsaber_speed_slow

            if self.moveit.planning:
                self.pickup_lightsaber_state = PickupLightsaberState.MOVE_TO_LIGHTSABER_PICK_WAIT
            else:
                pose = geometry_msgs.msg.Pose()
                pose.position.x = self.lightsaber_start_location.x
                pose.position.y = self.lightsaber_start_location.y + \
                    self.gripper_tcp_offset
                pose.position.z = self.lightsaber_start_location.z + \
                    self.lightsaber_full_length / 2. - \
                    self.lightsaber_grip_offset
                pose.orientation.x = 0.5
                pose.orientation.y = 0.5
                pose.orientation.z = -0.5
                pose.orientation.w = 0.5
                self.moveit.plan_traj_to_pose(pose, execute=True)

        elif self.pickup_lightsaber_state == PickupLightsaberState.MOVE_TO_LIGHTSABER_PICK_WAIT:
            if not self.moveit.busy:

                # Skip gripper actions in simulation since action server is not available
                if self.simulation:
                    self.pickup_lightsaber_state = \
                        PickupLightsaberState.ADD_ATTACHED_COLLISION_START
                else:
                    self.pickup_lightsaber_state = PickupLightsaberState.GRASP_START

        elif self.pickup_lightsaber_state == PickupLightsaberState.GRASP_START:
            if new_state:
                self.pickup_lightsaber_future = None

            if self.pickup_lightsaber_future is None:
                self.pickup_lightsaber_future = self.grasp()

            elif self.pickup_lightsaber_future is not None:
                if self.pickup_lightsaber_future.done():
                    self.pickup_lightsaber_future = \
                        self.pickup_lightsaber_future.result().get_result_async()
                    self.pickup_lightsaber_state = PickupLightsaberState.GRASP_WAIT

        elif self.pickup_lightsaber_state == PickupLightsaberState.GRASP_WAIT:
            if self.pickup_lightsaber_future.done():
                self.pickup_lightsaber_state = PickupLightsaberState.ADD_ATTACHED_COLLISION_START

        elif self.pickup_lightsaber_state == PickupLightsaberState.ADD_ATTACHED_COLLISION_START:
            if self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = PickupLightsaberState.ADD_ATTACHED_COLLISION_WAIT
            else:
                self.add_attached_lightsaber()

        elif self.pickup_lightsaber_state == PickupLightsaberState.ADD_ATTACHED_COLLISION_WAIT:
            if not self.moveit.busy_updating_obstacles:
                self.pickup_lightsaber_state = PickupLightsaberState.DRAW_START

        elif self.pickup_lightsaber_state == PickupLightsaberState.DRAW_START:
            if self.moveit.planning:
                self.pickup_lightsaber_state = PickupLightsaberState.DRAW_WAIT
            else:
                pose = geometry_msgs.msg.Pose()
                pose.position.x = self.lightsaber_start_location.x
                pose.position.y = self.lightsaber_start_location.y + \
                    self.gripper_tcp_offset
                pose.position.z = self.lightsaber_start_location.z + \
                    self.lightsaber_full_length / 2. - \
                    self.lightsaber_grip_offset + \
                    (self.lightsaber_lift_height * 1.2)
                pose.orientation.x = 0.5
                pose.orientation.y = 0.5
                pose.orientation.z = -0.5
                pose.orientation.w = 0.5
                self.moveit.plan_traj_to_pose(pose, execute=True)

        elif self.pickup_lightsaber_state == PickupLightsaberState.DRAW_WAIT:
            if not self.moveit.busy:
                self.pickup_lightsaber_state = PickupLightsaberState.RETURN_TO_HOME_START

        elif self.pickup_lightsaber_state == PickupLightsaberState.RETURN_TO_HOME_START:
            self.config.max_velocity_scaling_factor = self.pickup_lightsaber_speed_fast

            if self.moveit.planning:
                self.pickup_lightsaber_state = PickupLightsaberState.RETURN_TO_HOME_WAIT
            else:
                self.moveit.move_to_home()

        elif self.pickup_lightsaber_state == PickupLightsaberState.RETURN_TO_HOME_WAIT:
            if not self.moveit.busy:
                done = True

        return done

    def pickup_lightsaber_callback(self, request, response):
        """Begin lightsaber subsequence if currently in IDLE."""
        if self.state == State.IDLE:
            self.state = State.PICKUP_LIGHTSABER

        return response

    def move_to_home_callback(self, request, response):
        """Move to home position."""
        # no longer necessary since we're using the API home function
        self.goal_pose = copy.deepcopy(self.home_pose)

        self.looking_for_enemies = 0
        self.state = State.MOVE_TO_HOME_START

        return response

    def look_for_enemy_callback(self, request, response):
        """Begin looking for obstacles and attacking enemies."""
        if self.state != State.IDLE:
            return

        self.state = State.FIND_ALLIES
        self.num_waypoints_completed = 0
        self.num_waypoints_completed = 0
        return response

    def gripper_open_callback(self, request, response):
        """Call function to open the grippers."""
        self.open_gripper()

        return response

    def gripper_close_callback(self, request, response):
        """Call function to cause the grippers to close."""
        self.close_gripper()

        return response

    def gripper_grasp_callback(self, request, response):
        """Call function to cause the gripper to close around object."""
        self.grasp()

        return response

    def joint_positions(self, point):
        """Publish the robot's current joint trajectory in a JointTrajectory message."""
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.header.frame_id = ''
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['panda_joint1',
                            'panda_joint2',
                            'panda_joint3',
                            'panda_joint4',
                            'panda_joint5',
                            'panda_joint6',
                            'panda_joint7']

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = self.position[point]
        point.time_from_start.sec = 1

        traj.points = [point]

        self.trajectory = control_msgs.action.FollowJointTrajectory.Goal()
        self.trajectory = traj

        self.ready = 1

        self.joint_traj = traj
        self.pub_joint_traj.publish(self.joint_traj)

    def waypoint_callback(self, request, response):
        """Call function to add waypoint to plan to."""
        self.waypoints = 1
        self.waypoint_poses()
        self.state = State.WAYPOINTS
        self.is_waypoint = True
        return response

    def move_to_pose_callback(self, request, response):
        """
        Call /move_to_pose (moveit_testing_interfaces/srv/MoveToPose) service.

        Pose value given by user is stored
        State is updated to PLAN_TO_POSE_START

        Example call:
            ros2 service call /move_to_pose moveit_testing_interfaces/srv/MoveToPose
            "pose: {position: {x: 0.5, y: 0.5, z: 0.5},
            orientation: {x: 0., y: 0., z: 0., w: 1.}}"

        Args:
            request (MoveToPose): position and orientation

            response (EmptyResponse): no data

        Returns
        -------
            response (EmptyResponse): no data

        """
        self.goal_pose = request.pose

        self.state = State.PLAN_TO_POSE_START

        return response

    def move_to_position_callback(self, request, response):
        """
        Call /move_to_position (moveit_testing_interfaces/srv/MoveToPosition) service.

        Position given by user is stored
        State is updated to PLAN_TO_POSITION_START

        Example call:
        ros2 service call /move_to_position moveit_testing_interfaces/srv/MoveToPosition
            "position: {x: 0.5, y: 0.5, z: 0.5}"

        Args:
            request (MoveToPosition): position

            response (EmptyResponse): no data

        Returns
        -------
            response (EmptyResponse): no data

        """
        return response

    def home_waypoint_callback(self, request, response):
        """Add a home waypoint to move to home."""
        home_point = geometry_msgs.msg.Point()
        home_orientation = geometry_msgs.msg.Pose().orientation
        home_point.x = 0.306891
        home_point.y = -8.32667e-17
        home_point.z = 0.486882
        home_orientation.x = 1.0
        home_orientation.y = 1.38778e-16
        home_orientation.z = 2.22045e-16
        home_orientation.w = -6.93889e-17

        self.goal_home_waypoint = geometry_msgs.msg.Pose()
        self.goal_home_waypoint.position = home_point
        self.goal_home_waypoint.orientation = home_orientation

        self.home_waypoint = True

        self.state = State.WAYPOINTS

        return response

    def waypoint_poses(self):
        """Add waypoint for robot motion."""
        self.state = State.WAYPOINTS

        self.waypoints = 1
        point1 = geometry_msgs.msg.Point()
        point1.x = 0.35
        point1.y = 0.0
        point1.z = 0.6
        orientation1 = geometry_msgs.msg.Pose().orientation
        orientation1.x = math.pi

        points = [point1]
        orientations = [orientation1]

        self.goal_waypoint.position = points[0]
        self.goal_waypoint.orientation = orientations[0]

        self.state = State.WAYPOINTS

    def joint_waypoint_callback(self, request, response):
        """Initiate waypoint sequence through joint commands."""
        self.state = State.WAYPOINTS
        return response

    def move_to_orientation_callback(self, request, response):
        """
        Call /move_to_orientation (moveit_testing_interfaces/srv/MoveToOrientation) service.

        Orientation given by user is stored
        State is updated to PLAN_TO_ORIENTATION_START

        Example call:
        ros2 service call /move_to_orientation moveit_testing_interfaces/srv/MoveToOrientation
            "orientation: {x: 0., y: 0., z: 0., w: 1.}"

        Args:
            request (MoveToOrientation): orientation

            response (EmptyResponse): no data

        Returns
        -------
            response (EmptyResponse): no data

        """
        self.goal_pose = geometry_msgs.msg.Pose()
        self.goal_pose.orientation = request.orientation

        self.state = State.PLAN_TO_ORIENTATION_START

        return response

    def obstacles_callback(self, request, response):
        """
        Call /update_obstacles (moveit_testing_interfaces/srv/UpdateObstacles) service.

        Store obstacle position, dimensions, id and delete flag value input by the user

        Example calls:
        ros2 service call /update_obstacles moveit_testing_interfaces/srv/UpdateObstacles
        "{position: {x: 0.5, y: 0.5, z: 1.0},
        length: 0.5, width: 0.25, height: 2.0, id: 'MyBox', delete_obstacle: false}"

        ros2 service call /update_obstacles moveit_testing_interfaces/srv/UpdateObstacles
        "{position: {x: 0.5, y: 0.0, z: 0.0},
        length: 1.125, width: 0.033, height: 0.2, id: 'gripping', delete_obstacle: false}"

        Args:
            request (UpdateObstacles): obstacle information

            response (EmptyResponse): no data

        Returns
        -------
            response (EmptyResponse): no data

        """
        obstacle = moveit_msgs.msg.CollisionObject()
        obstacle.id = request.id

        pose = geometry_msgs.msg.Pose()
        pose.position = request.position
        obstacle.primitive_poses = [pose]

        shape = shape_msgs.msg.SolidPrimitive()
        shape.type = 3  # Cylinder
        shape.dimensions = [request.length, request.width, request.height]
        obstacle.primitives = [shape]

        obstacle.header.frame_id = self.moveit.config.base_frame_id

        self.moveit.update_obstacles([obstacle], delete=request.delete_obstacle)

        return response

    def persistent_obstacles_callback(self, request, response):
        """
        Call /update_persistent_obstacles (moveit_testing_interfaces/srv/UpdateObstacles) service.

        Store obstacle position, dimensions, id and delete flag value input by the user

        Example calls:
        ros2 service call /update_persistent_obstacles
        moveit_testing_interfaces/srv/UpdateObstacles
        "{position: {x: 0.0, y: -0.9, z: 0.0},
        length: 12.0, width: 0.25, height: 4.0, id: 'wall1', delete_obstacle: false}"

        ros2 service call /update_persistent_obstacles
        moveit_testing_interfaces/srv/UpdateObstacles
        "{position: {x: 0.0, y: 0.9, z: 0.0},
        length: 12.0, width: 0.25, height: 4.0, id: 'wall2', delete_obstacle: false}"

        Args:
            request (UpdateObstacles): obstacle information

            response (EmptyResponse): no data

        Returns
        -------
            response (EmptyResponse): no data

        """
        obstacle = moveit_msgs.msg.CollisionObject()
        obstacle.id = request.id

        pose = geometry_msgs.msg.Pose()
        pose.position = request.position
        obstacle.primitive_poses = [pose]

        shape = shape_msgs.msg.SolidPrimitive()
        shape.type = 1  # Box
        shape.dimensions = [request.length, request.width, request.height]
        obstacle.primitives = [shape]

        obstacle.header.frame_id = self.moveit.config.base_frame_id

        self.moveit.update_obstacles([obstacle], delete=request.delete_obstacle)

        return response

    def add_separate_lightsaber(self):
        """Add lightsaber as a separate collision object."""
        obstacle = moveit_msgs.msg.CollisionObject()
        obstacle.id = 'lightsaber'

        pose = geometry_msgs.msg.Pose()
        pose.position = self.lightsaber_start_location
        pose.orientation.z = -1.0

        obstacle.primitive_poses = [pose]

        shape = shape_msgs.msg.SolidPrimitive()
        shape.type = 3  # Cylinder
        shape.dimensions = [self.lightsaber_full_length,
                            self.lightsaber_diameter * 1.25 / 2.0, 0.2]
        obstacle.primitives = [shape]

        obstacle.header.frame_id = self.moveit.config.base_frame_id

        self.moveit.update_obstacles([obstacle], delete=False)

    def remove_separate_lightsaber(self):
        """Remove lightsaber as a separate collision object."""
        obstacle = moveit_msgs.msg.CollisionObject()
        obstacle.id = 'lightsaber'

        self.moveit.update_obstacles([obstacle], delete=True)

    def add_attached_lightsaber(self):
        """Add lightsaber as an attached collision object."""
        attached_obstacle = moveit_msgs.msg.AttachedCollisionObject()
        attached_obstacle.link_name = 'panda_hand_tcp'
        attached_obstacle.object.header.frame_id = 'panda_hand_tcp'
        attached_obstacle.object.header.stamp = self.get_clock().now().to_msg()
        attached_obstacle.object.id = 'lightsaber'

        pose = geometry_msgs.msg.Pose()
        pose.position.x = self.lightsaber_full_length / 2. - self.lightsaber_grip_offset
        pose.position.y = 0.0
        pose.position.z = self.gripper_tcp_offset
        pose.orientation.y = -1.0
        attached_obstacle.object.primitive_poses = [pose]

        shape = shape_msgs.msg.SolidPrimitive()
        shape.type = 3  # Cylinder
        shape.dimensions = [self.lightsaber_full_length*1.03,
                            self.lightsaber_diameter * 1.25 / 2.0, 0.2]
        attached_obstacle.object.primitives = [shape]

        attached_obstacle.object.operation = attached_obstacle.object.ADD

        attached_obstacle.touch_links = ['panda_rightfinger',
                                         'panda_leftfinger',
                                         'panda_hand_tcp',
                                         'panda_hand']

        self.moveit.update_attached_obstacles([attached_obstacle], delete=False)

    def remove_attached_lightsaber(self):
        """Remove lightsaber as an attached collision object."""
        attached_obstacle = moveit_msgs.msg.AttachedCollisionObject()
        attached_obstacle.object.id = 'lightsaber'

        self.moveit.update_attached_obstacles([attached_obstacle], delete=True)

    def add_separate_lightsaber_callback(self, request, response):
        """Call function to add lightsaber as seperate collision object."""
        self.add_separate_lightsaber()

        return response

    def remove_separate_lightsaber_callback(self, request, response):
        """Call function which removes lightsaber as collision object."""
        self.remove_separate_lightsaber()

        return response

    def add_attached_lightsaber_callback(self, request, response):
        """Add lightsaber as attached obstacle."""
        self.add_attached_lightsaber()

        return response

    def remove_attached_lightsaber_callback(self, request, response):
        """Delete lightsaber from attached obstacle list."""
        self.remove_attached_lightsaber()

        return response

    def attached_obstacles_callback(self, request, response):
        """
        Call /update_persistent_obstacles (moveit_testing_interfaces/srv/UpdateAttachedObstacles).

        Store obstacle position, dimensions, ids and delete flag value input by the user

        Example call:
        ros2 service call /update_attached_obstacles
        moveit_testing_interfaces/srv/UpdateAttachedObstacles
        "{link_name: "panda_hand_tcp",
        position: {x: 0.411, y: 0.0, z: 0.0},
        length: 1.125, width: 0.033, height: 0.2, id: 'gripping', type: 3, delete_obstacle: false}"

        Args:
            request (UpdateAttachedObstacles): obstacle information

            response (EmptyResponse): no data

        Returns
        -------
            response (EmptyResponse): no data

        """
        attached_obstacle = moveit_msgs.msg.AttachedCollisionObject()
        attached_obstacle.link_name = request.link_name
        attached_obstacle.object.header.frame_id = request.link_name
        attached_obstacle.object.header.stamp = self.get_clock().now().to_msg()
        attached_obstacle.object.id = request.id

        pose = geometry_msgs.msg.Pose()
        pose.position = request.position
        pose.orientation.y = -1.0
        attached_obstacle.object.primitive_poses = [pose]

        shape = shape_msgs.msg.SolidPrimitive()
        shape.type = request.type  # Box
        shape.dimensions = [request.length, request.width, request.height]
        attached_obstacle.object.primitives = [shape]

        attached_obstacle.object.operation = attached_obstacle.object.ADD

        attached_obstacle.touch_links = ['panda_rightfinger',
                                         'panda_leftfinger',
                                         'panda_hand_tcp',
                                         'panda_hand']

        self.moveit.update_attached_obstacles([attached_obstacle], delete=request.delete_obstacle)

        return response

    def add_walls(self):
        """Add walls and ceiling."""
        obstacle = moveit_msgs.msg.CollisionObject()
        obstacle.id = 'wall_0'

        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.25
        pose.position.y = self.side_wall_distance
        pose.position.z = 0.0
        obstacle.primitive_poses = [pose]

        shape = shape_msgs.msg.SolidPrimitive()
        shape.type = 1  # Box
        shape.dimensions = [3.0, 0.25, self.side_wall_height]
        obstacle.primitives = [shape]

        obstacle.header.frame_id = self.moveit.config.base_frame_id

        obstacle1 = moveit_msgs.msg.CollisionObject()
        obstacle1.id = 'wall_1'

        pose1 = geometry_msgs.msg.Pose()
        pose1.position.x = 0.25
        pose1.position.y = -self.side_wall_distance
        pose1.position.z = 0.0
        obstacle1.primitive_poses = [pose1]

        shape1 = shape_msgs.msg.SolidPrimitive()
        shape1.type = 1  # Box
        shape1.dimensions = [3.0, 0.25, self.side_wall_height]
        obstacle1.primitives = [shape1]

        obstacle1.header.frame_id = self.moveit.config.base_frame_id

        obstacle2 = moveit_msgs.msg.CollisionObject()
        obstacle2.id = 'floor'

        pose2 = geometry_msgs.msg.Pose()
        pose2.position.x = 0.0
        pose2.position.y = 0.0
        pose2.position.z = -(self.robot_table_height)
        obstacle2.primitive_poses = [pose2]

        shape2 = shape_msgs.msg.SolidPrimitive()
        shape2.type = 1  # Box
        shape2.dimensions = [4.0, 2.0, 0.02]
        obstacle2.primitives = [shape2]

        obstacle2.header.frame_id = self.moveit.config.base_frame_id

        obstacle3 = moveit_msgs.msg.CollisionObject()
        obstacle3.id = 'blocks_table'

        table_size = self.robot_table_height - self.table_offset

        pose3 = geometry_msgs.msg.Pose()
        pose3.position.x = self.table_center_x
        pose3.position.y = self.table_center_y
        pose3.position.z = -self.robot_table_height + table_size / 2.
        obstacle3.primitive_poses = [pose3]

        shape3 = shape_msgs.msg.SolidPrimitive()
        shape3.type = 1  # Box
        shape3.dimensions = [self.table_len_x, self.table_len_y, table_size]
        obstacle3.primitives = [shape3]

        obstacle3.header.frame_id = self.moveit.config.base_frame_id

        obstacle4 = moveit_msgs.msg.CollisionObject()
        obstacle4.id = 'back_wall'

        pose4 = geometry_msgs.msg.Pose()
        pose4.position.x = -self.back_wall_distance
        pose4.position.y = 0.0
        pose4.position.z = -self.robot_table_height + (self.back_wall_height/2)
        obstacle4.primitive_poses = [pose4]

        shape4 = shape_msgs.msg.SolidPrimitive()
        shape4.type = 1  # Box
        shape4.dimensions = [0.3, 2.0, self.back_wall_height]
        obstacle4.primitives = [shape4]

        obstacle4.header.frame_id = self.moveit.config.base_frame_id

        obstacle5 = moveit_msgs.msg.CollisionObject()
        obstacle5.id = 'ceiling'

        pose5 = geometry_msgs.msg.Pose()
        pose5.position.x = 0.0
        pose5.position.y = 0.0
        pose5.position.z = self.ceiling_height - self.robot_table_height
        obstacle5.primitive_poses = [pose5]

        shape5 = shape_msgs.msg.SolidPrimitive()
        shape5.type = 1  # Box
        obstacle.id = 'wall_0'

        shape5.dimensions = [4.0, 2.0, 0.02]
        obstacle5.primitives = [shape5]

        obstacle5.header.frame_id = self.moveit.config.base_frame_id

        obstacle6 = moveit_msgs.msg.CollisionObject()
        obstacle6.id = 'gripper_height_offset'

        pose6 = geometry_msgs.msg.Pose()
        pose6.position.x = 0.355
        pose6.position.y = 0.0
        pose6.position.z = self.gripper_height/2
        obstacle6.primitive_poses = [pose6]

        shape6 = shape_msgs.msg.SolidPrimitive()
        shape6.type = 1  # Box
        shape6.dimensions = [0.35, self.robot_table_width, self.gripper_height]
        obstacle6.primitives = [shape6]

        obstacle6.header.frame_id = self.moveit.config.base_frame_id

        self.moveit.update_obstacles([obstacle,
                                     obstacle1,
                                     obstacle2,
                                     obstacle3,
                                     obstacle4,
                                     obstacle5,
                                     obstacle6],
                                     delete=False)

        # arm table should be attached collision object
        attached_obstacle = moveit_msgs.msg.AttachedCollisionObject()
        attached_obstacle.link_name = 'panda_link0'
        attached_obstacle.object.header.frame_id = 'panda_link0'
        attached_obstacle.object.header.stamp = self.get_clock().now().to_msg()
        attached_obstacle.object.id = 'arm_table'

        pose2 = geometry_msgs.msg.Pose()
        pose2.position.x = 0.0
        pose2.position.y = 0.0
        pose2.position.z = -(self.robot_table_height/2)
        attached_obstacle.object.primitive_poses = [pose2]

        shape2 = shape_msgs.msg.SolidPrimitive()
        shape2.type = 1  # Box
        shape2.dimensions = [self.robot_table_length, self.robot_table_width,
                             self.robot_table_height]
        attached_obstacle.object.primitives = [shape2]

        attached_obstacle.object.operation = attached_obstacle.object.ADD

        attached_obstacle.touch_links = ['panda_link0', 'panda_link1']

        self.moveit.update_attached_obstacles([attached_obstacle], delete=False)

    def reset_allies_callback(self, request, response):
        """Update ally position."""
        self.allies_reset_flag = True
        return response

    def add_walls_callback(self, request, response):
        """Add walls and ceilings to planning scene as collision objects."""
        self.add_walls()

        return response

    def obj_detection_callback(self, data):
        """
        Subscription that updates objects detected from camera.

        Args:
            data:  detected objects in a list to be saved

        Returns
        -------
            none

        """
        self.detected_objects = data

    def update_detected_objects(self, object_type):
        """
        Get transforms seen to update all items of the selected object_type.

        Args:
            object_type: an ObjectType variable representing whether allies or enemies are
                            being updated

        Returns
        -------
            all_transforms_found: a boolean representing if any TransformExceptions were found
                                    while pasing transforms

        """
        if self.detected_objects is None:
            return False

        all_transforms_found = True

        if object_type == ObjectType.ALLY:

            self.detected_allies = []

            # Populate array with instances of class that stores the object information and
            # It's transform (if it can be found)
            for ally in self.detected_objects.allies:
                # Init object data
                obj_data = DetectedObjectData(ally)

                # Try to get the transform for the detected object
                try:
                    obj_data.tf = self.tf_buffer.lookup_transform(FRAMES.PANDA_BASE,
                                                                  obj_data.obj.name,
                                                                  rclpy.time.Time())

                except TransformException:
                    all_transforms_found = False

                # Append object data to array
                self.detected_allies.append(obj_data)

        elif object_type == ObjectType.ENEMY:

            self.detected_enemies = []

            # Populate array with instances of class that stores the object information and
            # It's transform (if it can be found)
            for enemy in self.detected_objects.enemies:
                # Init object data
                obj_data = DetectedObjectData(enemy)

                # Try to get the transform for the detected object
                try:
                    obj_data.tf = self.tf_buffer.lookup_transform(FRAMES.PANDA_BASE,
                                                                  obj_data.obj.name,
                                                                  rclpy.time.Time())

                except TransformException:
                    all_transforms_found = False

                # Append object data to array
                self.detected_enemies.append(obj_data)

        # Indicate if all transforms were found
        return all_transforms_found

    def check_ally_danger_fall(self, enemy_obj, swing_style):
        """
        Check if ally blocks are at risk of being knocked over by falling enemy block.

        Args:
            enemy_obj:  specifies the enemy whose fall trajectory is being checked
            swing_style:   specifies which attack syle to check danger for
                                0 - left attack (brick falls towards the right from desk view)
                                1 - right attack (brick falls towards the right from desk view)
                                2 - stabbing attack (brick falls towards command desk)

        Returns
        -------
            bool:   returns false if there is an ally in the 'danger zone' of the fall and
                        true if it is safe to make the move

        """
        # y is left to right
        # x is forward to backward
        enemy_to_ally = DetectedObjectData(enemy_obj)
        for ally in self.detected_allies:
            self.get_logger().info(f'detecting: {len(self.detected_allies)} allies')
            enemy_to_ally = self.tf_buffer.lookup_transform(
                                                            ally.obj.name,
                                                            enemy_obj.obj.name,
                                                            rclpy.time.Time())
            dist_y = enemy_to_ally.transform.translation.y
            dist_x = enemy_to_ally.transform.translation.x
            if swing_style == 0:
                if ((abs(dist_y) < self.block_width*1.25)
                        and (abs(dist_x) < (self.block_height+self.block_width*0.5))):
                    if (dist_x >= 0):
                        self.get_logger().info('not safe to attack in left swing')
                        return False
                # swing_style = 1
            if swing_style == 1:
                # swinging so the brick falls to the right
                if ((abs(dist_y) < self.block_width*1.25)
                        and (abs(dist_x) > -(self.block_height+self.block_width*0.5))):
                    if (dist_x <= 0):
                        self.get_logger().info('not safe to attack in right swing')
                        return False
                # swing_style = 2
            if swing_style == 2:
                if (((abs(dist_x)) < self.block_width)
                        and ((dist_y+self.block_width) < (self.block_height+self.block_width*0.5))
                        and (dist_y <= 0)):
                    self.get_logger().info('not safe to attack in stabbing style')
                    return False
            self.get_logger().info(f'safe to attack in style {swing_style}')
        return True


def entry(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()
    rclpy.spin(robot_control)
    rclpy.shutdown()
