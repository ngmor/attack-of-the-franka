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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from .moveit_interface import MoveIt, MoveConfig, MoveItApiErrors
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
import std_msgs.msg
import franka_msgs.action
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from attack_of_the_franka.common import FRAMES, angle_axis_to_quaternion, ObjectType
from rcl_interfaces.msg import ParameterDescriptor
from attack_of_the_franka_interfaces.msg import Detections, DetectedObject
from std_msgs.msg import Int16


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
    SETUP = auto(),
    FIND_ALLIES = auto(),
    DYNAMIC_MOTION = auto(),
    STAB_MOTION = auto(),
    ENEMIES_KILLED_COUNT = auto()

class DetectedObjectData():

    def __init__(self, obj):
        self.obj = obj
        self.tf = None

class MoveGroup(Node):
    """
    Control robot and planning scene using the moveit_interface API.

    Services:
        move_to_pose (moveit_testing_interfaces/srv/MoveToPose): move robot to specific position
                                                                 and orientation
        move_to_position (moveit_testing_interfaces/srv/MoveToPosition): move robot to specific
                                                                         position
        move_to_orientation (moveit_testing_interfaces/srv/MoveToOrientation): move robot to
                                                                               specific orientation
        update_obstacles (moveit_testing_interfaces/srv/UpdateObstacles): add obstacles to scene
    Timers:
        timer: main timer that runs at 100 Hz
    """

    def __init__(self):
        """Class constructor."""
        super().__init__('MoveGroup')

        self.interval = 1.0 / 100.0
        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.pub_joint_traj = self.create_publisher(trajectory_msgs.msg.JointTrajectory,
                                                    'panda_arm_controller/joint_trajectory', 10)
        self.sub_obj_detections = self.create_subscription(Detections,'object_detections',self.obj_detection_callback,10)
        self.srv_move_to_home = self.create_service(std_srvs.srv.Empty,
                                                    'move_to_home', self.move_to_home_callback)
        self.grip_open_close = self.create_service(std_srvs.srv.Empty,
                                                    'grip_open_close',
                                                    self.grip_open_close_callback)
        self.gripper_grasp = self.create_service(std_srvs.srv.Empty,
                                                    'gripper_grasp', self.gripper_grasp_callback)
        self.waypoints = self.create_service(std_srvs.srv.Empty,
                                                    'waypoints', self.waypoint_callback)
        self.test_jointtrajectory = self.create_service(std_srvs.srv.Empty,
                                                        'test_joint_trajectory',
                                                        self.test_jointtrajectory_callback)
        # self.grip_lightsaber = self.create_service(std_srvs.srv.Empty,
        #                                            'grip_lightsaber',
        #                                            self.grip_lightsaber_callback)
        self.srv_move_to_pose = self.create_service(moveit_testing_interfaces.srv.MoveToPose,
                                                    'move_to_pose', self.move_to_pose_callback)
        self.test_waypoint_lightsaber = self.create_service(std_srvs.srv.Empty, 'joint_waypoint',
                                                            self.test_waypoint_lightsaber_callback)
        self.srv_move_to_position = self.create_service(
            moveit_testing_interfaces.srv.MoveToPosition,
            'move_to_position', self.move_to_position_callback)
        self.srv_move_to_orientation = self.create_service(
            moveit_testing_interfaces.srv.MoveToOrientation,
            'move_to_orientation', self.move_to_orientation_callback)
        self.srv_update_obstacles = self.create_service(
            moveit_testing_interfaces.srv.UpdateObstacles,
            'update_obstacles', self.obstacles_callback)
        self.srv_grippers = self.create_service(
            moveit_msgs.srv.GraspPlanning,
            'grasp_plan', self.gripper_callback)
        self.pickup_action_client = ActionClient(self, control_msgs.action.GripperCommand,
                                         'panda_gripper/gripper_action')
        
        self.test_followjoints = ActionClient(self, control_msgs.action.FollowJointTrajectory,
                                        'panda_gripper/follow_joint_trajectory')
        # self.home_gripper_client = ActionClient(self, franka_msgs.action.Homing,
        #                                  'panda_gripper/homing')
        self.grip_lightsaber_client = ActionClient(self, franka_msgs.action.Grasp, 
                                        'panda_gripper/grasp')
        self.srv_update_persistent_obstacles = self.create_service(
            moveit_testing_interfaces.srv.UpdateObstacles,
            'update_persistent_obstacles', self.persistent_obstacles_callback)
        self.srv_update_attached_obstacles = self.create_service(
            moveit_testing_interfaces.srv.UpdateAttachedObstacles,
            'update_attached_obstacles', self.attached_obstacles_callback)
        self.home_waypoint_srv = self.create_service(std_srvs.srv.Empty, 'home_waypoint',
                                                     self.home_waypoint_callback)
        self.srv_add_walls = self.create_service(std_srvs.srv.Empty,
                                                'add_walls', self.add_walls_callback)
        self.look_for_enemy_srv = self.create_service(std_srvs.srv.Empty,
                                                    'look_for_enemy',
                                                    self.look_for_enemy_callback)

         # create a listener for brick position
        self.tf_buffer = Buffer()
        self.tf_obj_listener = TransformListener(self.tf_buffer, self)

         # Dimension parameters
        self.declare_parameter("robot_table.width", 0.605,
                               ParameterDescriptor(description="Robot table width"))
        self.robot_table_width = self.get_parameter("robot_table.width").get_parameter_value().double_value
        self.declare_parameter("robot_table.length", 0.911,
                               ParameterDescriptor(description="Robot table length"))
        self.robot_table_length = self.get_parameter("robot_table.length").get_parameter_value().double_value
        self.declare_parameter("robot_table.height", 0.827,
                               ParameterDescriptor(description="Robot table height"))
        self.robot_table_height = self.get_parameter("robot_table.height").get_parameter_value().double_value
        self.declare_parameter("ceiling_height", 2.4,
                               ParameterDescriptor(description="Ceiling height from floor"))
        self.ceiling_height = self.get_parameter("ceiling_height").get_parameter_value().double_value
        self.declare_parameter("side_wall.distance", 1.0,
                               ParameterDescriptor(description="Side wall distance from base of robot"))
        self.side_wall_distance = self.get_parameter("side_wall.distance").get_parameter_value().double_value
        self.declare_parameter("side_wall.height", 2.4,
                               ParameterDescriptor(description="Side wall height from floor"))
        self.side_wall_height = self.get_parameter("side_wall.height").get_parameter_value().double_value
        self.declare_parameter("back_wall.distance", 0.75,
                               ParameterDescriptor(description="Back wall distance from base of robot"))
        self.back_wall_distance = self.get_parameter("back_wall.distance").get_parameter_value().double_value
        self.declare_parameter("back_wall.height", 0.46,
                               ParameterDescriptor(description="Back wall height from floor"))
        self.back_wall_height = self.get_parameter("back_wall.height").get_parameter_value().double_value
        self.declare_parameter("lightsaber.full_length", 1.122,
                               ParameterDescriptor(description="Lightsaber full length"))
        self.lightsaber_full_length = self.get_parameter("lightsaber.full_length").get_parameter_value().double_value
        self.declare_parameter("lightsaber.grip_offset", 0.15,
                               ParameterDescriptor(description="Lightsaber grip offset"))
        self.lightsaber_grip_offset = self.get_parameter("lightsaber.grip_offset").get_parameter_value().double_value
        self.declare_parameter("lightsaber.gripper_height", 0.08,
                               ParameterDescriptor(description="Lightsaber gripper height"))
        self.lightsaber_gripper_height = self.get_parameter("lightsaber.gripper_height").get_parameter_value().double_value
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
        self.config.max_velocity_scaling_factor = 0.3
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

        # self.state = State.IDLE
        self.state = State.IDLE

        self.get_logger().info("moveit_interface_tester node started")

        self.grip = 0

        self.joint_traj = trajectory_msgs.msg.JointTrajectory()

        self.ready = 0

        self.trajectory = control_msgs.action.FollowJointTrajectory.Goal()
        
        self.waypoints = 0

        self.flag = 0

        self.ind = 0

        self.home_waypoint = False

        self.update_allies = False

        self.count = 2

        self.table_len_x = 0.5
        self.table_len_y = 0.8
        self.table_center_x = 1.443
        self.table_center_y = -0.052

        self.obstacles_added = 0

        self.is_waypoint = True
        self.detected_objects = None
        self.detected_allies = []
        self.detected_enemies = []

        self.num_movements = 0
        self.num_moves_completed = 0
        self.is_stab_motion = False

        self.block_height = 0.23
        self.block_width = 0.11
        self.block_length = 0.08
        self.sign = 1
        self.dead_count_pub = self.create_publisher(Int16, 'enemy_dead_count', 10)
        self.dead_enemy_count = None
        self.enemy_cnt = 0

        self.num_waypoints_completed = 0

        self.enemies_before = 0
    def obstacle_info(self):
        """
        Get robot transformation from base frame to end-effector frame.

        Args:
            none

        Returns
        -------
            A GetPlanningScene.result (moveit_msgs/srv/GetPlanningScene)
        """
        self.obstacle_future = \
            self.obstacle_client.call_async(moveit_msgs.srv.GetPlanningScene.Request())
        print(f"Obstacle Info: {self.obstacle_future.result()}")
        self.x = 1
        return self.obstacle_future.result()

    def close_gripper(self):
        grip_msg = control_msgs.action.GripperCommand.Goal()
        grip_msg.command.position = 0.0 #0.01
        grip_msg.command.max_effort = 60.0
        if self.pickup_action_client.server_is_ready():
            self.pickup_action_client.send_goal_async(grip_msg)
    
    def open_gripper(self):
        grip_msg = control_msgs.action.GripperCommand.Goal()
        grip_msg.command.position = 0.03    #0.04
        if self.pickup_action_client.server_is_ready():
            self.pickup_action_client.send_goal_async(grip_msg)

    def find_allies(self):
        all_transforms_found = self.update_detected_objects(ObjectType.ALLY)
        obstacle_list = []
        if all_transforms_found:
            # if ally00_here: # and table_here:
            for i in range(len(self.detected_allies)):
                obstacle = moveit_msgs.msg.CollisionObject()
                obstacle.id = self.detected_allies[i].obj.name               #EDIT: ally name

                shape = shape_msgs.msg.SolidPrimitive()
                shape.type = 1  # Box
                length = 0.078
                width = 0.105
                height = 0.2286                                         #EDIT: ally00.transform.translation.z - table.transform.translation.z
                shape.dimensions = [length, width, height]
                obstacle.primitives = [shape]

                pose = geometry_msgs.msg.Pose()
                pose.position.x = self.detected_allies[i].tf.transform.translation.x
                pose.position.y = self.detected_allies[i].tf.transform.translation.y
                pose.position.z = -0.091 + (height/2)                   #EDIT: table.transform.translation.z + (height/2)
                obstacle.primitive_poses = [pose]

                obstacle.header.frame_id = self.moveit.config.base_frame_id
                obstacle_list.append(obstacle)

            self.moveit.update_obstacles(obstacle_list, delete=False)

    def timer_callback(self):
        """
        Check states and call the specific plan or execute from MoveIt API accordingly.

        Args:
            no arguments

        Returns
        -------
            no returns

        """
        # call MoveIt handler
        self.moveit.handle()

        self.find_allies()

        # State machine
        if self.state == State.MOVE_TO_HOME_START:

            if self.moveit.planning:
                self.state = State.MOVE_TO_HOME_WAIT
            else:
                self.moveit.move_to_home()

        elif self.state == State.MOVE_TO_HOME_WAIT:
            if not self.moveit.busy:
                self.state = State.IDLE

        elif self.state == State.SETUP:
            try:
                table1 = self.tf_buffer.lookup_transform(FRAMES.PANDA_BASE, FRAMES.WORK_TABLE1, rclpy.time.Time())
                table1_here = True
            except TransformException:
                table1_here = False
                return
            try:
                table2 = self.tf_buffer.lookup_transform(FRAMES.PANDA_BASE, FRAMES.WORK_TABLE2, rclpy.time.Time())
                table2_here = True
            except TransformException:
                table2_here = False
                return

            if table1_here and table2_here:
                self.table1_x = table1.transform.translation.x
                self.table_len_x = table2.transform.translation.x - table1.transform.translation.x + 0.1
                self.table_len_y = abs(table1.transform.translation.y) + abs(table2.transform.translation.y) + 0.06
                self.table_center_x = ((abs(table1.transform.translation.x - table2.transform.translation.x))/2) + table1.transform.translation.x
                self.table_center_y = (table1.transform.translation.y + table2.transform.translation.y)/2

                self.add_walls()
                self.add_lightsaber()
                self.obstacles_added = 1
                self.state = State.LOOK_FOR_ENEMY


        # elif self.state == State.FIND_ALLIES:
        #     all_transforms_found = self.update_detected_objects(ObjectType.ALLY)

        #     # if ally00_here: # and table_here:
        #     self.get_logger().info('AHHHHHHHH')
        #     obstacle = moveit_msgs.msg.CollisionObject()
        #     obstacle.id = FRAMES.ALLY + '00'

        #     shape = shape_msgs.msg.SolidPrimitive()
        #     shape.type = 1  # Box
        #     length = 0.078
        #     width = 0.105
        #     height = 0.2286 #-(table.transform.translation.z - ally00.transform.translation.z)
        #     shape.dimensions = [length, width, height]
        #     obstacle.primitives = [shape]

        #     pose = geometry_msgs.msg.Pose()
        #     pose.position.x = ally00.transform.translation.x
        #     pose.position.y = ally00.transform.translation.y
        #     pose.position.z = -0.091 + (height/2) #table.transform.translation.z 0.115
        #     obstacle.primitive_poses = [pose]

        #     obstacle.header.frame_id = self.moveit.config.base_frame_id


        #     self.moveit.update_obstacles([obstacle], delete=False)

        #     self.state = State.LOOK_FOR_ENEMY


        elif self.state == State.LOOK_FOR_ENEMY:
            self.waypoints = 0
            all_transforms_found = self.update_detected_objects(ObjectType.ENEMY)
            self.x_disp = []
            self.rotate = []
            self.waypoint_movements = []
            self.goal_waypoint = geometry_msgs.msg.Pose()
            if not self.moveit.busy:
                if all_transforms_found:
                    self.enemies_after = len(self.detected_enemies)
                    # self.get_logger().info(f'length: {len(self.detected_enemies)}')
                    for i in range(len(self.detected_enemies)):
                        # self.get_logger().info(f'in detected enemies array: {self.detected_enemies[i].tf.transform.translation.x}')
                        self.x_disp.append(self.detected_enemies[i].tf.transform.translation.x - self.table1_x + 0.1)     #add buffer offset
                        self.rotate.append(math.atan2(self.detected_enemies[i].tf.transform.translation.y, self.detected_enemies[i].tf.transform.translation.x))

                        x_pos = self.detected_enemies[i].tf.transform.translation.x
                        y_pos = self.detected_enemies[i].tf.transform.translation.y
                        height = self.detected_enemies[i].tf.transform.translation.z - self.table1_x

                        self.goal_waypoint.position.x = x_pos - (self.lightsaber_full_length*0.75)
                        self.goal_waypoint.position.y = y_pos + self.sign*0.16            #adding slight offset (slightly more than half the block width)
                        self.goal_waypoint.position.z = 0.2 #-self.table_offset + height + 0.18

                        self.goal_waypoint.orientation.x = math.pi
                        self.goal_waypoint.orientation.z = -math.pi/16

                        self.knock_enemy_waypoint = geometry_msgs.msg.Pose()
                        self.knock_enemy_waypoint.position.x = x_pos - (self.lightsaber_full_length*0.75)
                        self.knock_enemy_waypoint.position.y = y_pos + self.sign*0.0725            #adding slight offset (slightly more than half the block width)
                        self.knock_enemy_waypoint.position.z = -self.table_offset + height + 0.18
                        self.knock_enemy_waypoint.orientation.x = math.pi
                        self.knock_enemy_waypoint.orientation.z = -math.pi/16

                        self.get_logger().info("adding waypoints")
                        self.waypoint_movements.append([self.goal_waypoint, self.knock_enemy_waypoint])

                        ####################
                        # Testing check for if ally would be hit by falling enemy block
                        ####################
                        self.check_ally_danger_fall(self.detected_enemies[i], 0)

                    self.state = State.DYNAMIC_MOTION

        elif self.state == State.DYNAMIC_MOTION:
            if self.moveit.planning:
                self.state = State.WAYPOINTS_WAIT
            else:
                # self.get_logger().info(f'waypoint array: {self.waypoint_movements}')
                if self.waypoint_movements:
                    try:
                        # self.get_logger().info(f'at index num_waypoints, num_moves: {self.num_waypoints_completed}, {self.num_moves_completed}')
                        self.moveit.plan_traj_to_pose(self.waypoint_movements[self.num_waypoints_completed][self.num_moves_completed])
                        self.num_moves_completed += 1
                        if self.num_moves_completed%2 == 0:
                            self.num_waypoints_completed += 1
                    except:
                        self.state = State.IDLE
                        pass
        ############
        # PROPOSED FLOW TO DECIDE ATTACK STYLE
        ############
        # after look for enemy or look for ally, first try a left attack
        # call function to check if ally in danger from enemy fall
        # if returns false, change the state to check right attack
        # else, create the waypoints and check in the plan returns an error
        # if returns an error, change state to right attack
        # else switch to execution state
        # repeat with right attack, moving to stab motion if invalid
        # repeat with stab motion, change to no attack possible if invalid

        # Ideas for looping through multiple obstacles: 
        #   1. add a state to go to after each execution is done to check if enemies are 
        #       left and incremeant to the next one?
        #   2. look for enemies in a function called in the timer after an initial 'start' 
        #       serivce is called


        # elif self.state == State.LEFT_ATTACK:
        #     if_ally_danger = self.check_ally_danger_fall(self.detected_enemies[i], 0)
        #     if if_ally_danger:
        #         self.state = State.RIGHT_ATTACK
        #     else:
        #         # essentially what's in the look for enemy state
        #         if ik compute or plan returns not success:
        #             self.state = State.RIGHT_ATTACK
        #         else:
        #             self.state = State.DYNAMIC_MOTION
        

        elif self.state == State.STAB_MOTION:
            self.is_stab_motion = True
            joint_waypoints = []
            if self.moveit.planning:
                self.state = State.WAYPOINTS_WAIT
            else:
                #####################################
                # Come in from center
                #####################################
                #goal waypoint
                self.is_waypoint = False
                for i in range(len(self.x_disp)):
                    self.waypoint_joints1 = [self.rotate[i],        #ONLY CHANGE THIS ONE(rotate panda_joint1)
                                            -0.7853981633974483,    # panda_joint2
                                            0.0,                    # panda_joint3
                                            -2.356194490192345,     # panda_joint4
                                            0.0,                    # panda_joint5
                                            (math.pi*5)/6,     # panda_joint6
                                            0.7853981633974483,     # panda_joint7
                                                                    # TODO - This might open the gripper when we try to move home
                                                                    # CAREFUL!
                                            0.0,                  # 0.035, 0.0 panda_finger_joint1
                                            0.0   
                                            ]

                    self.waypoint_joints2 = [self.rotate[i],        #ONLY CHANGE THIS ONE(rotate panda_joint1)
                                            math.radians(-50),    # panda_joint2
                                            math.radians(-1),                    # panda_joint3
                                            math.radians(-165),     # panda_joint4
                                            math.radians(0),                    # panda_joint5
                                            math.radians(108),     # panda_joint6
                                            math.radians(45),     # panda_joint7
                                                                    # TODO - This might open the gripper when we try to move home
                                                                    # CAREFUL!
                                            0.0,                  # 0.035, 0.0 panda_finger_joint1
                                            0.0   
                                            ]


                    self.waypoint_joints3 = [self.rotate[i],        #ONLY CHANGE THIS ONE(rotate panda_joint1)
                                            math.radians(-50 + ((100/0.4826)*self.x_disp[i])),    # panda_joint2     0.4826 meters is width of block table and 107 deg is the total degrees this joint changes to reach end of table
                                            math.radians(-1),                    # panda_joint3
                                            math.radians(-165 + ((100/0.4826)*self.x_disp[i])),     # panda_joint4
                                            math.radians(0),       # panda_joint5
                                            math.radians(108 - ((8/0.4826)*self.x_disp[i])),     # panda_joint6
                                            math.radians(45),     # panda_joint7
                                                                    # TODO - This might open the gripper when we try to move home
                                                                    # CAREFUL!
                                            0.0,                  # 0.035, 0.0 panda_finger_joint1
                                            0.0   
                                            ]

                    self.waypoint_joints4 = [self.rotate[i],        #ONLY CHANGE THIS ONE(rotate panda_joint1)
                                            math.radians(-50),    # panda_joint2
                                            math.radians(-1),                    # panda_joint3
                                            math.radians(-165),     # panda_joint4
                                            math.radians(0),                    # panda_joint5
                                            math.radians(108),     # panda_joint6
                                            math.radians(45),     # panda_joint7
                                                                    # TODO - This might open the gripper when we try to move home
                                                                    # CAREFUL!
                                            0.0,                  # 0.035, 0.0 panda_finger_joint1
                                            0.0   
                                            ]

                    joint_movements = [self.waypoint_joints1, self.waypoint_joints2, self.waypoint_joints3, self.waypoint_joints4]
                    self.num_movements = len(joint_movements)
                    joint_waypoints.append(joint_movements)

                self.moveit.joint_waypoints(joint_waypoints[self.num_waypoints_completed][self.num_moves_completed])
                self.num_moves_completed += 1
                if self.num_moves_completed%4 == 0:
                    self.num_waypoints_completed += 1

        elif self.state == State.WAYPOINTS:
            if self.moveit.planning:
                self.state = State.WAYPOINTS_WAIT
            else:
                #add wait for collision object to be in planning scene before plan! (new state?)
                #self.moveit.check_planning_scene(self.goal_waypoint)
                # if ___:
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

                if self.moveit.get_last_error() == MoveItApiErrors.NO_ERROR:
                    self.state = State.EXECUTE_START
                    self.get_logger().info("start execute!")
                elif self.sign == 1:
                    self.sign = -1
                    self.state = State.DYNAMIC_MOTION
                elif not self.is_stab_motion:
                    self.num_moves_completed = 0
                    self.num_waypoints_completed = 0
                    self.state = State.STAB_MOTION
                else:
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
                if self.num_moves_completed < self.num_movements:
                    if not self.is_stab_motion:
                        self.get_logger().info("wrong!")
                        self.state = State.DYNAMIC_MOTION
                    else:
                        self.state = State.STAB_MOTION
                else:
                    self.get_logger().info("done!")
                    all_transforms_found = self.update_detected_objects(ObjectType.ENEMY)
                    if all_transforms_found:
                        self.enemies_after = len(self.detected_enemies)
                        self.state = State.MOVE_TO_HOME_START
                        # TODO: change - self.enemies_before is currently hard coded to 0 in the init
                        self.dead_enemy_count+= self.enemies_before - self.enemies_after


        if self.dead_enemy_count:
            self.dead_count_pub.publish(self.dead_enemy_count)



    def move_to_home_callback(self, request, response):
        """
        Move to home position

        This works to move our end effector to the home postion, but it doesn't
        necessarily put all our joint states at home. Best way to do that probably
        be to add to the API a function that allows us to plan and execute a move
        with the input joint states
        panda_joint1 = 0 deg
        panda_joint2 = -45 deg
        panda_joint3 = 0 deg
        panda_joint4 = -135 deg
        panda_joint5 = 0 deg
        panda_joint6 = 90 deg
        panda_joint7 = 45 deg
        
        TODO finish docstring
        """
        self.get_logger().info("WHAT")
        # no longer necessary since we're using the API home function
        self.goal_pose = copy.deepcopy(self.home_pose)

        self.state = State.MOVE_TO_HOME_START
        self.get_logger().info("WHAT")

        return response

    def look_for_enemy_callback(self, request, response):
        if self.obstacles_added == 0:
            self.state = State.SETUP
        else:
            self.state = State.FIND_ALLIES
        return response

    def grip_open_close_callback(self, request, response):
        if self.grip == 0:
            self.close_gripper()
            self.grip = 1
        elif self.grip == 1:
            self.open_gripper()
            self.grip = 0
        return response

    def gripper_grasp_callback(self, request, response):
        #Note: might need to command robot to stay up when holding lightsaber
        self.grip_lightsaber_client.wait_for_server()
        if self.grip_lightsaber_client.server_is_ready():
            grip_goal = franka_msgs.action.Grasp.Goal()
            grip_goal.width = 0.00 #0.033
            grip_goal.epsilon.inner = 0.005
            grip_goal.epsilon.outer = 0.005
            grip_goal.speed = 0.03
            grip_goal.force = 80.0
            self.grip_lightsaber_client.send_goal_async(grip_goal)
        return response


    # def grip_lightsaber_callback(self, request, response):
    #     grip_lightsaber = moveit_msgs.msg.Grasp()
    #     grip_lightsaber.id = 'Grip_Lightsaber'

    #     # hand_posture = trajectory_msgs.msg.JointTrajectory()
    #     # hand_posture


    #     ee_pose_grip = geometry_msgs.msg.PointStamped()
    #     ee_pose_grip.header.frame_id = 'panda_hand_tcp'
    #     ee_pose_grip.header.stamp = self.get_clock().now().to_msg()
    #     # ee_pose_grip.point.x = 0.306891
    #     # ee_pose_grip.point.y = -8.32667e-17
    #     # ee_pose_grip.point.z = 0.486882

    #     # ee_pose_grip.orientation.x = 1.0
    #     # ee_pose_grip.orientation.y = 1.38778e-16
    #     # ee_pose_grip.orientation.z = 2.22045e-16
    #     # ee_pose_grip.orientation.w = -6.93889e-17

    #     grip_lightsaber.max_contact_force = 0.0
        
    #     if self.grip_lightsaber_client.server_is_ready():
    #         self.grip_lightsaber_client.send_goal_async(grip_lightsaber)
    #         #add attached collision objects

    #     return response


    def test_jointtrajectory_callback(self, request, response):
        traj = trajectory_msgs.msg.JointTrajectory()
        self.joint_traj = trajectory_msgs.msg.JointTrajectory()
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point1 = trajectory_msgs.msg.JointTrajectoryPoint()
        point2 = trajectory_msgs.msg.JointTrajectoryPoint()
        point3 = trajectory_msgs.msg.JointTrajectoryPoint()
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        traj.header.frame_id = ''
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7']

        position1 = [
            0.0,                    # panda_joint1
            -0.7853981633974483,    # panda_joint2
            0.0,                    # panda_joint3
            -2.356194490192345,     # panda_joint4
            0.0,                    # panda_joint5
            1.5707963267948966,     # panda_joint6
            0.7853981633974483,     # panda_joint7
                                    # TODO - This might open the gripper when we try to move home
                                    # CAREFUL!
            0.035,                  # panda_finger_joint1
            0.035,                  # panda_finger_joint2
        ]
        position2 = [0.069813, -0.5235988, 0.261799, -2.164208, 0.139626, 1.65806, 1.082104]
        

    #     position3 = [math.radians(-14), math.radians(-45), math.radians(0), math.radians(-135), math.radians(0), math.radians(90), math.radians(45)]

    #     velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    #     point.positions = position1
    #     point.time_from_start.sec = 1

    #     point1.positions = position1
    #    # point.velocities = velocities
    #     point1.time_from_start.sec = 1

    #     point2.positions = position2
    #     point2.time_from_start.sec = 1

    #     point3.positions = position3
    #     point3.time_from_start.sec = 1

    #     #traj.points = [point1, point2, point3]
    #     traj.points = [point]

        

    #     self.trajectory = control_msgs.action.FollowJointTrajectory.Goal()
    #     self.trajectory = traj
    #    # trajectory.component_path_tolerance = 
        
    #     self.ready = 1

    #     self.joint_traj = traj
        #self.pub_joint_traj.publish(self.joint_traj)
        return response

    def joint_positions(self, point):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.header.frame_id = ''
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7']

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = self.position[point]
        point.time_from_start.sec = 1

        traj.points = [point]

        self.trajectory = control_msgs.action.FollowJointTrajectory.Goal()
        self.trajectory = traj

        self.ready = 1

        self.joint_traj = traj
        self.pub_joint_traj.publish(self.joint_traj)


    def gripper_callback(self, request, response):
        #self.grip = request
        # self.get_logger().info(f'Grip request {self.grip}')

        # if self.grip == 0:
        #     self.close_gripper()
        #     self.grip = 1
        # elif self.grip == 1:
        #     self.open_gripper()
        #     self.grip = 0

        
        # collision_objects = moveit_msgs.msg.CollisionObject
        # collision_objects.id = "Box_0"
        # co_header = std_msgs.msg.Header
        # co_header.frame_id = 'panda_link0'
        # co_header.stamp = self.get_clock().now().to_msg()
        # collision_objects.primitives.resize(1)
        # collision_objects.primitives.type = collision_objects.primitives.BOX
        # collision_objects.primitives.dimensions.resize(3)
        # collision_objects.primitives.dimensions[0] = 0.2
        # collision_objects.primitives.dimensions[1] = 0.4
        # collision_objects.primitives.dimensions[2] = 0.4
        # collision_objects.primitive_poses.resize(1)
        # collision_objects.primitive_poses.position.x = 0.5
        # collision_objects.primitive_poses.position.y = 0
        # collision_objects.primitive_poses.position.z = 0.2
        #collision_objects.primitive_poses.orientation.w = 1.0


        # grasp = moveit_msgs.msg.Grasp
        # grasp.id = "panda_manipulator"

        # grasp.grasp_pose.header.frame_id = 'panda_link0'
        # grasp.grasp_pose.pose.orientation.w = 1.0
        # #grasp.grasp_pose.pose.position.x = 0.

        # grasp_post = trajectory_msgs.msg.JointTrajectory
        # grasp_pose = trajectory_msgs.msg.JointTrajectoryPoint




   

        # grasp_post.joint_names.resize(2)
        # grasp_post.joint_names[0] = "panda_finger_joint1"
        # grasp_post.joint_names[1] = "panda_finger_joint2"

        # # Set them as closed.
        # grasp_post.points.resize(1)
        # grasp_pose[0].positions.resize(2)
        # grasp_pose[0].positions[0] = 0.00
        # grasp_pose[0].positions[1] = 0.00
        # grasp.grasp_posture = grasp_pose


        # grip = moveit_msgs.srv.GraspPlanning.Response

        # grip.grasps = grasp
        # self.get_logger().info(f'grip message to send {grip}')

        

        #response = grip

        return response
        
    # def close_gripper(self):
    #     grasp_post = trajectory_msgs.msg.JointTrajectory
    #     grasp_pose = trajectory_msgs.msg.JointTrajectoryPoint

    #     grasp_post.joint_names.resize(2)
    #     grasp_post.joint_names[0] = "panda_finger_joint1"
    #     grasp_post.joint_names[1] = "panda_finger_joint2"

    #     # Set them as closed.
    #     grasp_post.points.resize(1)
    #     grasp_pose[0].positions.resize(2)
    #     grasp_pose[0].positions[0] = 0.00
    #     grasp_pose[0].positions[1] = 0.00

    def waypoint_callback(self, request, response):
        self.waypoints = 1
        self.waypoint_poses()
        self.state = State.WAYPOINTS
        return response
        

    def move_to_pose_callback(self, request, response):
        """
        Call /move_to_pose (moveit_testing_interfaces/srv/MoveToPose) service.

        Pose value given by user is stored
        State is updated to PLAN_TO_POSE_START

        Example call:
            ros2 service call /move_to_pose moveit_testing_interfaces/srv/MoveToPose "pose: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0., y: 0., z: 0., w: 1.}}"

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
       # self.waypoints = 1

        return response

    def home_waypoint_callback(self, request, response):
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
        #self.goal_pose.position = request.position
        self.goal_home_waypoint.position  = home_point
        self.goal_home_waypoint.orientation = home_orientation

        self.home_waypoint = True

        self.state = State.WAYPOINTS

        return response

    def waypoint_poses(self):
        self.state = State.WAYPOINTS
        # self.goal_waypoint.position.x = 0.6443
        # self.goal_waypoint.position.y = -0.1
        # self.goal_waypoint.position.z = 0.5
        self.goal_waypoint.orientation.x = math.pi

    #     self.waypoints = 1
    #     point1 = geometry_msgs.msg.Point()
    #     point1.x = 0.35
    #     point1.y = 0.0
    #     point1.z = 0.6
    #     orientation1 = geometry_msgs.msg.Pose().orientation
    #     orientation1.x = math.pi
    #     # orientation1.y = 0.0
    #     # orientation1.z = 0.0
    #     # orientation1.w = 0.0
    #     # orientation1.x = 1.0
    #     # orientation1.y = 0.5
    #     # orientation1.z = 0.0
    #     #orientation1.w = 2.0

    #     point2 = geometry_msgs.msg.Point()
    #     point2.x = 0.2
    #     point2.y = 0.35
    #     point2.z = 0.8
    #     orientation2 = geometry_msgs.msg.Pose().orientation
    #     orientation2.x = -1.0

    #     point3 = geometry_msgs.msg.Point()
    #     point3.x = -0.1
    #     point3.y = 0.35
    #     point3.z = 0.7
    #     orientation3 = geometry_msgs.msg.Pose().orientation
    #     orientation3.x = -1.0

    #     point4 = geometry_msgs.msg.Point()
    #     point4.x = -0.3
    #     point4.y = 0.0
    #     point4.z = 0.8
    #     orientation4 = geometry_msgs.msg.Pose().orientation
    #     orientation4.w = 1.0

    #     point5 = geometry_msgs.msg.Point()
    #     point5.x = -0.3
    #     point5.y = -0.3
    #     point5.z = 0.9
    #     orientation5 = geometry_msgs.msg.Pose().orientation
    #     orientation5.x = 1.0

    #     point6 = geometry_msgs.msg.Point()
    #     point6.x = -0.3
    #     point6.y = -0.4
    #     point6.z = 0.5
    #     orientation6 = geometry_msgs.msg.Pose().orientation
    #     orientation6.x = math.pi

    #     points = [point1, point2, point3, point4, point5, point6]
    #     orientations = [orientation1, orientation2, orientation3, orientation4, orientation5, orientation6]
    #     #points = [point1, point2]
    #     #orientations = [orientation1, orientation2]
    #     self.goal_waypoint = geometry_msgs.msg.Pose()
    #     #self.goal_pose.position = request.position
    #     # self.goal_waypoint.position  = points[0]
    #     # self.goal_waypoint.orientation = orientations[0]
    #     self.flag = 1
    #     self.i = len(points)
    #     self.state = State.WAYPOINTS
    #     i = self.ind
    #    # for i in range(1, len(points)):
    #     if i < len(points):
    #         if self.moveit._excecution_complete == 1:
    #             self.goal_waypoint = geometry_msgs.msg.Pose()
    #             #self.goal_pose.position = request.position
    #             self.goal_waypoint.position  = points[i]
    #             self.goal_waypoint.orientation = orientations[i]
    #             self.get_logger().info(f'point index: {i}')
    #             self.flag = 1
    #             self.state = State.WAYPOINTS
    #         else:
    #             pass

    def test_waypoint_lightsaber_callback(self, request, response):
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

        Example call:
        ros2 service call /update_obstacles moveit_testing_interfaces/srv/UpdateObstacles "{position: {x: 0.5, y: 0.5, z: 1.0}, length: 0.5, width: 0.25, height: 2.0, id: 'MyBox', delete_obstacle: false}"
        ros2 service call /update_obstacles moveit_testing_interfaces/srv/UpdateObstacles "{position: {x: 0.5, y: 0.0, z: 0.0}, length: 1.125, width: 0.033, height: 0.2, id: 'gripping', delete_obstacle: false}"

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
        #shape.type = 1  # Box
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

        Example call:
        ros2 service call /update_persistent_obstacles moveit_testing_interfaces/srv/UpdateObstacles "{position: {x: 0.0, y: -0.9, z: 0.0}, length: 12.0, width: 0.25, height: 4.0, id: 'wall1', delete_obstacle: false}"
        ros2 service call /update_persistent_obstacles moveit_testing_interfaces/srv/UpdateObstacles "{position: {x: 0.0, y: 0.9, z: 0.0}, length: 12.0, width: 0.25, height: 4.0, id: 'wall2', delete_obstacle: false}"

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

        self.moveit.update_persistent_obstacle(obstacle, delete=request.delete_obstacle)

        return response

    def add_lightsaber(self):
        attached_obstacle = moveit_msgs.msg.AttachedCollisionObject()
        attached_obstacle.link_name = 'panda_hand_tcp'
        attached_obstacle.object.header.frame_id = 'panda_hand_tcp'
        attached_obstacle.object.header.stamp = self.get_clock().now().to_msg()
        attached_obstacle.object.id = 'gripping'

        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.411
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.y = -1.0
        attached_obstacle.object.primitive_poses = [pose]

        shape = shape_msgs.msg.SolidPrimitive()
        shape.type = 3  # Cylinder
        shape.dimensions = [1.125, 0.033, 0.2]
        attached_obstacle.object.primitives = [shape]

        attached_obstacle.object.operation = attached_obstacle.object.ADD

        attached_obstacle.touch_links = ['panda_rightfinger', 'panda_leftfinger', 'panda_hand_tcp', 'panda_hand']

        self.moveit.update_attached_obstacles(attached_obstacle, delete=False)

    def attached_obstacles_callback(self, request, response):
        """
        Call /update_persistent_obstacles (moveit_testing_interfaces/srv/UpdateAttachedObstacles) service.

        Store obstacle position, dimensions, ids and delete flag value input by the user

        Example call:
        ros2 service call /update_attached_obstacles moveit_testing_interfaces/srv/UpdateAttachedObstacles "{link_name: "panda_hand_tcp", position: {x: 0.411, y: 0.0, z: 0.0}, length: 1.125, width: 0.033, height: 0.2, id: 'gripping', type: 3, delete_obstacle: false}"
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

        attached_obstacle.touch_links = ['panda_rightfinger', 'panda_leftfinger', 'panda_hand_tcp', 'panda_hand']

        self.moveit.update_attached_obstacles(attached_obstacle, delete=request.delete_obstacle)

        return response

    def add_walls(self):
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
        # self.moveit.update_persistent_obstacle(obstacle, delete=False)

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

        pose3 = geometry_msgs.msg.Pose()
        pose3.position.x = self.table_center_x 
        pose3.position.y = self.table_center_y 
        pose3.position.z = -0.091
        obstacle3.primitive_poses = [pose3]

        shape3 = shape_msgs.msg.SolidPrimitive()
        shape3.type = 1  # Box
        shape3.dimensions = [self.table_len_x, self.table_len_y, 0.023]
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
        pose5.position.z = self.ceiling_height
        obstacle5.primitive_poses = [pose5]

        shape5 = shape_msgs.msg.SolidPrimitive()
        shape5.type = 1  # Boxobstacle = moveit_msgs.msg.CollisionObject()
        obstacle.id = 'wall_0'

        shape5.dimensions = [4.0, 2.0, 0.02]
        obstacle5.primitives = [shape5]

        obstacle5.header.frame_id = self.moveit.config.base_frame_id

        obstacle6 = moveit_msgs.msg.CollisionObject()
        obstacle6.id = 'gripper_height_offset'

        pose6 = geometry_msgs.msg.Pose()
        pose6.position.x = 0.355
        pose6.position.y = 0.0
        pose6.position.z = self.lightsaber_gripper_height/2
        obstacle6.primitive_poses = [pose6]

        shape6 = shape_msgs.msg.SolidPrimitive()
        shape6.type = 1  # Box
        shape6.dimensions = [0.35, self.robot_table_width, self.lightsaber_gripper_height]
        obstacle6.primitives = [shape6]

        obstacle6.header.frame_id = self.moveit.config.base_frame_id

        self.moveit.update_persistent_obstacle([obstacle, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6], delete=False)

        
        #arm table should be attached collision object
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
        shape2.dimensions = [self.robot_table_length, self.robot_table_width, self.robot_table_height]
        attached_obstacle.object.primitives = [shape2]

        attached_obstacle.object.operation = attached_obstacle.object.ADD

        attached_obstacle.touch_links = ['panda_link0', 'panda_link1']

        self.moveit.update_attached_obstacles(attached_obstacle, delete=False)



    def add_walls_callback(self, request, response):

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
        # self.moveit.update_persistent_obstacle(obstacle, delete=False)

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

        pose3 = geometry_msgs.msg.Pose()
        pose3.position.x = self.table_center_x 
        pose3.position.y = self.table_center_y 
        pose3.position.z = -0.091
        obstacle3.primitive_poses = [pose3]

        shape3 = shape_msgs.msg.SolidPrimitive()
        shape3.type = 1  # Box
        shape3.dimensions = [self.table_len_x, self.table_len_y, 0.023]
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
        shape4.dimensions = [0.3, 4.0, self.back_wall_height]
        obstacle4.primitives = [shape4]

        obstacle4.header.frame_id = self.moveit.config.base_frame_id

        obstacle5 = moveit_msgs.msg.CollisionObject()
        obstacle5.id = 'ceiling'

        pose5 = geometry_msgs.msg.Pose()
        pose5.position.x = 0.0
        pose5.position.y = 0.0
        pose5.position.z = self.ceiling_height
        obstacle5.primitive_poses = [pose5]

        shape5 = shape_msgs.msg.SolidPrimitive()
        shape5.type = 1  # Box
        shape5.dimensions = [4.0, 2.0, 0.02]
        obstacle5.primitives = [shape5]

        obstacle5.header.frame_id = self.moveit.config.base_frame_id

        obstacle6 = moveit_msgs.msg.CollisionObject()
        obstacle6.id = 'gripper_height_offset'

        pose6 = geometry_msgs.msg.Pose()
        pose6.position.x = 0.0
        pose6.position.y = 0.0355
        pose6.position.z = self.lightsaber_gripper_height
        obstacle6.primitive_poses = [pose6]

        shape6 = shape_msgs.msg.SolidPrimitive()
        shape6.type = 1  # Box
        shape6.dimensions = [self.robot_table_length, 0.04, self.robot_table_height]
        obstacle6.primitives = [shape6]

        obstacle6.header.frame_id = self.moveit.config.base_frame_id

        self.moveit.update_persistent_obstacle([obstacle, obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6], delete=False)

        
        #arm table should be attached collision object
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
        shape2.dimensions = [self.robot_table_length, self.robot_table_width, self.robot_table_height]
        attached_obstacle.object.primitives = [shape2]

        attached_obstacle.object.operation = attached_obstacle.object.ADD

        attached_obstacle.touch_links = ['panda_link0', 'panda_link1']

        self.moveit.update_attached_obstacles(attached_obstacle, delete=False)

        return response

    def obj_detection_callback(self, data):
        # self.get_logger().info(f'object detection: {data}')
        self.detected_objects = data

    def update_detected_objects(self, object_type):
        
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
                    obj_data.tf = self.tf_buffer.lookup_transform(FRAMES.PANDA_BASE, obj_data.obj.name, rclpy.time.Time())
                    # x_pos = obj_data.tf.transform.translation.x
                    # y_pos = obj_data.tf.transform.translation.y
                    # height = obj_data.tf.transform.translation.z
                    # self.get_logger().info(f'detected ally position: ({x_pos}, {y_pos}, {height})')

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
                    obj_data.tf = self.tf_buffer.lookup_transform(FRAMES.PANDA_BASE, obj_data.obj.name, rclpy.time.Time())
                    # x_pos = obj_data.tf.transform.translation.x
                    # y_pos = obj_data.tf.transform.translation.y
                    # height = obj_data.tf.transform.translation.z
                    # self.get_logger().info(f'detected enemy position: ({x_pos}, {y_pos}, {height})')

                except TransformException:
                    all_transforms_found = False

                # Append object data to array
                self.detected_enemies.append(obj_data)

        # Indicate if all transforms were found
        return all_transforms_found

    def check_ally_danger_fall(self, enemy_obj, swing_style):
        # y is left to right
        # x is forward to backward
        enemy_to_ally = DetectedObjectData(enemy_obj)
        for ally in self.detected_allies:
            enemy_to_ally = self.tf_buffer.lookup_transform(ally.obj.name, enemy_obj.obj.name, rclpy.time.Time())
            dist_y = enemy_to_ally.transform.translation.y
            dist_x = enemy_to_ally.transform.translation.x
            self.get_logger().info(f'x,y distance to enemy: ({dist_x}, {dist_y})')
            if swing_style == 0:
                # swinging so the brick falls to the left from desk view
                self.get_logger().info(f'dist_y factor {abs(dist_y-0.5*self.block_width)}, {self.block_width*1.5}')
                self.get_logger().info(f'dist_x factor {(dist_x-0.5*self.block_width)}, {self.block_height+self.block_width*0.5}')
                if (abs(dist_y-0.5*self.block_width) < self.block_width*1.5) and (dist_x>=0) and ((dist_x-0.5*self.block_width) < (self.block_height+self.block_width*0.5)):
                    self.get_logger().info(f'not safe to attack in left swing')
                    # return False
                swing_style = 1
            if swing_style == 1:
                # swinging so the brick falls to the right
                self.get_logger().info(f'dist_y factor {abs(dist_y-0.5*self.block_width)}, {self.block_width*1.5}')
                self.get_logger().info(f'dist_x factor {(dist_x+0.5*self.block_width)}, {-(self.block_height+self.block_width*0.5)}')
                if (abs(dist_y-0.5*self.block_width) < self.block_width*1.5) and (dist_x <= 0) and ((dist_x+0.5*self.block_width) > -(self.block_height+self.block_width*0.5)):
                    self.get_logger().info(f'not safe to attack in right swing')
                    # return False
                swing_style = 2
            if swing_style == 2:
                # swinging so the brick falls straight backwards
                self.get_logger().info(f'dist_y factor {(dist_y-0.5*self.block_width)}, {(self.block_height+self.block_width*0.5)}')
                self.get_logger().info(f'dist_x factor {abs(dist_x-0.5*self.block_width)}, {self.block_width}')
                if (abs(dist_x-0.5*self.block_width) < self.block_width) and ((dist_y-0.5*self.block_width) < (self.block_height+self.block_width*0.5)):
                    self.get_logger().info(f'not safe to attack in stabbing style')
                    # return False
            # self.get_logger().info(f'safe to attack in style {swing_style}')
        return True

def movegroup_entry(args=None):
    rclpy.init(args=args)
    movegroup = MoveGroup()
    rclpy.spin(movegroup)
    rclpy.shutdown()
