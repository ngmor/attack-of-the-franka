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
    WAYPOINTS = auto()
    WAYPOINTS_WAIT = auto()
    NEXT_WAYPOINT = auto()


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
        self.srv_add_walls = self.create_service(
            std_srvs.srv.Empty, 'add_walls', self.add_walls_callback)


        # Initialize API class
        config = MoveConfig()
        config.base_frame_id = 'panda_link0'
        config.workspace_min_corner = geometry_msgs.msg.Vector3(
            x=-3.0,
            y=-3.0,
            z=-3.0
        )
        config.workspace_max_corner = geometry_msgs.msg.Vector3(
            x=3.0,
            y=3.0,
            z=3.0
        )
        config.tolerance = 0.01
        config.group_name = 'panda_manipulator'

        self.home_pose = geometry_msgs.msg.Pose()
        self.waypoint_joints = []

        # Select end effector attributes based on group name
        if config.group_name == 'panda_arm':
            config.ee_frame_id = 'panda_link8'  # end effector frame for panda_arm
            self.home_pose.position.x = 0.30691
            self.home_pose.position.y = 0.0
            self.home_pose.position.z = 0.590282
            self.home_pose.orientation.x = 0.92388
            self.home_pose.orientation.y = -0.382683
            self.home_pose.orientation.z = 8.32667e-17
            self.home_pose.orientation.w = 8.32667e-17
        elif config.group_name == 'panda_manipulator':
            config.ee_frame_id = 'panda_hand_tcp'  # end effector frame for panda_manipulator
            self.home_pose.position.x = 0.306891
            self.home_pose.position.y = -8.32667e-17
            self.home_pose.position.z = 0.486882
            self.home_pose.orientation.x = 1.0
            self.home_pose.orientation.y = 1.38778e-16
            self.home_pose.orientation.z = 2.22045e-16
            self.home_pose.orientation.w = -6.93889e-17

        config.home_joint_positions = [
            0.0,                    # panda_joint1
            -0.7853981633974483,    # panda_joint2
            0.0,                    # panda_joint3
            -2.356194490192345,     # panda_joint4
            0.0,                    # panda_joint5
            1.5707963267948966,     # panda_joint6
            0.7853981633974483,     # panda_joint7
                                    # TODO - This might open the gripper when we try to move home
                                    # CAREFUL!
            0.035,                  # 0.035, 0.0 panda_finger_joint1
            0.035,                  # 0.035, 0.0 panda_finger_joint2
        ]

        self.waypoint_joints = [math.radians(3), math.radians(-35), math.radians(31), math.radians(-111), math.radians(17), math.radians(81), math.radians(77), 0.035, 0.035]

        

        self.moveit = MoveIt(self, config)

        self.goal_pose = geometry_msgs.msg.Pose()
        self.goal_waypoint = geometry_msgs.msg.Pose()
        self.plan = None

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

        if self.ready == 1:
            if self.test_followjoints.server_is_ready():
                self.test_followjoints.send_goal_async(self.trajectory)
        

        # self.close_gripper()
        # if self.pickup_action_client.server_is_ready():
        #     self.pickup_action_client.send_goal_async(self.close_gripper())
        #     self.get_logger().info(f'gripper info output: {self.close_gripper()}')

        # State machine
        if self.state == State.MOVE_TO_HOME_START:

            if self.moveit.planning:
                self.state = State.MOVE_TO_HOME_WAIT
            else:
                self.moveit.move_to_home()

        elif self.state == State.MOVE_TO_HOME_WAIT:
            if not self.moveit.busy:
                self.state = State.IDLE

        if self.state == State.WAYPOINTS:

            if self.moveit.planning:
                self.state = State.WAYPOINTS_WAIT
            else:
                #add wait for collision object to be in planning scene before plan! (new state?)
                #self.moveit.check_planning_scene(self.goal_waypoint)
                # if ___:
                self.moveit.plan_traj_to_pose(self.goal_waypoint)
                #self.moveit.joint_waypoints(self.waypoint_joints)

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
                else:
                    self.state = State.IDLE

        elif self.state == State.NEXT_WAYPOINT:
                if self.home_waypoint == True:
                    self.state = State.IDLE
                    self.home_waypoint = False
                elif self.ind < self.i:
                    self.ind += 1
                    self.waypoint_poses()
                    self.state = State.WAYPOINTS
                else:
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
                if self.waypoints == 1:
                    self.state = State.NEXT_WAYPOINT
                else:
                    self.state = State.IDLE

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
        # no longer necessary since we're using the API home function
        self.goal_pose = copy.deepcopy(self.home_pose)

        self.state = State.MOVE_TO_HOME_START

        return response


    # def command_gripper_callback(self, command_grip):
    #     self.get_logger().info(f'Gripper Info: {command_grip.request}')
    #     grip_msg = control_msgs.action.GripperCommand.Goal()
    #     grip_msg.command.position = 0.04
    #     grip_msg.command.max_effort = 5.0
    #     if self.pickup_action_client.server_is_ready():
    #         self.pickup_action_client.send_goal_async(self.close_gripper())
    #         self.get_logger().info(f'gripper info output: {self.close_gripper()}')
    #     result = control_msgs.action.GripperCommand.Result()
    #     # result.position = 0.0
    #     # result.effort = 5.0
    #     # result.reached_goal = True
    #     # result.stalled = False
    #     return result


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
        self.waypoints = 1
        point1 = geometry_msgs.msg.Point()
        point1.x = 0.35
        point1.y = 0.0
        point1.z = 0.6
        orientation1 = geometry_msgs.msg.Pose().orientation
        orientation1.x = math.pi
        # orientation1.y = 0.0
        # orientation1.z = 0.0
        # orientation1.w = 0.0
        # orientation1.x = 1.0
        # orientation1.y = 0.5
        # orientation1.z = 0.0
        #orientation1.w = 2.0

        point2 = geometry_msgs.msg.Point()
        point2.x = 0.2
        point2.y = 0.35
        point2.z = 0.8
        orientation2 = geometry_msgs.msg.Pose().orientation
        orientation2.x = -1.0

        point3 = geometry_msgs.msg.Point()
        point3.x = -0.1
        point3.y = 0.35
        point3.z = 0.7
        orientation3 = geometry_msgs.msg.Pose().orientation
        orientation3.x = -1.0

        point4 = geometry_msgs.msg.Point()
        point4.x = -0.3
        point4.y = 0.0
        point4.z = 0.8
        orientation4 = geometry_msgs.msg.Pose().orientation
        orientation4.w = 1.0

        point5 = geometry_msgs.msg.Point()
        point5.x = -0.3
        point5.y = -0.3
        point5.z = 0.9
        orientation5 = geometry_msgs.msg.Pose().orientation
        orientation5.x = 1.0

        point6 = geometry_msgs.msg.Point()
        point6.x = -0.3
        point6.y = -0.4
        point6.z = 0.5
        orientation6 = geometry_msgs.msg.Pose().orientation
        orientation6.x = math.pi

        points = [point1, point2, point3, point4, point5, point6]
        orientations = [orientation1, orientation2, orientation3, orientation4, orientation5, orientation6]
        #points = [point1, point2]
        #orientations = [orientation1, orientation2]
        self.goal_waypoint = geometry_msgs.msg.Pose()
        #self.goal_pose.position = request.position
        # self.goal_waypoint.position  = points[0]
        # self.goal_waypoint.orientation = orientations[0]
        self.flag = 1
        self.i = len(points)
        self.state = State.WAYPOINTS
        i = self.ind
       # for i in range(1, len(points)):
        if i < len(points):
            if self.moveit._excecution_complete == 1:
                self.goal_waypoint = geometry_msgs.msg.Pose()
                #self.goal_pose.position = request.position
                self.goal_waypoint.position  = points[i]
                self.goal_waypoint.orientation = orientations[i]
                self.get_logger().info(f'point index: {i}')
                self.flag = 1
                self.state = State.WAYPOINTS
            else:
                pass

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

    def attached_obstacles_callback(self, request, response):
        """
        Call /update_persistent_obstacles (moveit_testing_interfaces/srv/UpdateAttachedObstacles) service.

        Store obstacle position, dimensions, ids and delete flag value input by the user

        Example call:
        ros2 service call /update_attached_obstacles moveit_testing_interfaces/srv/UpdateAttachedObstacles "{link_name: "panda_hand_tcp", position: {x: 0.3125, y: 0.0, z: 0.0}, length: 1.125, width: 0.033, height: 0.2, id: 'gripping', type: 3, delete_obstacle: false}"
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

    def add_walls_callback(self, request, response):
        obstacle = moveit_msgs.msg.CollisionObject()
        obstacle.id = 'wall_0'

        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.75
        pose.position.y = 0.75
        pose.position.z = 0.0
        obstacle.primitive_poses = [pose]

        shape = shape_msgs.msg.SolidPrimitive()
        shape.type = 1  # Box
        shape.dimensions = [1.125, 0.25, 2.0]
        obstacle.primitives = [shape]

        obstacle.header.frame_id = self.moveit.config.base_frame_id
        # self.moveit.update_persistent_obstacle(obstacle, delete=False)

        obstacle1 = moveit_msgs.msg.CollisionObject()
        obstacle1.id = 'wall_1'

        pose1 = geometry_msgs.msg.Pose()
        pose1.position.x = 0.75
        pose1.position.y = -0.75
        pose1.position.z = 0.0
        obstacle1.primitive_poses = [pose1]

        shape1 = shape_msgs.msg.SolidPrimitive()
        shape1.type = 1  # Box
        shape1.dimensions = [1.125, 0.25, 2.0]
        obstacle1.primitives = [shape1]

        obstacle1.header.frame_id = self.moveit.config.base_frame_id
        # self.moveit.update_persistent_obstacle(obstacle1, delete=False)

        obstacle2 = moveit_msgs.msg.CollisionObject()
        obstacle2.id = 'wall_2'

        pose2 = geometry_msgs.msg.Pose()
        pose2.position.x = -0.75
        pose2.position.y = 0.0
        pose2.position.z = 0.0
        obstacle2.primitive_poses = [pose2]

        shape2 = shape_msgs.msg.SolidPrimitive()
        shape2.type = 1  # Box
        shape2.dimensions = [0.25, 1.125, 2.0]
        obstacle2.primitives = [shape2]

        obstacle2.header.frame_id = self.moveit.config.base_frame_id
        # self.moveit.update_persistent_obstacle(obstacle2, delete=False)

        self.moveit.update_persistent_obstacle([obstacle, obstacle1, obstacle2], delete=False)

        return response


def movegroup_entry(args=None):
    rclpy.init(args=args)
    movegroup = MoveGroup()
    rclpy.spin(movegroup)
    rclpy.shutdown()