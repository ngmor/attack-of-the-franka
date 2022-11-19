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
from .moveit_interface import MoveIt, MoveConfig, MoveItApiErrors
import geometry_msgs.msg
import moveit_msgs.action
import moveit_msgs.srv
import moveit_msgs.msg
import std_srvs.srv
from enum import Enum, auto
import shape_msgs.msg
import moveit_testing_interfaces.srv
import copy


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


class SimpleMove(Node):
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
        super().__init__('simple_move')

        self.interval = 1.0 / 100.0
        self.timer = self.create_timer(self.interval, self.timer_callback)
        self.srv_move_to_home = self.create_service(std_srvs.srv.Empty,
                                                    'move_to_home', self.move_to_home_callback)
        self.srv_move_to_pose = self.create_service(moveit_testing_interfaces.srv.MoveToPose,
                                                    'move_to_pose', self.move_to_pose_callback)
        self.srv_move_to_position = self.create_service(
            moveit_testing_interfaces.srv.MoveToPosition,
            'move_to_position', self.move_to_position_callback)
        self.srv_move_to_orientation = self.create_service(
            moveit_testing_interfaces.srv.MoveToOrientation,
            'move_to_orientation', self.move_to_orientation_callback)
        self.srv_update_obstacles = self.create_service(
            moveit_testing_interfaces.srv.UpdateObstacles,
            'update_obstacles', self.obstacles_callback)
        self.srv_update_persistent_obstacles = self.create_service(
            moveit_testing_interfaces.srv.UpdateObstacles,
            'update_persistent_obstacles', self.persistent_obstacles_callback)
        self.srv_update_attached_obstacles = self.create_service(
            moveit_testing_interfaces.srv.UpdateAttachedObstacles,
            'update_attached_obstacles', self.attached_obstacles_callback)


        # Initialize API class
        config = MoveConfig()
        config.base_frame_id = 'panda_link0'
        config.workspace_min_corner = geometry_msgs.msg.Vector3(
            x=-1.0,
            y=-1.0,
            z=-1.0
        )
        config.workspace_max_corner = geometry_msgs.msg.Vector3(
            x=1.0,
            y=1.0,
            z=1.0
        )
        config.tolerance = 0.01
        config.group_name = 'panda_arm'

        self.home_pose = geometry_msgs.msg.Pose()

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
            0.035,                  # panda_finger_joint1
            0.035,                  # panda_finger_joint2
        ]

        self.moveit = MoveIt(self, config)

        self.goal_pose = geometry_msgs.msg.Pose()
        self.plan = None

        self.state = State.IDLE

        self.get_logger().info("moveit_interface_tester node started")

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

        # State machine

        if self.state == State.MOVE_TO_HOME_START:

            if self.moveit.planning:
                self.state = State.MOVE_TO_HOME_WAIT
            else:
                self.moveit.move_to_home()

        elif self.state == State.MOVE_TO_HOME_WAIT:
            if not self.moveit.busy:
                self.state = State.IDLE

        elif self.state == State.PLAN_TO_POSE_START:

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
        self.goal_pose = geometry_msgs.msg.Pose()
        self.goal_pose.position = request.position

        self.state = State.PLAN_TO_POSITION_START

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
        ros2 service call /update_obstacles moveit_testing_interfaces/srv/UpdateObstacles
            "{position: {x: 0.5, y: 0.5, z: 1.0}, length: 0.5, width: 0.25, height: 2.0,
            id: 'MyBox', delete_obstacle: false}"

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

    def persistent_obstacles_callback(self, request, response):
        """
        Call /update_persistent_obstacles (moveit_testing_interfaces/srv/UpdateObstacles) service.

        Store obstacle position, dimensions, id and delete flag value input by the user

        Example call:
        ros2 service call /update_persistent_obstacles moveit_testing_interfaces/srv/UpdateObstacles
            "{position: {x: 0.75, y: 0.5, z: 0.0}, length: 1.0, width: 0.25, height: 4.0,
            id: 'wall1', delete_obstacle: false}"

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

        self.moveit.add_persistent_obstacle(obstacle, delete=request.delete_obstacle)

        return response

    def attached_obstacles_callback(self, request, response):
        """
        Call /update_persistent_obstacles (moveit_testing_interfaces/srv/UpdateAttachedObstacles) service.

        Store obstacle position, dimensions, ids and delete flag value input by the user

        Example call:
        ros2 service call /update_attached_obstacles moveit_testing_interfaces/srv/UpdateAttachedObstacles
            "{position: {x: 0.75, y: 0.5, z: 0.0}, length: 1.0, width: 0.25, height: 4.0,
            id: 'gripping', delete_obstacle: false}"

        Args:
            request (UpdateAttachedObstacles): obstacle information

            response (EmptyResponse): no data

        Returns
        -------
            response (EmptyResponse): no data

        """
        attached_obstacle = moveit_msgs.msg.AttachedCollisionObject()
        attached_obstacle.link_name = request.link_name
        attached_obstacle.obstacle.id = request.id

        pose = geometry_msgs.msg.Pose()
        pose.position = request.position
        attached_obstacle.obstacle.primitive_poses = [pose]

        shape = shape_msgs.msg.SolidPrimitive()
        shape.type = request.type  # Box
        shape.dimensions = [request.length, request.width, request.height]
        attached_obstacle.obstacle.primitives = [shape]

        attached_obstacle.obstacle.header.frame_id = self.moveit.config.base_frame_id

        self.moveit.add_persistent_obstacle(attached_obstacle.obstacle, delete=request.delete_obstacle)

        return response

def simple_move_entry(args=None):
    rclpy.init(args=args)
    simple_move = SimpleMove()
    rclpy.spin(simple_move)
    rclpy.shutdown()
