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
MoveIt Python API.

Description: This API is used to interact with MoveIt to add obstacles to the environment,
             plan a trajectory avoiding any present obstacles, and execute the trajectory that was
             planned. Functions are available for users to input goal poses for the end-effector
             that include orientation and/or position. Error codes are returned to indicate invalid
             usage/behavior of the API functions.

Parameters passed into the API:
    node:   pass in the node that calls the functions contained in this API
    config: the custom parameters contained within the MoveConfig() class that must
            be specified by the user

Publishers:
    planning_scene (moveit_msgs/msg/PlanningScene): publish obstacles
Subscribers:
    joint_states (sensor_msgs/msg/JointState): gets joint states of each joint
Clients:
    compute_ik (moveit_msgs/srv/GetPositionIK): compute inverse kinematics
    move_action (moveit_msgs/action/MoveGroup): get plan message
    execute_trajectory (moveit_msgs/action/ExecuteTrajectory): get execute message
    get_planning_scene (moveit_msgs/srv/GetPlanningScene): get plan scene message

Authors:
    Megan Sindelar
    Nick Morales
    Sushma Chandra
    Vaishnavi Dornadula

Last Updated: November 10th, 2022
"""

# Good reference:
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html

# Code standards for this API:
# 1. For any class variables that should be internal to the class (private), prepend
#    a "_" to the variable name
# 2. For any class variables that are accessible outside of the class (public), do not
#    prepend anything to the variable name.
# 3. Local variables inside functions need not have a prepended "_" because they're not accessible
#    anyway.
# 4. This is a Python module, so anything we create outside of the class (ex: Enums)
#    are also available if anyone imports the module. Same rules for public vs. private:
#    If something should not be externally accessible, prepend a "_".
# 5. Default to making items (class variables and otherwise) private, unless you have an
#    explicit reason for them to be public.
# 6. Any class variables that are read within the class MUST be initialized in the class
#    constructor.

from enum import Enum, auto
import copy
from rclpy.action import ActionClient
from rclpy.task import Future
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.action
import moveit_msgs.srv
import sensor_msgs.msg
import std_msgs.msg
import shape_msgs.msg
import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# TODO - change to use SMACH?
class _State(Enum):
    """Top level states for main state machine."""

    IDLE = auto(),
    PLANNING = auto(),
    EXECUTING = auto(),

class _PlanState(Enum):
    """Planning states, part of main state machine."""

    IDLE = auto(),
    START_IK_COMPUTE = auto()
    GOAL_IK_COMPUTE = auto(),
    WAIT_FOR_READY = auto(),
    WAIT_FOR_PLAN_ACK = auto(),
    WAIT_FOR_PLAN_RESULT = auto(),


class _ExecState(Enum):
    """Execute states, part of main state machine."""

    IDLE = auto(),
    WAIT_FOR_READY = auto(),
    WAIT_FOR_EXEC_ACK = auto(),
    WAIT_FOR_EXEC_COMPLETE = auto(),


class _ObstacleState(Enum):
    """Obstacle states for obstacle state machine."""

    IDLE = auto(),
    WAIT_FOR_READY = auto(),
    PUBLISH = auto()


class MoveItApiErrors(Enum):
    """API Error Codes."""

    NO_ERROR = 0,
    NOT_IN_IDLE_STATE = 1,
    IK_TIMEOUT = 2,
    IK_RESULT_ERROR = 3,
    PLAN_ERROR = 4,
    EXEC_ERROR = 5,
    CONTROL_ERROR = 6,
    EMPTY_JOINT_HOME_POSITIONS = 20,


class MoveConfig():
    """Configuration variables for API that must be user specified."""

    base_frame_id = ''
    ee_frame_id = ''
    workspace_min_corner = geometry_msgs.msg.Vector3()
    workspace_max_corner = geometry_msgs.msg.Vector3()
    pipeline_id = 'move_group'
    group_name = ''
    ik_timeout_sec = 5
    num_planning_attempts = 10
    allowed_planning_time = 5.0
    max_velocity_scaling_factor = 0.1
    max_acceleration_scaling_factor = 0.1
    tolerance = 0.001 #0.1
    home_joint_positions = []  # Must be in the same order as in the joint state message
                               # TODO - improve? also input joint names?


class MoveIt():
    """
    MoveIt Python API.

    Contains the necessary variables and functions to take in parameters
    such as end effector position/orientation and plan and/or execute that trajectory.
    Users are also able to place objects in the planning scene.

    Publishers:
        planning_scene (moveit_msgs/msg/PlanningScene): publish obstacles
    Subscribers:
        joint_states (sensor_msgs/msg/JointState): gets joint states of each joint
    Clients:
        compute_ik (moveit_msgs/srv/GetPositionIK): compute inverse kinematics
        move_action (moveit_msgs/action/MoveGroup): get plan message
        execute_trajectory (moveit_msgs/action/ExecuteTrajectory): get execute message
        get_planning_scene (moveit_msgs/srv/GetPlanningScene): get plan scene message

    Parameters passed into the API:
        node:   pass in the node that calls the functions contained in this API
        config: the custom parameters contained within the MoveConfig() class that must
                be specified by the user
    """

    def __init__(self, node, config):
        """Class constructor."""
        # Create necessary clients and other ROS items on input node
        # TODO - do we have to handle namespaces or will that just work?
        self._node = node
        self.config = config
        self._compute_ik_client = self._node.create_client(
            moveit_msgs.srv.GetPositionIK, 'compute_ik')
        self._move_action_client = ActionClient(
            self._node, moveit_msgs.action.MoveGroup, 'move_action')
        self._exec_traj_client = ActionClient(
            self._node, moveit_msgs.action.ExecuteTrajectory, 'execute_trajectory')
        self._obstacle_pub = self._node.create_publisher(
            moveit_msgs.msg.PlanningScene, 'planning_scene', 10)
        self.obstacle_client = self._node.create_client(
            moveit_msgs.srv.GetPlanningScene, 'get_planning_scene')
        self.sub_joint_states = self._node.create_subscription(
            sensor_msgs.msg.JointState, '/joint_states', self._sub_joint_state_callback, 10)
        self.pub_planning_scene = self._node.create_publisher(moveit_msgs.msg.PlanningScene, '/planning_scene', 10)

        # TODO - other initialization that needs to happen
        self._plan_future = Future()
        self._ik_future = Future()
        self._plan_result_future = Future()
        self._ik_result = None

        self._state = _State.IDLE
        self._state_last = None
        self._obs_state = _ObstacleState.IDLE
        self._obs_state_last = None
        self._plan_state = _PlanState.GOAL_IK_COMPUTE
        self._plan_state_last = None
        self._exec_state = _ExecState.WAIT_FOR_READY
        self._exec_state_last = None
        self._exec_future = Future()
        self._exec_result_future = Future()
        self.movement_result = moveit_msgs.action.ExecuteTrajectory.Result()

        self._move_to_home = False
        self._waypoint = False
        self._plan_and_execute = False
        self._plan = None
        self._joint_states = sensor_msgs.msg.JointState()
        self._start_joint_states = sensor_msgs.msg.JointState()
        self._goal_joint_states = sensor_msgs.msg.JointState()
        self._goal_pose = geometry_msgs.msg.Pose()
        self._start_pose = geometry_msgs.msg.Pose()
        self._error = MoveItApiErrors.NO_ERROR

        self._planning_scene = moveit_msgs.msg.PlanningScene()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)
        self._ik_start_sec = 0

        self._persistent_obstacles = []
        self._attached_obstacles = []
        self._obs_list = []

        self._excecution_complete = 0

        self.check_obstacles = 0

        self.plan_idle = False


        # # ----------------- Sample Collision Object -------------------- #
        # obstacle_pose1 = geometry_msgs.msg.Pose()
        # obstacle_shape1 = shape_msgs.msg.SolidPrimitive()
        # self.sample_attached_collision = moveit_msgs.msg.AttachedCollisionObject()
        # self.sample_attached_collision.link_name = 'panda_hand_tcp'
        # self.sample_attached_collision.object.id = 'saber'

        # obstacle_pose1.position.x = 0.1
        # obstacle_pose1.position.y = 0.1
        # obstacle_pose1.position.z = 0.3
        # obstacle_pose1.orientation.y = -1.0
        # obstacle_pose1.orientation.w = 1.0
        # self.sample_attached_collision.object.primitive_poses = [obstacle_pose1]
        
        # obstacle_shape1.type = 3
        # obstacle_shape1.dimensions = [0.6, 0.05, 0.2]
        # self.sample_attached_collision.object.primitives = [obstacle_shape1]
        # self.sample_attached_collision.object.header.frame_id = 'panda_hand_tcp'
        # self.sample_attached_collision.object.header.stamp = self._node.get_clock().now().to_msg()
        # self.sample_attached_collision.object.operation = self.sample_attached_collision.object.ADD
        # # has a detach pose element that is important for end-effector grasping
        # # has weight element to specify for lightsaber down the line
        # attached_object = moveit_msgs.msg.PlanningScene()
        # attached_object.name = 'sample'
        # attached_object.robot_model_name = self.config.base_frame_id
        # #attached_object.robot_state.attached_collision_objects = [self.sample_attached_collision]
        # attached_object.robot_state.attached_collision_objects.append(self.sample_attached_collision)
        # #attached_object.world.collision_objects.append(self.sample_attached_collision.object)
        # self._obstacle_pub.publish(attached_object)
        # self._node.get_logger().info("published")
        # # -------------------------------------------------------------- #

    def handle(self):
        """
        External function to cyclically handle MoveIt interactions.

        This must be called in a timer callback.

        Moves between IDLE, PLANNING, EXECUTING, and OBSTACLE states to keep track of which
        related MoveIt Interation is being affected

        Args:
            none

        Returns
        -------
            no returns

        """
        self._get_transform()

        new_state = self._state != self._state_last

        if new_state:
            # TODO - change to debug level
            self._node.get_logger().info(
                f"MoveIt main sequence changed to {self._state.name}")
            self._state_last = self._state

        # reset states to prevent init bugs
        if self._state != _State.PLANNING:
            self._plan_state = _PlanState.IDLE
            self._move_to_home = False
            self._waypoint = False
        
        if self._state != _State.EXECUTING:
            self._exec_state = _ExecState.IDLE

        # State Machine
        if self._state == _State.IDLE:
            pass

        elif self._state == _State.PLANNING:
            self._excecution_complete = 0
            if new_state:
                # TODO - this is getting a little messy, might be a good idea to rework it if
                # possible

                # Skip IK compute if we're moving to home
                if self._move_to_home:
                    self._plan_state = _PlanState.WAIT_FOR_READY
                elif self._waypoint:
                    self._plan_state = _PlanState.WAIT_FOR_READY
                # If we specify a custom start state, we need to start with
                # computing inverse kinematics of that start state
                elif self._start_pose is not None:
                    self._plan_state = _PlanState.START_IK_COMPUTE
                # No custom start state, just start from where we are. Only have to compute goal
                # inverse kinematics
                else:
                    self._plan_state = _PlanState.GOAL_IK_COMPUTE

            # Execute planning subsequence
            self._plan_sequence()

        elif self._state == _State.EXECUTING:
            if new_state:
                self._exec_state = _ExecState.WAIT_FOR_READY

            self._exec_sequence()

        self._obs_sequence()

        # Set externally accessible status variables
        self.busy = not (self._state == _State.IDLE)
        self.planning = self._state == _State.PLANNING
        self.plan_idle = self._plan_state == _PlanState.IDLE
        self.executing = self._state == _State.EXECUTING
        self.planning_and_executing = (self.planning or self.executing) and self._plan_and_execute

    def _get_transform(self):
        """
        Get robot transformation from base frame to end-effector frame.

        Args:
            none

        Returns
        -------
            no returns

        """
        try:
            self._base_endeffector = self._tf_buffer.lookup_transform(
                self.config.base_frame_id,
                self.config.ee_frame_id,
                rclpy.time.Time())
        except Exception:
            pass

    def _sub_joint_state_callback(self, msg):
        """
        To get joint states of the robot.

        Callback function for /joint_states (sensor_msgs/msg/JointState) topic

        Args:
            msg: the data from the topic /joint_states

        Returns
        -------
            no returns

        """
        self._joint_states = msg
      #  self._node.get_logger().info(f'start: {self._joint_states}')

    def _obs_sequence(self):
        """
        Publish obstacles into the planning scene by running through Obstacle state machine.

        Publishes:
            planning_scene (moveit_msgs/msg/PlanningScene): publish obstacles

        Args:
            none

        Returns
        -------
            no returns

        """
        new_state = self._obs_state != self._obs_state_last

        if new_state:
            # TODO - change to debug level
           # self._node.get_logger().info(
            #    f"MoveIt object sequence changed to {self._obs_state.name}")
            self._obs_state_last = self._obs_state

        if self._obs_state == _ObstacleState.IDLE:
            pass

            # test = moveit_msgs.srv.ApplyPlanningScene(diff_scene)
            # test1 = 

        elif self._obs_state == _ObstacleState.WAIT_FOR_READY:
            if new_state:
                self.obstacle_future = self.obstacle_client.call_async(
                    moveit_msgs.srv.GetPlanningScene.Request())
            if self.obstacle_future.done():
                self._planning_scene = copy.deepcopy(self.obstacle_future.result().scene)
                if self._attached_obstacles:
                    # self._node.get_logger().info(
                    # f"Attached obstacles message {self._planning_scene.robot_state}")
                    # self._node.get_logger().info(
                    # f"Length of attached obstacles message {len(self._planning_scene.robot_state.attached_collision_objects)}")
                    # self._node.get_logger().info(
                    # f"First element of attached obstacles message {self._planning_scene.robot_state.attached_collision_objects[0]}")
                    # self._node.get_logger().info(
                    # f"Second element of attached obstacles message {self._planning_scene.robot_state.attached_collision_objects[1]}")
                    # add to planning scene's attached collision object list
                    self._planning_scene.robot_state.attached_collision_objects = self._attached_obstacles
                    # attached_obstacle = moveit_msgs.msg.CollisionObject()
                    # attached_obstacle = self._attached_obstacles
                    # self._planning_scene.world.collision_objects.append(attached_obstacle)
                    self._planning_scene.is_diff = True
                # process info
                self._update_collision_object()
                self._obs_state = _ObstacleState.PUBLISH

        elif self._obs_state == _ObstacleState.PUBLISH:
            if self._obs_state is not _State.PLANNING:
                self._obstacle_pub.publish(self._planning_scene)
                self._obs_state = _ObstacleState.IDLE

    def _plan_sequence(self):
        """
        Run plan subsequence.

        Called by the handle function while in the planning state. Checks completion
        of async functions to move between plan states and saves the plan output into
        internal variable, self._plan

        Args:
            none

        Returns
        -------
            no returns

        """
        new_plan_state = self._plan_state != self._plan_state_last

        if new_plan_state:
            # TODO - change to debug level
            self._node.get_logger().info(
                f"MoveIt plan sequence changed to {self._plan_state.name}")
            self._plan_state_last = self._plan_state

        # Plan state machine
        if self._plan_state == _PlanState.START_IK_COMPUTE:
            if new_plan_state:
                # Trigger compute IK once at start of state
                self._node.get_logger().info(f"start pose: {self._start_pose}")
                self._ik_request(self._joint_states, self._start_pose)
                
            # wait for IK to finish
            if self._ik_future.done():

                if self._ik_future.result().error_code.val != 1:
                    self._error = MoveItApiErrors.IK_RESULT_ERROR
                    self._state = _State.IDLE
                    self._node.get_logger().error(
                        "START_IK_COMPUTE returned an error code other than SUCCESS(1)")
                    self._node.get_logger().error(
                        f'START_IK_COMPUTE error code: {self._ik_future.result().error_code.val}')
                else:
                    self._start_joint_states = \
                        copy.deepcopy(self._ik_future.result().solution.joint_state)
                    self._error = MoveItApiErrors.NO_ERROR
                    self._plan_state = _PlanState.GOAL_IK_COMPUTE

            # Timeout
            elif ((self._node.get_clock().now().seconds_nanoseconds()[0] - self._ik_start_sec)
                    > (2*self.config.ik_timeout_sec)):
                self._error = MoveItApiErrors.IK_TIMEOUT
                self._state = _State.IDLE
                self._node.get_logger().error("START_IK_COMPUTE Timed out")

        elif self._plan_state == _PlanState.GOAL_IK_COMPUTE:
            if new_plan_state:
                # Trigger compute IK once at start of state
                # self._node.get_logger().info(f'goal pose: {type(self._goal_pose)}')
                self._ik_request(self._start_joint_states, self._goal_pose)
            # wait for IK to finish
            if self._ik_future.done():

                if self._ik_future.result().error_code.val != 1:
                    self._error = MoveItApiErrors.IK_RESULT_ERROR
                    self._state = _State.IDLE
                    self._node.get_logger().error(
                        "GOAL_IK_COMPUTE returned an error code other than SUCCESS(1)")
                    self._node.get_logger().error(
                        f'GOAL_IK_COMPUTE error code: {self._ik_future.result().error_code.val}')
                else:
                    self._goal_ik_result = copy.deepcopy(self._ik_future.result())
                    self._goal_joint_states = self._goal_ik_result.solution.joint_state
                    self._error = MoveItApiErrors.NO_ERROR
                    self._plan_state = _PlanState.WAIT_FOR_READY

            # Timeout
            elif ((self._node.get_clock().now().seconds_nanoseconds()[0] - self._ik_start_sec)
                    > (2*self.config.ik_timeout_sec)):
                self._error = MoveItApiErrors.IK_TIMEOUT
                self._state = _State.IDLE
                self._node.get_logger().error("GOAL_IK_COMPUTE Timed out")

        elif self._plan_state == _PlanState.WAIT_FOR_READY:
            # if self._move_action_client.server_is_ready(), then move to start
            if self._move_action_client.server_is_ready():
                # send the goal plan through the move action client
                self._node.get_logger().info("wat")
                self._plan_future = \
                    self._move_action_client.send_goal_async(self._assemble_plan_message(self._attached_obstacles))
                self._plan_state = _PlanState.WAIT_FOR_PLAN_ACK
        elif self._plan_state == _PlanState.WAIT_FOR_PLAN_ACK:
            # once that future object.done() returns true,
            # to get the result
            if self._plan_future.done():
                self._plan_result_future = self._plan_future.result().get_result_async()

                # go to complete state
                self._plan_state = _PlanState.WAIT_FOR_PLAN_RESULT

        elif self._plan_state == _PlanState.WAIT_FOR_PLAN_RESULT:
            # if self._plan_result_future.done() returns true, then done planning
            if self._plan_result_future.done():
                # Save the result and into a plan variable
                self._plan = copy.deepcopy(self._plan_result_future.result().result)

                if self._plan.error_code.val != 1:
                    self._error = MoveItApiErrors.PLAN_ERROR
                    self._node.get_logger().error(
                        "Plan result returned an error code other than SUCCESS(1)")
                    self._node.get_logger().error(f'Plan error code: {self._plan.error_code.val}')
                    # if self._plan.error_code.val == -4:
                    #     self._start_joint_states = self._joint_states
                    #     self._error = MoveItApiErrors.CONTROL_ERROR
                    #     self._node.get_logger().error(f"Control error {self._error}")
                    #     # self._plan_future = \
                    #     # self._move_action_client.send_goal_async(self._assemble_plan_message(self._attached_obstacles))
                    #     # if self._plan_future.done():
                    #     #     self._error = MoveItApiErrors.NO_ERROR
                    #     self._start_joint_states = self._joint_states
                    #     self._node.get_logger().info(f'joint states:: {self._joint_states}')

                    #     # if self._start_pose is None:
                    #     #     self._plan_state = _PlanState.START_IK_COMPUTE
                    #     # else:
                    #     #     self._plan_state = _PlanState.WAIT_FOR_READY
                    #     self.reset_robot_state()
                    #     if self._move_to_home:
                    #         self.move_to_home()
                    #     # self._state = _State.PLANNING
                    #     self._node.get_logger().error(f"State {self._state}")
                    # else:
                    #     self._state = _State.IDLE
                else:
                    self._error = MoveItApiErrors.NO_ERROR
                    self._excecution_complete = 1
                # go back to IDLE
            self._state = _State.IDLE
                
            self._node.get_logger().error(f"State {self._state}")

    def _exec_sequence(self):
        """
        Run execute subsequence.

        Called by the handle function while in the execute state. Checks competion
        of async functions to move between execute states and saves the plan output into
        internal variable, self.movement_result

        Args:
            none

        Returns
        -------
            no returns

        """
        new_exec_state = self._exec_state != self._exec_state_last

        if new_exec_state:
            # TODO - change to debug level
            self._node.get_logger().info(
                f"MoveIt exec sequence changed to {self._exec_state.name}")
            self._exec_state_last = self._exec_state

        # Exec state machine
        if self._exec_state == _ExecState.WAIT_FOR_READY:
            # Wait for the action server to be ready
            if self._exec_traj_client.server_is_ready():
                # Execute the plan
                self._exec_future = \
                    self._exec_traj_client.send_goal_async(self._assemble_exec_message())
                self._exec_state = _ExecState.WAIT_FOR_EXEC_ACK

        elif self._exec_state == _ExecState.WAIT_FOR_EXEC_ACK:
            # wait for robot to execute the plan
            if self._exec_future.done():
                self._exec_result_future = self._exec_future.result().get_result_async()

                self._exec_state = _ExecState.WAIT_FOR_EXEC_COMPLETE

        elif self._exec_state == _ExecState.WAIT_FOR_EXEC_COMPLETE:
            # Wait for move to complete
            if self._exec_result_future.done():

                # Store movement result
                self.movement_result = copy.deepcopy(self._exec_result_future.result().result)

                if self.movement_result.error_code.val != 1:
                    self._error = MoveItApiErrors.EXEC_ERROR
                    self._node.get_logger().error("Execute result returned an" +
                                                  "error code other than SUCCESS(1)")
                    self._node.get_logger().error(
                        f'Execute error code: {self.movement_result.error_code.val}')
                else:
                    self._error = MoveItApiErrors.NO_ERROR

                # return back to IDLE state to get ready for next target
                self._state = _State.IDLE

    def _plan_traj(self, goal_pose, start_pose=None, execute=False):
        """
        Start the trajectory planning process.

        Internal function is called by externally accessible functions where the user can
        specify position and or orientation of the end effector.

        Args:
            goal_pose:  a geometry Pose and/or Orientation variable that specifies where
            start_pose: (optional) if none is specified, then use the current configuration
                        if start pose is provided, that input must be of type geometry message
                        Pose with x, y, z
            execute:    (optional) defaults to false, indicating that it will only plan, not plan
                        and execute

        Returns
        -------
            self._error: an error code indicating success or specified failure of
                         type enum ERROR_CODES

        """
        # makes sure that we are IDLE before planning a task
        if self._state != _State.IDLE:
            # return some form of error, we are busy with another task
            self._error = MoveItApiErrors.NOT_IN_IDLE_STATE
            return self._error

        # If no start state is specified, default to current position
        if start_pose is None:
            self._start_joint_states = self._joint_states

        self._start_pose = start_pose
        self._goal_pose = goal_pose

        # Signal whether or not we will execute this plan immediately
        self._plan_and_execute = execute

        self._state = _State.PLANNING

        # return some sort of message indicating success or failure
        self._error = MoveItApiErrors.NO_ERROR
        return self._error

    def plan_traj_to_pose(self, goal_pose, start_pose=None, execute=False):
        """
        Plan a trajectory with the full input pose.

        Args:
            goal_pose:  a geometry Pose and/or Orientation variable that specifies where
            start_pose: (optional) if none is specified, then use the current configuration
                        if start pose is provided, that input must be of type geometry message
                        Pose with x, y, z
            execute:    (optional) defaults to false, indicating that it will only plan, not plan
                        and execute

        Returns
        -------
            error of type enum ERROR_CODES provided from _plan_traj function

        """
        # TODO any further logic to set the input goal pose for _plan_traj function
        return self._plan_traj(goal_pose=goal_pose, start_pose=start_pose, execute=execute)

    def plan_traj_to_position(self, goal_position, start_pose=None, execute=False):
        """
        Plan a trajectory with only an input position specified.

        Args:
            goal_pose:  a geometry Pose and/or Orientation variable that specifies where
            start_pose: (optional) if none is specified, then use the current configuration
                        if start pose is provided, that input must be of type geometry message
                        Pose with x, y, z
            execute:    (optional) defaults to false, indicating that it will only plan, not plan
                        and execute

        Returns
        -------
            error of type enum ERROR_CODES provided from _plan_traj function

        """
        goal_pose = geometry_msgs.msg.Pose()
        goal_pose.position = copy.deepcopy(goal_position)

        # TODO - doesn't seem to work the very first time
        if start_pose is None:
            # setting the orientaion to the current orientation
            goal_pose.orientation = copy.deepcopy(self._base_endeffector.transform.rotation)
        else:
            goal_pose.orientation = start_pose.orientation
        # TODO any further logic to set the input goal pose for _plan_traj function
        return self._plan_traj(goal_pose=goal_pose, start_pose=start_pose, execute=execute)

    def plan_traj_to_orientation(self, goal_orientation, start_pose=None, execute=False):
        """
        Plan a trajectory with only an input orientation specified.

        Args:
            goal_pose:  a geometry Pose and/or Orientation variable that specifies where
            start_pose: (optional) if none is specified, then use the current configuration
                        if start pose is provided, that input must be of type geometry message
                        Pose with x, y, z
            execute:    (optional) defaults to false, indicating that it will only plan, not plan
                        and execute

        Returns
        -------
            error of type enum ERROR_CODES provided from _plan_traj function

        """
        goal_pose = geometry_msgs.msg.Pose()
        # goal_pose.orientation = goal_orientation
        goal_pose.orientation = copy.deepcopy(goal_orientation)

        if start_pose is None:
            # setting the position to the current position
            goal_pose.position.x = self._base_endeffector.transform.translation.x
            goal_pose.position.y = self._base_endeffector.transform.translation.y
            goal_pose.position.z = self._base_endeffector.transform.translation.z
        else:
            goal_pose.position = start_pose.position

        # TODO any further logic to set the input goal pose for _plan_traj function
        return self._plan_traj(goal_pose=goal_pose, start_pose=start_pose, execute=execute)

    def exec_traj(self, plan=None):
        """
        Execute a trajectory.

        Args:
            plan: If the plan argument is passed in, use that plan. Otherwise use
                  the internally saved plan.

        Returns
        -------
            none

        """
        if self._state != _State.IDLE:
            # TODO - return some form of error, we are busy
            return

        if plan is not None:
            # if a plan is passed in, use it instead of the currently stored plan.
            # TODO - error checking to see if this is a valid plan?
            self._plan = copy.deepcopy(plan)
        elif self._plan is None:
            # TODO - return some form of error, we don't have a current plan and
            # one wasn't passed in
            return

        self._state = _State.EXECUTING
        # TODO - return some sort of message indicating success or failure
        return

    def move_to_home(self):
        """
        Plan and execute a move to the home position

        TODO - finish docstring
        """

        # makes sure that we are IDLE before planning a task
        if self._state != _State.IDLE:
            # return some form of error, we are busy with another task
            self._error = MoveItApiErrors.NOT_IN_IDLE_STATE
            return self._error
        # Check if joint position list is empty
        elif not self.config.home_joint_positions:
            self._error = MoveItApiErrors.EMPTY_JOINT_HOME_POSITIONS
            return self._error

        self._start_joint_states = self._joint_states
        self._start_pose = None

        self._goal_joint_states = copy.deepcopy(self._joint_states)
        self._goal_joint_states.position = copy.deepcopy(self.config.home_joint_positions)

        # Execute immediately after planning
        self._plan_and_execute = True

        # Skip IK
        # TODO - may want a more general way of doing this if we want to skip IK for other reasons?
        self._move_to_home = True

        self._state = _State.PLANNING

        # return some sort of message indicating success or failure
        self._error = MoveItApiErrors.NO_ERROR
        return self._error

    def get_plan(self):
        """
        Return the currently stored plan.

        We can use this if we want to calculate multiple plans at one time and then
        execute them without having to plan in between each one. The higher level
        node would store the plans somewhere and then pass them back into our execute
        trajectory method.

        Args:
            none

        Returns
        -------
            self._plan - the current plan saved from the plan state machine

        """
        return self._plan

    def _ik_request(self, start_joint_states, goal_pose):
        """
        Request the inverse kinematics from the compute_ik service.

        Args:
        ----
            start_joint_states: the start position of the robots provided by the joint
                                state subscriber or the user if specified
            goal_pose:          user provided goal pose from the externally accessible
                                planning function class
        Returns
        -------
            none

        """
        # Build IK message
        ik_message = moveit_msgs.msg.PositionIKRequest()
        ik_message.group_name = self.config.group_name
        ik_message.robot_state.joint_state = copy.deepcopy(start_joint_states)
        # This version uses the starting configuration
        # we need to also have the possibility of using an arbitrary configuration

        ik_message.pose_stamped.header.frame_id = self.config.base_frame_id
        ik_message.pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        ik_message.pose_stamped.pose = copy.deepcopy(goal_pose)
        ik_message.timeout.sec = self.config.ik_timeout_sec
        self._ik_future = \
            self._compute_ik_client.call_async(moveit_msgs.srv.GetPositionIK.Request(
                                               ik_request=ik_message))

        self._ik_start_sec = self._node.get_clock().now().seconds_nanoseconds()[0]

    # def reset_robot_state(self):
    #     assemble_msg = moveit_msgs.action.MoveGroup.Goal()

    #     header = std_msgs.msg.Header()
    #     header.frame_id = self.config.base_frame_id
    #     header.stamp = self._node.get_clock().now().to_msg()

    #     assemble_msg.request.workspace_parameters.header = header
    #     assemble_msg.request.workspace_parameters.min_corner = self.config.workspace_min_corner
    #     assemble_msg.request.workspace_parameters.max_corner = self.config.workspace_max_corner
    #     assemble_msg.request.start_state.joint_state = self._start_joint_states
    #     # assemble_msg.planning_options.planning_scene_diff.is_diff = True
    #     self._node.get_logger().info(f"start joint states: {self._start_joint_states}")
    #     assemble_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
    #     assemble_msg.planning_options.planning_scene_diff.robot_state.joint_state = self._start_joint_states
    #     assemble_msg.request.goal_constraints = \
    #         _joint_states_to_goal_constraints(self._goal_joint_states,
    #                                           self.config.tolerance)

    #     assemble_msg.request.pipeline_id = self.config.pipeline_id
    #     assemble_msg.request.group_name = self.config.group_name
    #     assemble_msg.request.num_planning_attempts = self.config.num_planning_attempts
    #     assemble_msg.request.allowed_planning_time = self.config.allowed_planning_time
    #     assemble_msg.request.max_velocity_scaling_factor = self.config.max_velocity_scaling_factor
    #     assemble_msg.request.max_acceleration_scaling_factor = \
    #         self.config.max_acceleration_scaling_factor

    #     # obstacle avoidance
    #     # assemble_msg.planning_options.planning_scene_diff.world.collision_objects = \
    #     #     self.obstacles.world.collision_objects

    #     assemble_msg.planning_options.plan_only = not self._plan_and_execute
    #     if self._move_action_client.server_is_ready():
    #         # send the goal plan through the move action client
    #         self._node.get_logger().info("wat")
    #         self._plan_future = \
    #             self._move_action_client.send_goal_async(assemble_msg)

    def _assemble_plan_message(self, attached_collision_objects):
        """
        Generate plan request message from start position to goal pose.

        Args:
            none

        Returns
        -------
            assemble_msg: a moveit_msgs.action.MoveGroup.Goal() that can be asyncronously
                          called in the planning state machine to generate a plan

        """
        assemble_msg = moveit_msgs.action.MoveGroup.Goal()

        header = std_msgs.msg.Header()
        header.frame_id = self.config.base_frame_id
        header.stamp = self._node.get_clock().now().to_msg()

        assemble_msg.request.workspace_parameters.header = header
        assemble_msg.request.workspace_parameters.min_corner = self.config.workspace_min_corner
        assemble_msg.request.workspace_parameters.max_corner = self.config.workspace_max_corner
        assemble_msg.request.start_state.joint_state = self._start_joint_states
        # assemble_msg.planning_options.planning_scene_diff.is_diff = True
       # assemble_msg.request.start_state.attached_collision_objects = attached_collision_objects
       # assemble_msg.planning_options.planning_scene_diff.
       # assemble_msg.planning_options.planning_scene_diff.robot_state.attached_collision_objects = attached_collision_objects

        if len(attached_collision_objects) >= 1:
            #acm = moveit_msgs.msg.AllowedCollisionMatrix()
            assemble_msg.request.start_state.is_diff = True
            # assemble_msg.planning_options.planning_scene_diff.allowed_collision_matrix.entry_values.append(('panda_rightfinger', 'gripping', True))
            # assemble_msg.planning_options.planning_scene_diff.allowed_collision_matrix.entry_values.append(('panda_leftfinger', 'gripping', True))
            # assemble_msg.planning_options.planning_scene_diff.allowed_collision_matrix.entry_values.append(('panda_hand_tcp', 'gripping', True))
            # assemble_msg.planning_options.planning_scene_diff.allowed_collision_matrix.entry_values.append(('panda_hand', 'gripping', True))

            collision_obj = moveit_msgs.msg.CollisionObject()
            collision_obj = attached_collision_objects[0].object

            attached_obj = moveit_msgs.msg.AttachedCollisionObject()
            attached_obj = attached_collision_objects[0]
            assemble_msg.planning_options.planning_scene_diff.robot_state.attached_collision_objects.append(attached_obj)

            planning_scene = moveit_msgs.msg.PlanningScene()
            #planning_scene.world.collision_objects.append(collision_obj)
            # planning_scene.allowed_collision_matrix.entry_values.append(('panda_rightfinger', 'gripping', True))
            # planning_scene.allowed_collision_matrix.entry_values.append(('panda_leftfinger', 'gripping', True))
            # planning_scene.allowed_collision_matrix.entry_values.append(('panda_hand_tcp', 'gripping', True))
            # planning_scene.allowed_collision_matrix.entry_values.append(('panda_hand', 'gripping', True))
            planning_scene.robot_state.attached_collision_objects.append(attached_obj)
            planning_scene.is_diff = True

            self.pub_planning_scene.publish(planning_scene)

        # Get from compute ik
        assemble_msg.request.goal_constraints = \
            _joint_states_to_goal_constraints(self._goal_joint_states,
                                              self.config.tolerance)

        assemble_msg.request.pipeline_id = self.config.pipeline_id
        assemble_msg.request.group_name = self.config.group_name
        assemble_msg.request.num_planning_attempts = self.config.num_planning_attempts
        assemble_msg.request.allowed_planning_time = self.config.allowed_planning_time
        assemble_msg.request.max_velocity_scaling_factor = self.config.max_velocity_scaling_factor
        assemble_msg.request.max_acceleration_scaling_factor = \
            self.config.max_acceleration_scaling_factor

        # obstacle avoidance
        # assemble_msg.planning_options.planning_scene_diff.world.collision_objects = \
        #     self.obstacles.world.collision_objects

        assemble_msg.planning_options.plan_only = not self._plan_and_execute
        return assemble_msg

    def _assemble_exec_message(self):
        """
        Generate execute request message from saved plan trajectory.

        Args:
            none

        Returns
        -------
            assemble_msg - variable of type moveit_msgs.action.ExecuteTrajectory.Goal() that
                is populated with the planned trajectory created in the plan state machine

        """
        assemble_msg = moveit_msgs.action.ExecuteTrajectory.Goal()
        assemble_msg.trajectory = self._plan.planned_trajectory

        return assemble_msg

    # id_string will not need to be present when it's not user input
    def _update_collision_object(self):
        """
        Add, update, or delete obstacles (internal).

        Args:
            none

        Returns
        -------
            none

        """
        if not self._delete_obs:
            _obstacle_ID_list = []
            for obstacle in self._planning_scene.world.collision_objects:
                _obstacle_ID_list.append(obstacle.id)
            for obstacle in self._obs_list:
                try:
                    index = _obstacle_ID_list.index(obstacle.id)
                except ValueError:
                    index = -1

                if index == -1:
                    # object isn't in list, do whatever you need  to
                    # ADD
                    self._planning_scene.world.collision_objects.append(copy.deepcopy(obstacle))
                    if self._attached_obstacles:
                        # add to planning scene's attached collision object list
                        self._planning_scene.robot_state.attached_collision_objects = self._attached_obstacles
                else:
                    # object is in list
                    # UPDATE
                    self._planning_scene.world.collision_objects[index] = copy.deepcopy(obstacle)
                    if self._attached_obstacles:
                        # add to planning scene's attached collision object list
                        self._planning_scene.robot_state.attached_collision_objects = self._attached_obstacles
        else:
            for obstacle in self._obs_list:
                for i in range(len(self._planning_scene.world.collision_objects)):
                    if obstacle.id == self._planning_scene.world.collision_objects[i].id:
                        self._planning_scene.world.collision_objects.pop(i)
                        break
        return

    def update_obstacles(self, obstacle_list, delete=False):
        """
        Add, modify, or delete obstacles in the planning scene.

        Args:
            obstacle_list: list of obstacles, each of type moveit_msgs.msg.CollisionObject()
            delete:        a boolean variable to specify whether to delete the obstacles

        Returns
        -------
            none

        """
        if self._obs_state != _ObstacleState.IDLE:
            # TODO - return some form of error, we are busy
            return
        self._obs_state = _ObstacleState.WAIT_FOR_READY
        # if self._persistent_obstacle is not empty, append to obstacle_list
        if self._persistent_obstacles:
            #obstacle_list.extend(self._persistent_obstacles)
            obstacle_list.extend(self._persistent_obstacles)
        self._obs_list = obstacle_list
        self._delete_obs = delete
        # TODO - return some sort of message indicating success or failure
        return

    def update_attached_obstacles(self, attached_object, delete=False):
        """
        Add an attached obstacle 

        Args:
            attached_object: object of type moveit_msgs.msg.AttachedCollisionObject()
                             to add to the Planning Scene
        Returns
        -------
            none
        """
        if delete:
            for i in range(len(self._persistent_obstacles)):
                if attached_object.object.id == self._attached_obstacles[i].object.id:
                    self._attached_obstacles.pop(i)
        else:
            # add object to persistent obstacle list
            #self._persistent_obstacles.append(attached_object.object)
            self._attached_obstacles.append(attached_object)
            # add to planning scene's attached collision object list
            # self._planning_scene.robot_state.attached_collision_objects.append(attached_object)
        self.update_obstacles(self._obs_list, False)
        return

    # def update_persistent_obstacle(self, obstacle, delete=False):
    #     """
    #     Add an obstacle to the scene that doesn't exist physically

    #     Args:
    #         obstacle: object of type moveit_msgs.msg.CollisionObject()
    #                   to add to the Planning Scene
    #     Returns
    #     -------
    #         none
    #     """
    #     if delete:
    #         for i in range(len(self._persistent_obstacles)):
    #             if obstacle.id == self._persistent_obstacles[i].id:
    #                 self._persistent_obstacles.pop(i)
    #                 break
    #     else:
    #         # add object to persistent obstacle list
    #         if self._persistent_obstacles:
    #             self._persistent_obstacles.append(obstacle)
    #         else:
    #             self._persistent_obstacles = obstacle
    #     self.update_obstacles(self._obs_list, False)
    #     return

    def update_persistent_obstacle(self, obstacle, delete=False):
        """
        Add an obstacle to the scene that doesn't exist physically

        Args:
            obstacle: object of type moveit_msgs.msg.CollisionObject()
                      to add to the Planning Scene
        Returns
        -------
            none
        """
        if delete:
            for i in range(len(self._persistent_obstacles)):
                if obstacle.id == self._persistent_obstacles[i].id:
                    self._persistent_obstacles.pop(i)
                    break
        else:
            # add object to persistent obstacle list
            if self._persistent_obstacles:
                self._persistent_obstacles.append(obstacle)
            else:
                self._persistent_obstacles = obstacle
        self.update_obstacles(self._obs_list, False)
        return

    def get_last_error(self):
        """
        Return the last error recorded by the API.

        If last action was successful, this will be NO_ERROR

        Args:
            none

        Returns
        -------
            variable of type Enum MoveItApiErrors indicating error state

        """
        return self._error

    def joint_waypoints(self, waypoint):
        if self._state != _State.IDLE:
            # return some form of error, we are busy with another task
            self._error = MoveItApiErrors.NOT_IN_IDLE_STATE
            return self._error
        # Check if joint position list is empty
        # elif not self.config.home_joint_positions:
        #     self._error = MoveItApiErrors.EMPTY_JOINT_HOME_POSITIONS
        #     return self._error

        self._start_joint_states = self._joint_states
        self._start_pose = None

        self._goal_joint_states = copy.deepcopy(self._joint_states)
        self._goal_joint_states.position = copy.deepcopy(waypoint)

        # Execute immediately after planning
        self._plan_and_execute = False

        # Skip IK
        # TODO - may want a more general way of doing this if we want to skip IK for other reasons?
        #self._move_to_home = True
        self._waypoint = True

        self._state = _State.PLANNING

        # return some sort of message indicating success or failure
        self._error = MoveItApiErrors.NO_ERROR
        return self._error


    def check_planning_scene(self, goal_waypoint):
        
        # self.obstacle_future = self.obstacle_client.call_async(
        #             moveit_msgs.srv.GetPlanningScene.Request())
        # if self.obstacle_future.done():
        #     self._planning_scene = copy.deepcopy(self.obstacle_future.result().scene)
        #     if self._attached_obstacles:
        #         self._node.get_logger().info(
        #         f"Attached obstacles message {self._planning_scene.robot_state}")
        #         self._node.get_logger().info(
        #         f"Length of attached obstacles message {len(self._planning_scene.robot_state.attached_collision_objects)}")
                # self._node.get_logger().info(
                # f"First element of attached obstacles message {self._planning_scene.robot_state.attached_collision_objects[0]}")
                # self._node.get_logger().info(
                # f"Second element of attached obstacles message {self._planning_scene.robot_state.attached_collision_objects[1]}")
        return

def _joint_states_to_goal_constraints(joint_state, tolerance):
    """
    Generate goal constrains given the IK joint states.

    Args:
        joint_state: the joint states given from the results of the inverse kinematics
                    computations
        tolerance:   provided by the configuration input from the user

    Returns
    -------
        goal_constraint: of type moveit_msgs.msg.Constraints to be used in the
                        _assemble_plan_message function's goal contraint parameter

    """
    joint_constraints = []
    for i in range(len(joint_state.name)):
        joint_constraint = moveit_msgs.msg.JointConstraint()

        if i == 7 or i == 8:
            pass
        else:
            joint_constraint.joint_name = joint_state.name[i]
            joint_constraint.position = joint_state.position[i]
        
        # TODO - rework this into an array of tolerances and weights
        joint_constraint.tolerance_above = tolerance
        joint_constraint.tolerance_below = tolerance
        joint_constraint.weight = 1.0

        joint_constraints.append(joint_constraint)

    
    goal_constraint = [moveit_msgs.msg.Constraints(joint_constraints=joint_constraints)]

    return goal_constraint
