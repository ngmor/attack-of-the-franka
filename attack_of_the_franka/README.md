# Attack of the Frankas Package

## Description of Package
Contains the nodes and launchfiles necessary to run the vision and motion nodes to recognize blocks of different colors and generate paths to hit blocks of a certain color.

### Quickstart Guide of Useful Commands:
#### Running Full Workflow:
`ros2 launch attack_of_the_franka attack_of_the_franka.launch.py`
    
#### Run Only Camera Functionality:
`ros2 launch attack_of_the_franka realsense.launch.py`

#### Run Only Robot Movement Related Programs:
`ros2 launch attack_of_the_franka robot.launch.py`

#### To Pick Up the Lightsaber from the Sheath:
`ros2 service call /pickup_lightsaber std_srvs/srv/Empty`

#### To Start Attacking Enemies
`ros2 service call /look_for_enemy std_srvs/srv/Empty`

#### Move to Home Position
`ros2 service call /move_to_home std_srvs/srv/Empty`

### List of all nodes and launchfiles and what they do:
#### Package Name: attack_of_the_franka
Type: ament_python

#### Nodes:
- `camera_processor`
    - Performs image processing for ally and enemy detection based on color.
    - Gets workspace area transforms and the robot transform
- `common.py`
    - A common library for functions/values used by all nodes
- `robot_control`
    - Runs the state machine related to motion and interacts with moveit_interface API to plan and attack enemies. It processes camera information to locate allies and enemies and provides helpful services to do a variety of tasks with the lightsaber and blocks interaction. 

#### Launchfiles:
- `realsense.launch.py` 
    - Launches the nodes needed to read in information from the RealSense camera and AprilTags to recognize the table and block locations relative to the robot base
- `robot.launch.py`
    - Launches the `robot_control` node and other programs to view the RVIZ simulation of the Franka robot
- `attach_of_the_franka.launch.py`
    - combines the above two launch files to set up all the nodes needed to attack enemies

### Helpful Services:
 - `move_to_home` (std_srvs.srv.Empty): move the robot to a predetermined home position
 - `pickup_lightsaber` (std_srvs.srv.Empty): command the robot to follow waypoints to pick up the fixed lightsaber with the end-effector
 - `look_for_enemy` (std_srvs.srv.Empty): check for any enemies detected in the planning scene and begin to calculate how to attack if possible
 - `gripper_open` (std_srvs.srv.Empty): move the robot end-effector position to open
 - `gripper_close` (std_srvs.srv.Empty): move the robot end-effector position to close
 - `gripper_grasp` (std_srvs.srv.Empty): move the robot end-effector together until it's grasping an object between them
 - `waypoints` (std_srvs.srv.Empty): command the robot to move to a predetermined waypoint
 - `move_to_pose` (moveit_testing_interfaces/srv/MoveToPose): move robot to specific position and orientation
 - `joint_waypoint` (std_srvs.srv.Empty): command specific joint positions
 - `move_to_position` (moveit_testing_interfaces/srv/MoveToPosition): move robot to specific position
 - `move_to_orientation` (moveit_testing_interfaces/srv/MoveToOrientation): move robot to specific orientation
 - `update_obstacles` (moveit_testing_interfaces/srv/UpdateObstacles): add obstacles to scene
 - `update_persistent_obstacles` (moveit_testing_interfaces.srv.UpdateObstacles): add obstacle to scene permanently
 - `update_attached_obstacles` (moveit_testing_interfaces.srv.UpdateAttachedObstacles): add obstacle attached to robot links
 - `home_waypoint` (std_srvs.srv.Empty): plan to a hard coded home value in the callback
 - `add_walls` (std_srvs.srv.Empty): add walls and ceiling collision objects to the planning scene
 - `remove_separate_lightsaber` (std_srvs.srv.Empty): remove the lightsaber as separate collision object
 - `add_separate_lightsaber` (std_srvs.srv.Empty): add lightsaber as a separate collision object
 - `remove_attached_lightsaber` (std_srvs.srv.Empty): remove the lightsaber attached to the end-effector in the planning scene in RVIZ
 - `add_attached_lightsaber` (std_srvs.srv.Empty): add the lightsaber attached to the end-effector to the planning scene as an attached collision object.
 - `reset_allies` (std_srvs.srv.Empty): update the position of the allies in the planning scene to current