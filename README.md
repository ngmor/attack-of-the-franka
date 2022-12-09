# attack-of-the-franka

Team Name: Attack of the Frankas (Group 2)
Team Members:
Nick Morales
Megan Sindelar
Sushma Chandra
Vaishnavi Dornadula
Robot Used: Franka
Github Repository Link: https://github.com/ngmor/attack-of-the-franka

Brief Description of Project:
Our project is to have a robot use a lightsaber to help the Rebellion fight the Empire, where the setup of the workspace is configured by a human. The robot holds the lightsaber in its grippers to knock over members of the Imperial Army, represented by red blocks placed upright on a table near the Franka. Franakin Skywalker will need to avoid blocks representing members of the Rebellion, marked by upright blue blocks, using computer vision to differentiate the blocks.


Usage Instructions:
To clone all necessary repositories, clone this repository into the `src` directory in your workspace root. Then from the workspace root directory, run the command:

`vcs import . < src/attack-of-the-franka/project.repos`

(This assumes you don't change the name of the repository, update the path accordingly if you do.)

If you don't have `vcstool` installed, install it with:

`sudo apt install python3-vcstool`

Summary of Project Capabilities (Goals Achieved): 
Robot recognizes which block it must knock over, picks up its lightsaber, and knocks over the “enemy” blocks without colliding into its “ally” blocks on the attack swing
The robot can pick up the lightsaber from a fixture on the side of the table
The robot is able to avoid multiple allies and knock over multiple enemies.
The robot calculates whether knocking an enemy over will cause the falling block to collide with an ally, and therefore does not swing in that style (i.e. prioritize leaving allies standing over knocking over enemies).
Robot will check all its swing/attack styles before giving up on the attack

High Level Workflow:
Upon launching the attack of the franka launch file, the camera and robot nodes will start running. An rviz simulation of the franka with the walls and ceiling will appear as well as an rviz visualization of the transformations generated by the vision processing. Calling the /pickup_lightsaber service will command the robot to pick up the lightsaber from its sheath and the /look_for_enemies service will set the robot up to begin planning attacks on the red blocks. The state machine will evaluate three attack types, left swing, right swing, and stab, to ensure it doesn’t knock any blue blocks. If it is unable to attack an enemy, it will move on to the next one, until it is done with all enemies that are reachable.

A discussion of the overall system architecture and high level concepts:


Quickstart Guide of Useful Commands:
Running Full Workflow:
ros2 launch attack_of_the_franka attack_of_the_franka.launch.py
    
Run Only Camera Functionality:
    ros2 launch attack_of_the_franka realsense.launch.py

Run Only Robot Movement Related Programs:
    ros2 launch attack_of_the_franka robot.launch.py

To Pick Up the Lightsaber from the Sheath:
    ros2 service call /pickup_lightsaber std_srvs/srv/Empty

To Start Attacking Enemies
    ros2 service call /look_for_enemies std_srvs/srv/Empty

Helpful Services:
move_to_home (std_srvs.srv.Empty): moves the robot to a predetermined home position
gripper_open (std_srvs.srv.Empty): moves the robot's end-effector position to open
gripper_close (std_srvs.srv.Empty): moves the robot's end-effector position to close
gripper_grasp (std_srvs.srv.Empty): moves the robot's end-effector together until               it's grasping an object between them
waypoints (std_srvs.srv.Empty): commands the robot to move to a predetermined waypoint
test_joint_trajectory (std_srvs.srv.Empty): commands robot to a predetermined waypoint using the joint angles
move_to_pose (moveit_testing_interfaces/srv/MoveToPose): move robot to specific position and orientation
joint_waypoint (std_srvs.srv.Empty)
move_to_position (moveit_testing_interfaces/srv/MoveToPosition): move robot to specific position
move_to_orientation (moveit_testing_interfaces/srv/MoveToOrientation): move robot to specific orientation
update_obstacles (moveit_testing_interfaces/srv/UpdateObstacles): add obstacles to scene
pickup_lightsaber (std_srvs.srv.Empty): commands the robot to follow waypoints to pick up pick up the fixed lightsaber with the end-effector
update_persistent_obstacles (moveit_testing_interfaces.srv.UpdateObstacles): adds obstacle to scene permanently
update_attached_obstacles (moveit_testing_interfaces.srv.UpdateAttachedObstacles): adds obstacle attached to robot links
home_waypoint (std_srvs.srv.Empty): plans to a hard coded value in the callback
add_walls (std_srvs.srv.Empty): adds walls and ceiling collision objects to the planning scene
remove_separate_lightsaber (std_srvs.srv.Empty): removes the lightsaber as collision object
add_separate_lightsaber (std_srvs.srv.Empty): add lightsaber as a separate collision object
remove_attached_lightsaber (std_srvs.srv.Empty): remove the lightsaber attached to the end-effector in the planning scene in rviz
add_attached_lightsaber (std_srvs.srv.Empty): add the lightsaber attached to the end-effector to the planning scene as an attached collision object.
look_for_enemy (std_srvs.srv.Empty): check for any enemies detected in the planning scene and begins to calculate how to attack if possible
reset_allies (std_srvs.srv.Empty): updates the position of the allies in the planning scene to current

List of all nodes and launchfiles and what they do:
Package Name: attack_of_the_franka
    Type: ament_python
Nodes:
    camera_processor.py
        Performs image processing for ally and enemy detection based on color.
    Gets workspace area transforms and the robot transform
    common.py
        A common library for functions/values used by all nodes
    robot_control.py
Runs the state machine related to motion and interacts with our moveit_interface API to plan and attack enemies. It processes camera information to locate allies and enemies and provides helpful services to do a variety of tasks with the lightsaber and blocks interaction. 
    simple_move
Controls robot and planning scene using the moveit_interface API.
Launchfiles:
    realsense.launch.py 
Launches the nodes needed to read in information from the RealSense camera and april tags to recognize the table and block locations relative to the robot base
    robot.launch.py 
Launches the robot_control node and other programs to view the rviz simulation of the franka robot
    attach_of_the_franka.launch.py 
        
combines the above two launch files to set up all the nodes needed to run the services and nodes needed to attack enemies
    moveit_testing.launch.py
Launches the moveit launch file as well as out testing node named simple_move
Videos:
https://drive.google.com/file/d/1o-AKXsr8Wqxdj_DX8L0xjUgXkbujYIkm/view?usp=sharing
https://drive.google.com/file/d/1WRIVM4u_p33MkGhK_5lwX5Xugu6PrdSB/view?usp=sharing

Clean 1 ally 1 enemy shot:
https://drive.google.com/file/d/1_kVLCrcbNZSKzEiMgiprXgzdQIEo_umJ/view?usp=sharing

Any other information, as appropriate (algorithms used, lessons learned, future work):
Lessons Learned:

Future Work:
Future work we would like to incorporate would be to have more robust movements that could take the distance from an ally to adjust its swing waypoints. We would also like to adjust our gripper to be able to turn the lightsaber on when it picks it up. 

