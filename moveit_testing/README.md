# Motion Planning Interface Package

This is an API for the ROS2 MoveIt MoveGroup node in Python.

To see a demo of the API, run the following command:

`ros2 launch moveit_testing moveit_testing.launch.py`

This will launch the demonstration node, `simple_move`. From there you can call one of the below services to demonstrate different functionality.

## Features of the demo node:
### move_to_pose Service 
of type moveit_testing_interfaces/srv/MoveToPose

This service takes in a pose specifying both the position and orientation goals for the end-effector

Command Line Example Call:

`ros2 service call /move_to_pose moveit_testing_interfaces/srv/MoveToPose "pose: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0., y: 0., z: 0., w: 1.}}"`

### move_to_position Service 
of type moveit_testing_interfaces/srv/MoveToPosition

This service takes in only a position goal for the end effector

Command Line Example Call:

`ros2 service call /move_to_position moveit_testing_interfaces/srv/MoveToPosition "position: {x: 0.5, y: 0.5, z: 0.5}"`

### move_to_orientation Service 
of type moveit_testing_interfaces/srv/MoveToOrientation

This service takes in only a orientation goal for the end effector

Command Line Example Call:

`ros2 service call /move_to_orientation moveit_testing_interfaces/srv/MoveToOrientation "orientation: {x: 0., y: 0., z: 0., w: 1.}"`
    
### update_obstacles Service 
of type moveit_testing_interfaces/srv/UpdateObstacles
This service takes in a position (x,y,z), a length, a width, a height, and an id and spawns a box in the planning scene. If the `delete_obstacle` parameter is false, it will add the box to the scene. If it is true, it will remove the box from the scene.

Command Line Example Call:

`ros2 service call /update_obstacles moveit_testing_interfaces/srv/UpdateObstacles "{position: {x: 0.5, y: 0.5, z: 1.0}, length: 0.5, width: 0.25, height: 2.0, id: 'MyBox', delete_obstacle: false}"`

## Inputs to the API:
### Parameters to API: 
node - pass in the node that calls the functions contained in this API
config - the custom parameters contained within the MoveConfig() class that must be specified by the user 

## Externally Available from API: 
### Functions:
#### handle function
Cyclically handle MoveIt interactions.

Must be called in a timer callback.
        
Moves between IDLE, PLANNING, EXECUTING, and OBSTACLE states to keep track of which related MoveIt Interation is being affected

Arguments:
    none

Returns:
    none


#### plan_traj_to_pose function
Plan a trajectory with the full input pose, calls _plan_traj function with user provided input

Arguments:
    goal_pose - a geometry Pose and/or Orientation variable that specifies where 
    start_pose - (optional) if none is specified, then use the current configuration if start pose is provided, that input must be of type geometry message Pose with x, y, z
    execute - (optional) defaults to false. If it is true, the plan will be immediately executed after calculation.
Returns:
    error of type enum ERROR_CODES provided from _plan_traj function


#### plan_traj_to_position function
Plan a trajectory with only an input position specified, calls _plan_traj function with user provided input

Arguments:
    goal_pose - a geometry Pose and/or Orientation variable that specifies where 
    start_pose - (optional) if none is specified, then use the current configuration if start pose is provided, that input must be of type geometry message Pose with x, y, z
    execute - (optional) defaults to false. If it is true, the plan will be immediately executed after calculation.
Returns:
    error of type enum ERROR_CODES provided from _plan_traj function


#### plan_traj_to_orientation function
Plan a trajectory with only an input orientation specified, calls _plan_traj function with user provided input

Arguments:
    goal_pose - a geometry Pose and/or Orientation variable that specifies where 
    start_pose - (optional) if none is specified, then use the current configuration if start pose is provided, that input must be of type geometry message Pose with x, y, z
    execute - (optional) defaults to false. If it is true, the plan will be immediately executed after calculation.
Returns:
    error of type enum ERROR_CODES provided from _plan_traj function


#### exec_traj function
Execute a trajectory.

Arguments:
    plan - If the plan argument is passed in, use that plan. Otherwise use 
    the internally saved plan.
Returns:
    none

#### get_plan function
We can use this if we want to calculate multiple plans at one time and then
execute them without having to plan in between each one. The higher level
node would store the plans somewhere and then pass them back into our execute
trajectory method.
Arguments:
    none
    
Returns:
    self._plan - the current plan saved from the plan state machine


#### update_obstacles function
Externally accessible function that allows user to update information on obstacles in the plan space including adding, modifying, and deleting.

Arguments:
    obstacle_list - list of obstacles, each of type moveit_msgs.msg.CollisionObject()
    delete - a boolean variable to specify whether to delete the obstacles

Returns:
    none

#### get_last_error function
Externally accessible function for the user to get what error the api currently has stored

Parameters:
    none
Returns:
    variable of type Enum MoveItApiErrors indicating error state

### Status Outputs

- `busy` (bool) - true if the main sequence is busy planning or executing.
- `planning` (bool) - true if the planning subsequence is currently running.
- `executing` (bool) - true if the executing subsequence is currently running.
- `planning_and_executing` (bool) - true is busy both planning and executing.

## Authors:
- Sushma Chandra
- Vaishnavi Dornadula
- Nick Morales
- Megan Sindelar

## Last Edited: December 10th, 2022
