This repository contains all the files needed to run the package consitsting of a custom URDF/XACRO robot with the Navigation Stack and Joint Trajectory controller.

![URDF Robot](https://user-images.githubusercontent.com/94687473/151810045-e1a3b6e8-002b-4188-aef8-68717f377d66.png)

https://user-images.githubusercontent.com/94687473/151810094-26261796-5dab-4566-bc13-6bc5b9d60cdd.mp4



Instructions to install:
1) Change directory to workspace: `cd ~/catkin_ws/src/`
2) Git clone the repository in the src directory of the workspace.
3) `catkin_make` at the root of the workspace
4) `source ~/catkin_ws/devel/setup.bash`


ROS Requirements:
1) Run the following to update the navigation stack for ROS Noetic:
`$ sudo apt-get update
$ sudo apt-get install ros-noetic-move-base
$ sudo apt-get install ros-noetic-map-server
$ sudo apt-get install ros-noetic-amcl
$ sudo apt-get install ros-noetic-eband-local-planner
$ sudo apt-get install ros-noetic-global-planner`

2) Run the following command to install opencv: 
`$ sudo apt-get install ros-noetic-cv-bridge
$ sudo apt-get install ros-noetic-vision-opencv`


4) This step is not mandatory as it only affects the material colours in Gazebo. The file "gazebo.material" present under the model folder, should be copy and pasted in place of the system one.


5) Launch any of the following to spawn the world(the empty or with boxes can be chosen from the launch file), the URDF robot, the specific RVIZ configuration and Gazebo (the Gazebo GUI can be enabled or disabled in the launch files):
- To launch the Navigation Stack simply run: `roslaunch ros_autonomous_manipulator_mobilerobot navigation_stack.launch`
    - For autonomy using the implement grid search algorith, use: `rosrun ros_autonomous_manipulator_mobilerobot grid_search`
- For teleoperation use: `roslaunch ros_autonomous_manipulator_mobilerobot teleop_drive.launch`
- For GUI control use: `roslaunch ros_autonomous_manipulator_mobilerobot gui_drive.launch`
    - To control the manipulator arm which uses a joint_trajectory_controller. Simply use: "rqt" and then select the controller, and then which part of the manipulator arm you'd like to control between "arm_controller", "claw_controller" and "gripper_controller". After giving a point with the sliders, the trajectory_controller will calculate the path.
    - Separately there is a python script that will send a preset position to all joints. Run `rosrun ros_autonomous_manipulator_mobilerobot joint_command`. The file needs to be edited if the position needs to be changed.
- There is another file for full autonomy called "full_autonomy". It is just a work-in-progress. The idea behind it was to have the grid search running with the robot patrolling the map, and then stop the computer vision module when the area of the cube reaches a certain value, then send cmd_vel twist messages to move the robot a bit closer to the box, then run the python script that does the kinematics for the manipulator arm, which would complete a sequence of movements and pick up a box. Once the boxed has been picked up simply send a navigation goal for home. After the system sends a "goal reached", remove the waypoints that have been completed by remembering the index in the list of lists, and then continue with the grid search until the map is completed and all cubes have been brought to the origin. 
In order to use the manipulator accordingly, the computer vision would be used to detect the box, calculate the center of the box countour and the center of the camera frame, and then measuring the distance between the 2 points, which would tell me if the box is on the ground, at a medium level, or high, for example the box on the staircase. Furthermore, to enable the grabber to pick up the box at any angle, I would use the HoughLines method, to find any line and parallel line in the masked frame, and from there project all lines on a 2d graph to understand the distance between 3 parallel lines (2 edges, and the edge/line in the center if the box is rotated towards the robot), which would give me the theta representing the rotation of the cube. This would then be transfered and translated to claw/gripper joint positions.


Directory explanation:
- demo: folder which contains videos/photos of development
- useful_commands: folder which containts txt files with useful ROS commands
- launch: contains all launch files
- script: folder containing all python executable files
- models/world: contains the gazebo world
- models/robot_design/urdf: contains the URDF model created with XACRO
- models/robot_design/config: contains all the joint controllers and configuration files
- rviz: different configurations for RVIZ in function of what file has been launched (teleop, gui or navigation)
- map: contains 2 maps which have been generated with the turtlebot, but my robot can do Hector Mapping as well with RFK localisation.

