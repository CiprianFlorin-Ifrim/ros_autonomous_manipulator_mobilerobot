# assessmentmobilerobots_2022
Files for Mobile Robots - PDE3432 - Assessment III 2022
This repository contains all the files needed to run the package.


Instructions to install:
1) Change directory to workspace: cd ~/catkin_ws/src/
2) Git clone the repository in the src directory of the workspace.
3) "catkin_make" at the root of the workspace
4) source ~/catkin_ws/devel/setup.bash


ROS Requirements:
1) Run the following to update the navigation stack for ROS Noetic:
$ sudo apt-get update
$ sudo apt-get install ros-noetic-move-base
$ sudo apt-get install ros-noetic-map-server
$ sudo apt-get install ros-noetic-amcl
$ sudo apt-get install ros-noetic-eband-local-planner
$ sudo apt-get install ros-noetic-global-planner

2) Run the following command to install opencv: 
$ sudo apt-get install ros-noetic-cv-bridge
$ sudo apt-get install ros-noetic-vision-opencv


4) This step is not mandatory as it only affects the material colours in Gazebo. The file "gazebo.material" present under the model folder, should be copy and pasted in place of the system one.


5) Launch any of the following to spawn the world(the empty or with boxes can be chosen from the launch file), the URDF robot, the specific RVIZ configuration and Gazebo (the Gazebo GUI can be enabled or disabled in the launch files):
- To launch the Navigation Stack simply run: `roslaunch assessmentmobilerobots_2022 navigation_stack.launch`
    - For autonomy using the implement grid search algorith, use: `rosrun assessmentmobilerobots_2022 grid_search`
- For teleoperation use: `roslaunch assessmentmobilerobots_2022 teleop_drive.launch`
- For GUI control use: `roslaunch assessmentmobilerobots_2022 gui_drive.launch`
    - To control the manipulator arm which uses a joint_trajectory_controller. Simply use: "rqt" and then select the controller, and then which part of the manipulator arm you'd like to control between "arm_controller", "claw_controller" and "gripper_controller". After giving a point with the sliders, the trajectory_controller will calculate the path.
    - Separately there is a python script that will send a preset position to all joints. Run `rosrun assessmentmobilerobots_2022 joint_command`. The file needs to be edited if the position needs to be changed.
- There is another file for full autonomy called "". It is just a work-in-progress. The idea behind it was to have the grid search running with the robot patrolling the map, and then stop the computer vision module when the area of the cube reaches a certain value, then send cmd_vel twist messages to move the robot a bit closer to the box, then run the python script that does the kinematics for the manipulator arm, which would complete a sequence of movements and pick up a box. Once the boxed has been picked up simply send a navigation goal for home. After the system sends a "goal reached", remove the waypoints that have been completed by remembering the index in the list of lists, and then continue with the grid search until the map is completed and all cubes have been brought to the origin.


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

Notes:
- I am available for a Zoom meeting to explain anything, if requested.
- Watch the video "presentation" in the demo folder, as it goes over everything available/developed.
- I have added a 3s delay to the move_random script as the node would crash if all the controllers and the world wouldn't be loaded fast enough, issue caused by cpu or cache.
- The wheels have a PID controller with tuned values, however I have removed the PID from the manipulator arm joints as the PID was drasticly reducing the speed of the movements, and optimal values have not been found for the simulation.
- The manipulator_arm branch has been merged with the main branch and deleted after.
