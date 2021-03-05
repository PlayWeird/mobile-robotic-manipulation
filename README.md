# mobile-robotic-manipulation

A project for CS 491 Autonomous Mobile Manipulation class at the University of Nevada, Reno.

# Setup

## First time setting up only

Setup a workspace with the instructions at [Autonomous Movile Manipulation GitHub repository](https://github.com/robowork/autonomous_mobile_manipulation/tree/project).

Clone this repository in `~/your_catkin_ws/src`.

Your directory tree should look like this:
```
src
├── autonomous_mobile_manipulation
│   └── ...
└── mobile-robotic-manipulation
    ├── README.md
    └── sample_arm_control
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch
        ├── include
        └── src
```

## Build packages in this repository

Build the packages in this project repository (replace `<your_catkin_ws>` with your catkin workspace name) with:
```
cd ~/<your_catkin_ws>/src/mobile-robotic-manipulation

CMAKE_PREFIX_PATH=~/<your_catkin_ws>/devel:/opt/ros/melodic catkin build sample_arm_control
```

# Run sample arm control simulation

## Launch the simulation

Source your workspace.

In terminal 1, run:
```
roslaunch robowork_gazebo bvr_SIM_playpen.launch
```

Wait until terminal 1 finishes setting up the simulation (after about 10 seconds), in terminal 2, run:
```
ROS_NAMESPACE="bvr_SIM" roslaunch robowork_moveit_config robowork_moveit_planning_execution.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM
```

Wait until terminal 2 finishes setting up Moveit (after about 3 seconds), in terminal 3, run:
```
ROS_NAMESPACE="bvr_SIM" roslaunch sample_arm_control sample_arm_control.launch
```

## Expected result

The gripper arm on the robot moves upward.
