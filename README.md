# ROS 2 Turtlesim Catch-em-all

This repo is a three-package ROS 2 (C++) workspace showcasing a turtle robot dynamically seeking and navigating to other turtles.

Once a turtle is reached, it is despawned and a new turtle spawns at a random coordinate.

## Iteration 2 (latest src commit), with angular PD controller:

<img width="1920" height="1080" alt="ROS 2 Turtlesim Catchemall PD" src="https://github.com/user-attachments/assets/a50ba3fd-df50-48ff-82f1-b40a2b6c7616" />

## Iteration 1 (outdated), using first principles and without an angular PD controller:

<img width="1920" height="1080" alt="ROS 2 Turtlesim Catchemall" src="https://github.com/user-attachments/assets/7ffdf807-fb9f-4db2-a0d8-9d1c38bb8bbf" />

## Architecture

<img width="1425" height="608" alt="Catchemall drawio" src="https://github.com/user-attachments/assets/1da8d724-42ff-46ab-ac0b-1309746b5b33" />

## Prerequisites

- ROS 2 (developed and tested on **Humble**, OS: Ubuntu 22.04)
- Turtlesim

```bash
sudo apt install ros-$ROS_DISTRO-turtlesim
```

## Build

```bash
git clone https://github.com/dynanomino/ros2-turtlesim-catchemall.git
cd ros2-turtlesim-catchemall
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch my_robot_bringup turtle1_controller.launch.xml
```

## What you should see

The turtlesim window should pop open, and the controller node will drive the turtle to new turtles, once they spawn in.
The terminal will log node activity.

## Compatibility

This was tested on ROS 2 Humble, it is recommended to build from source if any issues arise in importing.

## Thank You For Reading!
