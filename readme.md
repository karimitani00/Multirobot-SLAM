# MR-SLAM Final Project

```
This project carries out mr-slam in simulation
```

## Getting Started

```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workpace
```

### Prerequisites

What software you need to install

```
numpy
ROS
ROS - nav2d
```
### Installing

copy the 'simple_sensing' folder into src.
then build and source:

```
cd ..
catkin_make
```

### Run

to run the nav2d exploration

```
roslaunch mr_slam nav2d_simulation.launch
rosservice call /robot_0/StartMapping 3
rosservice call /robot_0/StartExploration 2

rostopic pub /robot_1/Localize/goal nav2d_navigator/LocalizeActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  velocity: 0.5"

rosservice call /robot_1/StartExploration 2
```
