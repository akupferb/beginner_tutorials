# ROS package "beginner_tutorials"
Results from ROS beginner tutorials package from ROS Wiki

## Overview
The ROS package beginner_tutorials is taken from a basic catkin workspace created using the ROS Wiki tutorials for an initial Publisher & Subcsriber nodes model. The publisher creates a custom text string and publishes it to a topic using the std_msgs class type. The Subscriber receives the data from the same topic and uses its callback function to print out the received string with info.

## Assumptions/Dependencies
The code assumes you are using *Ros Kinetic*.

The package builds using *catkin*.

This package relies on the following dependencies that must be installed:
* roscpp
* rospy
* std_msgs

## To Run
Open three separate terminals.

Terminal 1: 
```
roscore
```
Terminal 2:
```
cd <your catkin workspace source file>
git clone https://github.com/akupferb/beginner_tutorials.git
cd <your catkin workspace>
catkin_make
source devel/setup.bash
rosrun beginner_tutorials talker
```
Terminal 3:
```
cd <your catkin workspace>
source devel/setup.bash
rosrun beginner_tutorials listener
```

## Notes
The package.xml file use *exec_depend<>* instead of *run_depend<>*. For this format change, '<package>' has been updated to '<package format="2">'.



