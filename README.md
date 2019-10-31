# ROS package "beginner_tutorials"
Built from basics of ROS beginner tutorials package from ROS Wiki

## Overview
The ROS package beginner_tutorials is taken from a basic catkin workspace created using the ROS Wiki tutorials for an initial Publisher & Subcsriber nodes model. The publisher creates a custom text string and publishes it to a topic using the std_msgs class type. The Subscriber receives the data from the same topic and uses its callback function to print out the received string with info. The package has ROS launch files that can activate and run both nodes at once, and has arguments that can be used to modify the nodes.

## Assumptions/Dependencies
The code assumes you are using *Ros Kinetic*.

The package builds using *catkin*.

This package relies on the following dependencies that must be installed:
* roscpp
* rospy
* std_msgs
* std_srvs

## Download Package
```
cd <your catkin workspace>/src
git clone https://github.com/akupferb/beginner_tutorials.git
```

## Build
```
cd <your catkin workspace>
catkin_make
source devel/setup.bash
```

## Run
### To Run using ROS Launch
To run the launch file with no arguments, (this will use the default setting):
```
roslaunch beginner_tutorials naming.launch
```
There are two commmand-line arguments available with the 'naming.launch' file:

* speaker (Modifies the output string name for who is talking)<br/>
* looper (Modifies the publish frequency of the talker node)<br/>

The default values are: *speaker*="Ari", *looper*="5"

To modify the corresponding parameters, you use the := operator<br/>
The following examples show uses of either or both arguments:
```
roslaunch beginner_tutorials naming.launch speaker:=Sam
roslaunch beginner_tutorials naming.launch looper:=10
roslaunch beginner_tutorials naming.launch speaker:=Alex looper:=2
```

### To Run without using ROS Launch
Open three separate terminals.

Terminal 1: 
```
roscore
```
Terminal 2:
```
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

## ROS Service
The Subscriber node is also set up as a Server node, which can accept service calls to change the ouput string.<br/>
To utilize the service, open a new terminal while the program is running and enter the following:
```
rosservice call /toggle_header
```

## Notes
The package.xml file use *exec_depend* instead of *run_depend*. For this format change, 'package' in line 2 has been updated to 'package format="2"'.
