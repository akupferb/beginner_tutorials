# ROS package "beginner_tutorials"
Built from basics of ROS beginner tutorials package from ROS Wiki, this package includes a basic pub/sub system, and added: a service, launch arugments and parameters, a TF broadcaster, and a level 2 ros integration test.

## Overview
The ROS package beginner_tutorials is taken from a basic catkin workspace created using the ROS Wiki tutorials for an initial Publisher & Subcsriber nodes model. The publisher creates a custom text string and publishes it to a topic using the std_msgs class type. The Subscriber receives the data from the same topic and uses its callback function to print out the received string with info. The package has ROS launch files that can activate and run both nodes at once, and has arguments that can be used to modify the nodes. In addition, the Subscriber node also contains a server that calls for data that can change the subscriber's output string, and the Publisher node contains a broadcaster for a transformation frame (TF).

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

## Test
```
cd <your catkin workspace>
catkin_make run_tests
```

## Run
### To Run using ROS Launch (with ROSbag capability)
To run the launch file with no arguments, (this will use the default setting):
```
roslaunch beginner_tutorials naming.launch
```
There are three commmand-line arguments available with the 'naming.launch' file:

* speaker (Modifies the output string name for who is talking)<br/>
* looper (Modifies the publish frequency of the talker node)<br/>
* use_rosbag (Enables="1"/Disables="0" Rosbag recording)<br/>

The default values are: *speaker*="Ari", *looper*="5", *use_rosbag*="0"<br/>
**Note: *use_rosbag* only takes binary inputs of 0 or 1**

To modify the corresponding parameters, you use the := operator<br/>
The following examples show a variety of ways you can use the arguments:
```
roslaunch beginner_tutorials naming.launch speaker:=Sam
roslaunch beginner_tutorials naming.launch looper:=10 use_rosbag:=1
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

## Ros Bag Recording
You can enable rosbag to record topics published and play them back later.<br/>
Rosbag can be enabled using the roslaunch argument with a value of 1 (```use_rosbag:=1```), and disabled using a value of 0.<br/>
If enabled, rosbag will record all topics (the entire time the program is runnning) to a .bag file in its home directory [~/.ros], with a unique name identifier. You can move and rename this file to a location of your choice using the "mv" operator.<br/>
To inspect the bag file, run: ``` rosbag info <filename>.bag ```, which will show the details of the file. If the bag file recorded the topics properly, it should output a list including the file size, message number, and topics, amongst others.
## Ros Bag Playback
To use rosbag to playback topics while the original publisher is not running, ensure the ROS master (roscore) is active, and run **only** the listener node in a separate terminal: 
```rosrun beginner_tutorials listener```

Then, open a new terminal:
```
cd <bag file location>
rosbag play <filename>.bag
```
And you should see the bag file publish the topics and the listener receive them!


## ROS Service
The Subscriber node is also set up as a Server node, which can accept service calls to change the ouput string.<br/>
To utilize the service, open a new terminal while the program is running and enter the following:
```
rosservice call /toggle_header
```

## ROS TF
To view the TF broadcast sent out by the talker node, run the following in a new terminal while roslaunch is running:
```
 rosrun tf tf_echo /world /talk
```
To view the tree of frames being broadcast over ROS, run the following in a new terminal while roslaunch is running:
```
rosrun rqt_tf_tree rqt_tf_tree
```

## Notes
The package.xml file use *exec_depend* instead of *run_depend*. For this format change, 'package' in line 2 has been updated to 'package format="2"'.
