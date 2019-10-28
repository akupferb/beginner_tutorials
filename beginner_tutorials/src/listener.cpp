/**Copyright (C) 2019 Ari Kupferberg
 * @file        listener.cpp
 * @author      Ari Kupferberg
 * @date        10/27/2019
 * @brief       ROS file for subscribing to text strings
 */

#include "std_msgs/String.h"
#include "ros/ros.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */



/**
*  @brief   Function Callback for Subscriber that prints a text string
*  @param	constant msg type passed by reference
*  @return	None
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
*  @brief   This is the main function
*  @param	argc for ROS
*  @param	argv for ROS
*  @return	0 Exit status
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
