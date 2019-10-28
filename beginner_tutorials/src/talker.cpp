/**Copyright (C) 2019 Ari Kupferberg
 * @file        talker.cpp
 * @author      Ari Kupferberg
 * @date        10/27/2019
 * @brief       ROS file for publishing text strings
 */

#include <sstream>
#include "std_msgs/String.h"
#include "ros/ros.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

/**
*  @brief   This is the main function
*  @param	argc for ROS
*  @param	argv for ROS
*  @return	0 Exit status
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    
    ss << " Ari says: ""Hello ROS & 808X!"" " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
