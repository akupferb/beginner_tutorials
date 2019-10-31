/**Copyright (C) 2019 Ari Kupferberg
 * @file        listener.cpp
 * @author      Ari Kupferberg
 * @date        10/27/2019
 * @brief       ROS file for subscribing to text strings, 
 *              and changing output based on service calls
 */
#include "std_msgs/String.h"
#include <std_srvs/Empty.h>
#include "ros/ros.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 * And the use of a service to change output string,
 */

bool header = true;

/**
*  @brief   Function Callback for Service that changes a boolean value
*  @param	Service Request Type variable passed by reference
*  @param	Service Response Type variable passed by reference
*  @return	boolean true
*/
bool toggleHeader( std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &resp) {
   header = !header;
   ROS_WARN_STREAM("Header changed to: " << (header?"'I heard'":"'Listen now'"));
   return true;
}

/**
*  @brief   Function Callback for Subscriber that prints a text string
*  @param	constant msg type passed as a pointer
*  @return	None
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
   ROS_INFO_STREAM((header?"I heard":"Listen now") << ": [" << msg->data << "]");
}

/**
*  @brief   This is the main function
*  @param	argc for ROS
*  @param	argv for ROS
*  @return	0 Exit status
*/
int main(int argc, char **argv) {
   // Initialize the ROS system
   ros::init(argc, argv, "listener");

   // Establish this program as a ROS node
   ros::NodeHandle nh;

   // Register our service with the master
   ros::ServiceServer server = nh.advertiseService("toggle_header", &toggleHeader);

   // Create a Subscriber object
   ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

   // Give control to ROS
   ros::spin();

   return 0;
}
