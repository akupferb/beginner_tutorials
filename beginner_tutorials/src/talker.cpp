/**Copyright (C) 2019 Ari Kupferberg
 * @file        talker.cpp
 * @author      Ari Kupferberg
 * @date        10/27/2019
 * @brief       ROS file for publishing text strings
 */

#include <sstream>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <log4cxx/logger.h>

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
   // Initialize the ROS system
   ros::init(argc, argv, "talker");
   
   // --------
   // 2 lines of code, taken from the book, that activate DEBUG log viewing in console:
   log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
   ros::console::notifyLoggerLevelsChanged();
   // --------

   // Establish this program as a ROS node
   ros::NodeHandle n;

   // Create Publisher object
   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

   // Loop at 5 Hz until the node is shut down
   ros::Rate loop_rate(5);

   int count = 0;
   std::string textString;
   while (ros::ok()) {
   
      if (count > 100) {
         // Send a one-time output as a log message of 'error' level
         ROS_ERROR_STREAM_ONCE("RESTARTING COUNT");
         count = 0;
      }
      textString = " Ari says: Good morning 808X ";
      if (count > 25) {
         // Send a one-time  output as a log message of 'debug' level
         ROS_DEBUG_STREAM_ONCE(count-1 << " iterations have passed.");
         textString = " Ari says: Good afternoon 808X! ";
      }
      if (count > 50) {
         // Send a one-time output as a log message of 'info' level
         ROS_INFO_STREAM_ONCE(count-1 << " iterations have passed!");
         textString = " Ari says: Good evening 808X!! ";
      }
      if (count > 75) {
         // Send a one-time output as a log message of 'warn' level
         ROS_WARN_STREAM_ONCE(count-1 << " ITERATIONS HAVE PASSED!!!");
         textString = " Ari says: GOOD NIGHT 808X!!! ";
      }

      // Assign the string to the message data
      std_msgs::String msg;
      msg.data = textString;

      // ROS_INFO("%s", msg.data.c_str());

      // Publish the message to the 'chatter' topic
      chatter_pub.publish(msg);
      
      // Give one-time control to ROS
      ros::spinOnce();

      // Wait until next iteration
      loop_rate.sleep();
      // Increase counter
      ++count;
   }

   return 0;
}
