/*
Copyright (c) 2019, Ari Kupferberg
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**
 * @file        talker.cpp
 * @author      Ari Kupferberg
 * @date        10/27/2019
 * @brief       ROS file for publishing text strings
 */

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <log4cxx/logger.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <cmath>


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
  // 2 lines of code, taken from the book,
  // that activate DEBUG log viewing in console:
  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  ros::console::notifyLoggerLevelsChanged();
  // --------

  // Establish this program as a ROS node
  ros::NodeHandle n;

  std::string speakerName;
  int loopRate;
  // Get the parameter values for 'speakerName'
  // & 'loopRate' from ROS Parameters
  const std::string PARAM1 = "~speaker";
  bool ok1 = ros::param::get(PARAM1, speakerName);
  const std::string PARAM2 = "~looper";
  bool ok2 = ros::param::get(PARAM2, loopRate);

  // If parameter does not return a value,
  // produce error message, and exit program
  if (!ok1) {
    ROS_FATAL_STREAM("Could not get parameter: " << PARAM1);
    exit(1);
  }
  if (!ok2) {
    ROS_FATAL_STREAM("Could not get parameter: " << PARAM2);
    exit(1);
  }

  // Create Publisher object
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // Loop at 5 Hz until the node is shut down
  ros::Rate loopCycle(loopRate);

  int count = 0;
  std::string textString;
  while (ros::ok()) {
    ROS_DEBUG_STREAM_ONCE("The counter will restart every 100 iterations.");
    if (count > 100) {
      // Send a one-time output as a log message of 'error' level
      ROS_ERROR_STREAM_THROTTLE(20, "RESTARTING COUNT");
      count = 0;
    }
    textString = " says: Good morning 808X ";
    if (count > 25) {
      // Send a one-time  output as a log message of 'debug' level
      ROS_DEBUG_STREAM_THROTTLE(20, count-1 << " iterations have passed.");
      textString = " says: Good afternoon 808X! ";
    }
    if (count > 50) {
      // Send a one-time output as a log message of 'info' level
      ROS_INFO_STREAM_THROTTLE(20, count-1 << " iterations have passed!");
      textString = " says: Good evening 808X!! ";
    }
    if (count > 75) {
      // Send a one-time output as a log message of 'warn' level
      ROS_WARN_STREAM_THROTTLE(20, count-1 << " ITERATIONS HAVE PASSED!!!");
      textString = " says: GOOD NIGHT 808X!!! ";
    }

    // Concatenate the string into a string stream
    std::stringstream ss;
    ss << " " << speakerName << textString;

    // Assign the string to the message data
    std_msgs::String msg;
    msg.data = ss.str();

    // ROS_INFO("%s", msg.data.c_str());

    // Publish the message to the 'chatter' topic
    chatter_pub.publish(msg);

    // ------
    // Create the transform broadcaster object
    static tf::TransformBroadcaster br;
    // Create the transform object
    tf::Transform transform;
    // Set the frame translation
    transform.setOrigin(tf::Vector3(3.0, 5.0, 2.0));
    // Create and set a quaternion object
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI/2);
    // Set the frame rotation
    transform.setRotation(q);
    // Send the transform into the ether
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    // ------

    // Give one-time control to ROS
    ros::spinOnce();

    // Wait until next iteration
    loopCycle.sleep();
    // Increase counter
    ++count;
  }

  return 0;
}
