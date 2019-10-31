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
 * @file        listener.cpp
 * @author      Ari Kupferberg
 * @date        10/27/2019
 * @brief       ROS file for subscribing to text strings, 
 *              and changing output based on service calls
 */
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include "ros/ros.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 * And the use of a service to change output string,
 */

bool header = true;

/**
*  @brief   Callback Function for Service that changes a boolean value
*  @param	  Service Request Type variable passed by reference
*  @param	  Service Response Type variable passed by reference
*  @return	boolean true
*/
bool toggleHeader(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &resp) {
  header = !header;
  ROS_WARN_STREAM("Header changed to: "
             << (header?"'I heard'":"'Listen now'"));
  return true;
}

/**
*  @brief   Callback Function for Subscriber that prints a text string
*  @param	  constant msg type passed as a pointer
*  @return	None
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO_STREAM((header?"I heard":"Listen now")
                          << ": [" << msg->data << "]");
}

/**
*  @brief   This is the main function
*  @param	  argc for ROS
*  @param	  argv for ROS
*  @return	0 Exit status
*/
int main(int argc, char **argv) {
  // Initialize the ROS system
  ros::init(argc, argv, "listener");

  // Establish this program as a ROS node
  ros::NodeHandle nh;

  // Register our service with the master
  ros::ServiceServer server = nh.advertiseService(
                         "toggle_header", &toggleHeader);

  // Create a Subscriber object
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  // Give control to ROS
  ros::spin();

  return 0;
}
