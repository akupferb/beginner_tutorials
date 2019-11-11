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
 * @file        myTest.cpp
 * @author      Ari Kupferberg
 * @date        11/10/2019
 * @brief       RosTest for Talker node TF broadcaster
 */
 
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

/**
 * @brief Test Case will check if the TF broadcaster is running properly
 * @param TESTSUITE group
 * @param TestTfBroadcast description
 * @return None
 */
TEST(TESTSuite, testTfBroadcast) {
  // Create a Ros Node Handle
  ros::NodeHandle nTest;
  // Create a TF listener object
  tf::TransformListener listener;
  // Create a TF stamped transform object
  tf::StampedTransform transform;

  listener.waitForTransform("/world", "/talk", ros::Time(0), ros::Duration(10));
  listener.lookupTransform("/world", "/talk", ros::Time(0), transform);
  
  double z = transform.getOrigin().z();
  EXPECT_EQ(z, 2);
}

/**
 * @brief Main function initializes ROS and runs all the TESTS created
 * @param	argc for ROS
 * @param	argv for ROS
 * @return Run all the tests
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "testTalker");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

