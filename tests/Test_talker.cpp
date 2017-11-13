/**
 * @file        : Test_talker.cpp
 * @author      : Sudarshan Raghunathan
 * @copyright   : 2017 Sudarshan Raghunathan
 * @brief       : Unit Tests for ROS Publisher
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/Services.h"

/**
 * @brief Tests client exists for the NewMessage service
 * @param none
 * @return none
 */
TEST(TestSuite, serviceTest) {
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient < beginner_tutorials::Services
      > (
      "update");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

/**
 * @brief Runs the gtests
 * @param argc argv
 * @return runs tests
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talkerTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
