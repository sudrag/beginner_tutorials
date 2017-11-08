/**
 * @file        : talker.cpp
 * @author      : Sudarshan Raghunathan
 * @copyright   : 2017 Sudarshan Raghunathan
 * @brief       : ROS Publisher
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/Services.h"


// Message from master, message without service
std::string message = "Modified String Inserted :";  // NOLINT
/**
 * @brief Updates message to subsribe, send to listener
 * @param request and response from srv
 * @return true
 */
bool update_string(beginner_tutorials::Services::Request &req,  // NOLINT
    beginner_tutorials::Services::Response &res) {  // NOLINT
      message = req.mesReq;
      res.mesRep = message;
  ROS_INFO_STREAM("Updating with the new message");
      return true;
    }

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

   int rate = 10;  // defualt of 10
  if (argc == 2) {  // if argument is given from launch arc will equal 2, otherwise 1
    if (atoi(argv[1]) < 0) {
      ROS_ERROR_STREAM("Frequency entered is negative");
    }
    rate = atoi(argv[1]);  // argv[1] is the argument talkFreq when we say talkFreq:=value in roslaunch command
    ROS_DEBUG_STREAM("Frequency changed to " << rate);
  }

  if (rate < 3) {
    ROS_WARN_STREAM("Publisher Frequency is too slow");
  }

  ros::Rate loop_rate(rate);  // set the rate
  ROS_INFO_STREAM("Currrent Rate: " << rate);

  ros::ServiceServer service = n.advertiseService("update", update_string);  // Server
 

 /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << message<< " " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
