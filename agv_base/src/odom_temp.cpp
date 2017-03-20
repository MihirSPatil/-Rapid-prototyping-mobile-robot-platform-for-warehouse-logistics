#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <grizzly_msgs/Drive.h>

#include <iomanip>
#include <iostream>
#include <sstream>


void tempCallback(const grizzly_msgs::Drive::ConstPtr & msg) {
  double left = msg->front_left + msg->rear_left;

  left /= 2;

  double right = msg->front_right + msg->rear_right;
  right /= 2;

  //ROS_INFO_STREAM( msg->front_left<<","<<msg->front_right<<","<<msg->rear_left<<","<<msg->rear_right);
  ROS_INFO_STREAM(left << "," << right);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_temp");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cmd_drive", 50, tempCallback);

  ros::Publisher val_pub = n.advertise<std_msgs::String>("odom_pub", 50);


  ros::spin();

  return 0;
}
