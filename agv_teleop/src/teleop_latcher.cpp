#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

ros::Publisher twist_pub;
geometry_msgs::Twist vel;

void cb(const geometry_msgs::Twist msg) {
  vel = msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "teleop_latcher");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel_1", 50, cb);
  twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

  ros::Rate rate(50);

  while (n.ok()) {
    ros::spinOnce();

    twist_pub.publish(vel);

    rate.sleep();
  }
}
