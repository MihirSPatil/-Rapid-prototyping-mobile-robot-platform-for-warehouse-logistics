#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <grizzly_msgs/Drive.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#define PI 3.1415926535897931

int64_t enL = 0;
int64_t enR = 0;

int64_t oldenL = 0;
int64_t oldenR = 0;

double wheelCircumference = 0.63; //meters
double wheelDistance = 0.44; //meters

uint8_t encoderResolution = 30;

ros::Time current_time, last_time;
ros::Publisher odom_pub;
geometry_msgs::Quaternion odom_quat;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double dx = 0, dy = 0, dth = 0;

void algorithm2(double dl, double dr, double dt);


void tempCallback(const grizzly_msgs::Drive::ConstPtr & msg) {
  double left = msg->front_left + msg->rear_left;

  left /= 2;

  double right = msg->front_right + msg->rear_right;
  right /= 2;

  //ROS_INFO_STREAM(left << "," << right);


  //==============================================================


  double dl = (left * wheelCircumference) / encoderResolution;
  double dr = (right * wheelCircumference) / encoderResolution;

  //ROS_INFO_STREAM("Left:" << dl << "\tRight:" << dr);

  double dt = (current_time - last_time).toSec();
  //=============================== Calculate x, y, th ====================

  algorithm2(dl, dr, dt);

  vx = dx / dt;
  vy = dy / dt;
  vth = dth / dt;

  x += dx;
  y += dy;
  th += dth;

  if (th > PI) {
    th = th - (2 * PI);
  } else if (th < -PI) {
    th = th + (2 * PI);
  }

  //  ROS_INFO_STREAM("x:" << x << "\ty:" << y << "\ttheta:" << ((th * 180) / PI));

  //  ROS_INFO_STREAM("theta:" << ((th * 180) / PI));

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  odom_quat = tf::createQuaternionMsgFromYaw(th);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  odom_pub.publish(odom);

  last_time = current_time;
}

void algorithm2(double dl, double dr, double dt) {
  //Solved from
  //http://rossum.sourceforge.net/papers/DiffSteer/
  double s_ = (dl + dr) / 2;

  dth = (dl - dr) / (wheelDistance);
  dx = s_ * cos(th + dth); //th+dth because th+=dth is to be done after this function is executed
  dy = s_ * sin(th + dth);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_temp");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cmd_drive", 50, tempCallback);

  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate rate(40);

  while (n.ok()) {
    ros::spinOnce();

    //TF broadcaster
    static tf::TransformBroadcaster odom_tf_broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, th);
    transform.setRotation(q);
    odom_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    rate.sleep();
  }
}
