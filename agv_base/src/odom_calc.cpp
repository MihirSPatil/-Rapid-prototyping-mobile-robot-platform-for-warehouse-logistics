#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
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

void algorithm1(double dl, double dr, double dt);
void algorithm2(double dl, double dr, double dt);

double string_to_double(const std::string& s) {
  std::istringstream i(s);
  double x;

  if (!(i >> x)) return 0;

  return x;
}

void odomCallback(const std_msgs::String::ConstPtr & msg) {
  current_time = ros::Time::now();

  std::istringstream ss(msg->data);
  std::string token;
  int i = 0;
  std::string in[2];

  while (std::getline(ss, token, ',')) {
    in[i++] = token;
  }

  enL = -round(string_to_double(in[0]));
  enR = -round(string_to_double(in[1]));

  //ROS_INFO_STREAM("Left:" << enL << "\tRight:" << enR);

  //-------------------- Convert enL and enR into Meters -------------
  int difL = 0, difR = 0;

  if (enL != oldenL) {
    difL = enL - oldenL;
    oldenL = enL;
  }

  if (enR != oldenR) {
    difR = enR - oldenR;
    oldenR = enR;
  }

  double dl = (difL * wheelCircumference) / encoderResolution;
  double dr = (difR * wheelCircumference) / encoderResolution;

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

void algorithm1(double dl, double dr, double dt) {
  //Solved from
  //http://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot

  if (fabs(dl - dr) < 1.0e-6) { // basically going straight
    dx = dl * cos(th);
    dy = dr * sin(th);
  } else {
    float R = wheelDistance * (dl + dr) / (2 * (dr - dl)),
          dth = (dr - dl) / wheelDistance;

    dx = R * sin(dth + th) - R *sin(th);
    dy = -(R * cos(dth + th) + R * cos(th));
    //th = boundAngle(th + wd);
  }
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
  ros::init(argc, argv, "odom_calc");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe<std_msgs::String>("odom_pub", 50, odomCallback);

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
