/**
Software License Agreement (BSD)

\file      dead_reckoning_node.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros/ros.h"
#include "grizzly_motion/dead_reckoning.h"

void encodersCallback(DeadReckoning* dr, ros::Publisher* pub_odom, 
  ros::Publisher* pub_joints,  const grizzly_msgs::DriveConstPtr encoders)
{
  nav_msgs::Odometry odom;
  sensor_msgs::JointState joints;
  if (dr->next(encoders, &odom, &joints)) {
    pub_odom->publish(odom);
    pub_joints->publish(joints);
  }
}

/**
 * Main entry point.
 */
int main(int argc, char ** argv)
{
  double vehicle_width, wheel_radius;

  ros::init(argc, argv, "grizzly_dead_reckoning"); 
  ros::param::get("vehicle_width", vehicle_width);
  ros::param::get("wheel_radius", wheel_radius);

  DeadReckoning dr(vehicle_width, wheel_radius);
  ros::NodeHandle nh("");
  ros::Publisher pub_odom(nh.advertise<nav_msgs::Odometry>("odom", 1));
  ros::Publisher pub_joints(nh.advertise<sensor_msgs::JointState>("joint_states",1));
  ros::Subscriber sub(nh.subscribe<grizzly_msgs::Drive>("motors/encoders", 1, boost::bind(
    encodersCallback, &dr, &pub_odom, &pub_joints, _1)));

  ros::spin();
}
