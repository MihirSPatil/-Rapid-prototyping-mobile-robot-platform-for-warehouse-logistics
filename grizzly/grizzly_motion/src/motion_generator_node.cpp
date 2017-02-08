/**
Software License Agreement (BSD)

\file      motion_generator.cpp
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <grizzly_msgs/Drive.h>

/**
 * Manages a single subscription to Twist messages, produces a Grizzly Drive
 * message with the appropriate individual wheel speeds.
 */
class MotionGenerator
{
public:
  MotionGenerator() : nh_("")
  {
    ros::param::get("vehicle_width", width_);
    ros::param::get("wheel_radius", radius_);

    pub_ = nh_.advertise<grizzly_msgs::Drive>("cmd_drive", 1);
    sub_ = nh_.subscribe("cmd_vel", 1, &MotionGenerator::twist_callback, this);
  }

protected:
  void twist_callback(const geometry_msgs::TwistConstPtr&);

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double width_, radius_;
};

/**
 * Open-loop mapping between linear/angular commands and individual wheel speed
 * commands. Currently very naive, but in the future may provide some further
 * intelligence, though not closed-loop control.
 */
void MotionGenerator::twist_callback(const geometry_msgs::TwistConstPtr& twist)
{
  double right_speed = twist->linear.x + twist->angular.z * (width_ / 2);
  double left_speed = twist->linear.x - twist->angular.z * (width_ / 2);

  grizzly_msgs::Drive drive;
  drive.header.stamp = ros::Time::now();
  drive.front_left = drive.rear_left = left_speed / radius_;
  drive.front_right = drive.rear_right = right_speed / radius_;
  pub_.publish(drive);
}

/**
 * Main entry point.
 */
int main (int argc, char ** argv)
{
  ros::init(argc, argv, "grizzly_motion_generator"); 
  MotionGenerator mg;
  ros::spin();
}
