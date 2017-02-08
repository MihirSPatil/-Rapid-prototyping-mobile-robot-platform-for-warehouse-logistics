/**
Software License Agreement (BSD)

\file      motors_monitor.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
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

#include "grizzly_motion/motors_monitor.h"
#include <roboteq_msgs/Feedback.h>
#include <roboteq_msgs/Status.h>
#include <grizzly_msgs/eigen.h>
#include <boost/bind.hpp>


MotorsMonitor::MotorsMonitor(ros::NodeHandle* nh) : nh_("")
{

  double motors_timeout_seconds;
  ros::param::param<double>("~motors_timeout", motors_timeout_seconds, 0.31);
  motors_timeout_ = ros::Duration(motors_timeout_seconds);

  fb_sub_[0] = nh_.subscribe<roboteq_msgs::Feedback>("motors/front_right/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,(int)grizzly_msgs::Drives::FrontLeft));
  fb_sub_[1] = nh_.subscribe<roboteq_msgs::Feedback>("motors/front_left/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,(int)grizzly_msgs::Drives::FrontRight));
  fb_sub_[2] = nh_.subscribe<roboteq_msgs::Feedback>("motors/rear_right/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,(int)grizzly_msgs::Drives::RearLeft));
  fb_sub_[3] = nh_.subscribe<roboteq_msgs::Feedback>("motors/rear_left/feedback",1,boost::bind(&MotorsMonitor::motor_feedback,this,_1,(int)grizzly_msgs::Drives::RearRight));

  stat_sub_[0] = nh_.subscribe<roboteq_msgs::Status>("motors/front_right/status",1,boost::bind(&MotorsMonitor::motor_status,this,_1,(int)grizzly_msgs::Drives::FrontLeft));
  stat_sub_[1] = nh_.subscribe<roboteq_msgs::Status>("motors/front_left/status",1,boost::bind(&MotorsMonitor::motor_status,this,_1,(int)grizzly_msgs::Drives::FrontRight));
  stat_sub_[2] = nh_.subscribe<roboteq_msgs::Status>("motors/rear_right/status",1,boost::bind(&MotorsMonitor::motor_status,this,_1,(int)grizzly_msgs::Drives::RearLeft));
  stat_sub_[3] = nh_.subscribe<roboteq_msgs::Status>("motors/rear_left/status",1,boost::bind(&MotorsMonitor::motor_status,this,_1,(int)grizzly_msgs::Drives::RearRight));

  for (int i=grizzly_msgs::Drives::FrontLeft;i<=grizzly_msgs::Drives::RearRight;i++) {
    fault_level_[i] = 0;
  } 
}

template<class M>
static inline ros::Duration age(M msg) 
{
  return ros::Time::now() - msg->header.stamp;
}

/**
 * Called in the context of whether cmd_vels may be passed through as safe.
 */
bool MotorsMonitor::ok()
{
  // If we have no data, or its old, then definitely not okay.
  for (int i=grizzly_msgs::Drives::FrontLeft;i<=grizzly_msgs::Drives::RearRight;i++) {
    if (!last_received_status_[i] || !last_received_feedback_)
    {
      ROS_DEBUG_THROTTLE(1.0, "Motors not ok due to missing feedback or status.");
      return false;
    }
    if (age(last_received_status_[i]) > motors_timeout_)
    {
      ROS_DEBUG_THROTTLE(1.0, "Motors not ok due to timed-out status message.");
      return false;
    }
    fault_level_[i] = lookForSeriousFault (last_received_status_[i]->fault, i);
    if (fault_level_[i] > 0)
    {
      ROS_DEBUG_STREAM_THROTTLE(1.0, "Motors not ok from " << grizzly_msgs::nameFromDriveIndex(i) << 
                                     " driver due to fault code " << (int)last_received_status_[i]->fault);
      return false;
    }
  }

  return true;
}

int MotorsMonitor::lookForSeriousFault(uint8_t fault_code, const int motor_num)
{
  int error_level = 0;
  using roboteq_msgs::Status;
 
  //Warning level faults 
  if (fault_code & Status::FAULT_UNDERVOLTAGE) {
    error_level = 1;
  }

  if (fault_code & Status::FAULT_EMERGENCY_STOP) {
    error_level = 1;
  }

  if (fault_code & Status::FAULT_SEPEX_EXCITATION_FAULT) {
    error_level = 1;
  }

  if (fault_code & Status::FAULT_STARTUP_CONFIG_FAULT) {
    error_level = 1;
  }

  //More serious faults 
  if (fault_code & Status::FAULT_OVERHEAT) {
    error_level = 2;
  }

  if (fault_code & Status::FAULT_OVERVOLTAGE) {
    error_level = 2;
  }

  if (fault_code & Status::FAULT_SHORT_CIRCUIT) {
    error_level = 2;
  } 

  if (fault_code & Status::FAULT_MOSFET_FAILURE) {
    error_level = 2;
  }
  return error_level;
}

void MotorsMonitor::motor_feedback(const roboteq_msgs::FeedbackConstPtr msg, const int motor_num) {
  last_received_feedback_[motor_num] = msg;
}

void MotorsMonitor::motor_status(const roboteq_msgs::StatusConstPtr& msg, const int motor_num) {
  last_received_status_[motor_num] = msg;
}
