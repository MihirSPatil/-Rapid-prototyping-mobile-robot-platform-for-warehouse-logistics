/**
Software License Agreement (BSD)

\file      encoders_monitor.h
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

#ifndef GRIZZLY_MOTION_ENCODERS_MONITOR_H
#define GRIZZLY_MOTION_ENCODERS_MONITOR_H

#include <ros/ros.h>
#include <grizzly_msgs/Drive.h>
#include <grizzly_msgs/eigen.h>

namespace diagnostic_updater {
class DiagnosticStatusWrapper;
}

class EncodersMonitor {
public:
  EncodersMonitor() {}
  EncodersMonitor(ros::NodeHandle* nh);

  bool ok();
  bool moving();
  void diagnostic(diagnostic_updater::DiagnosticStatusWrapper&);

  bool detectFailedEncoder();
  bool detectFailedEncoderCandidate(VectorDrive::Index* candidate);

  // Callbacks receive inbound data
  void encodersCallback(const grizzly_msgs::DriveConstPtr&);
  void driveCallback(const grizzly_msgs::DriveConstPtr&);

  // Encoder data must be this fresh to not be considered out of date.
  ros::Duration encoders_timeout;

  // Threshold of rad/s difference between mean error and a singular wheel's error necessary
  // to mark it as suspicious.
  double encoder_speed_error_diff_threshold;

  // Time lag between suspecting a failure and taking action.
  ros::Duration encoder_fault_time_to_failure;

protected:
  ros::Subscriber sub_encoders_, sub_drive_;
  int failed_encoder_;

  grizzly_msgs::DriveConstPtr last_received_encoders_;
  grizzly_msgs::DriveConstPtr last_received_drive_;
  ros::Time time_of_last_nonsuspect_encoders_;
};

#endif
