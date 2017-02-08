/**
Software License Agreement (BSD)

\file      motion_safety.h
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
#include "grizzly_motion/change_limiter.h"

#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"

using boost::shared_ptr;

namespace grizzly_msgs {
ROS_DECLARE_MESSAGE(Drive);
ROS_DECLARE_MESSAGE(RawStatus);
}

namespace std_msgs {
ROS_DECLARE_MESSAGE(Bool);
}

namespace diagnostic_updater {
class Updater;
class HeaderlessTopicDiagnostic;
}

class EncodersMonitor;
class MotorsMonitor;

typedef ChangeLimiter<grizzly_msgs::Drive> DriveChangeLimiter;

namespace MotionStates
{
  enum MotionState
  {
    // State of non-error, non-moving vehicle. As soon as the vehicle has been
    // stationary for a brief period, the Moving state transitions to this one.
    // Certain errors that will trigger an MCU estop in the Moving or Startup
    // states will not do so in the Stopped state.
    Stopped,

    // State when movement has been requested but is being held pending the
    // mandatory delay period (and accompanying ambience).
    Starting,

    // State when vehicle is moving without error.
    Moving,

    // State when an estop has been triggered (by the MCU or otherwise). Wait for
    // the encoders to report the vehicle as stationary, with no motion being
    // commanded, then transition to Stopped.
    PendingStopped,

    // State when vehicle is faulted and will remain so until ROS is restarted.
    // Assert and hold an MCU estop.
    Fault
  };
}
typedef MotionStates::MotionState MotionState;

class MotionSafety
{
public:
  MotionSafety() {}
  MotionSafety(ros::NodeHandle* nh);

  bool isEstopped();
  void setStop(const std::string reason, bool estop, bool fault);
  void checkFaults();

protected:
  ros::NodeHandle* nh_;

  // Main watchdog function, which takes care of calling the other components
  // to assess overall system health (and thus safety of driving), and manage
  // state transitions.
  void watchdogCallback(const ros::TimerEvent&);
  ros::Timer watchdog_timer_;
  MotionState state_;
  std::string reason_;

  // Keeps the time of the last received command message with a non-stationary
  // movement request to at least one of the wheels.
  ros::Time last_commanded_movement_time_;
  ros::Time last_non_precharge_time_;

  // Duration of time to spend in the Starting phase.
  ros::Duration starting_duration_;

  // Tracks the absolute time when we will transition from Starting to Moving,
  // as that's a timed transition.
  ros::Time transition_to_moving_time_;

  // Topics directly monitored in this class.
  void driveCallback(const grizzly_msgs::DriveConstPtr&);
  void estopCallback(const std_msgs::BoolConstPtr&);
  void mcuStatusCallback(const grizzly_msgs::RawStatusConstPtr&);
  ros::Subscriber sub_drive_, sub_mcu_status_, sub_user_estop_; 
  grizzly_msgs::RawStatusConstPtr last_mcu_status_; 

  // Publish cmd_drive through to safe_cmd_drive.
  ros::Publisher pub_safe_drive_;
    
  // Communication to the MCU.
  ros::Publisher pub_ambience_;
  ros::Publisher pub_estop_;

  // Publish diagnostics for the whole node from here.
  shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  void diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // Monitor the frequency of the MCU status and incoming cmd_drive messages for
  // acceptable range.
  double expected_mcu_status_frequency_;
  double min_cmd_drive_freq_, max_cmd_drive_freq_;
  shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> diag_mcu_status_freq_, diag_cmd_drive_freq_;

  // Separate class for monitoring encoders for sanity.
  shared_ptr<EncodersMonitor> encoders_monitor_; 
  shared_ptr<MotorsMonitor> motor_drivers_monitor_;
  shared_ptr<DriveChangeLimiter> accel_limiters_[4];

  double width_;
  double radius_;
  double max_accel_;
};


