/**
   Software License Agreement (BSD)

   \file      motion_safety_node.cpp
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

#include "grizzly_motion/motion_safety.h"
#include "grizzly_motion/motors_monitor.h"
#include "grizzly_motion/encoders_monitor.h"
#include "grizzly_motion/change_limiter.h"

#include "grizzly_msgs/Ambience.h"
#include "grizzly_msgs/Drive.h"
#include "grizzly_msgs/RawStatus.h"
#include "std_msgs/Bool.h"

using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::FrequencyStatusParam;
using diagnostic_updater::HeaderlessTopicDiagnostic;
using diagnostic_updater::Updater;

MotionSafety::MotionSafety(ros::NodeHandle* nh)
        : nh_(nh), state_(MotionStates::Stopped)
{
        ros::param::get("vehicle_width", width_);
        ros::param::get("wheel_radius", radius_);
        ros::param::get("max_acceleration", max_accel_); // m/s^2
        starting_duration_ = ros::Duration(2.0);

        // Drive pass-through
        sub_drive_ = nh_->subscribe("cmd_drive", 1, &MotionSafety::driveCallback, this);
        pub_safe_drive_ = nh_->advertise<grizzly_msgs::Drive>("safe_cmd_drive", 1);

        // MCU interface
        pub_ambience_ = nh_->advertise<grizzly_msgs::Ambience>("mcu/ambience", 1);
        pub_estop_ = nh_->advertise<std_msgs::Bool>("mcu/estop", 1);
        sub_mcu_status_ = nh_->subscribe("mcu/status", 1, &MotionSafety::mcuStatusCallback, this);
        watchdog_timer_ = nh_->createTimer(ros::Duration(0.05), &MotionSafety::watchdogCallback, this);

        // Drive pass-through
        sub_user_estop_ = nh_->subscribe("estop", 1, &MotionSafety::estopCallback, this);

        // Set up the diagnostic updater
        diagnostic_updater_.reset(new Updater());
        diagnostic_updater_->setHardwareID("grizzly");

        // Frequency report on statuses coming from the MCU.
        expected_mcu_status_frequency_ = 50;
        diag_mcu_status_freq_.reset(new HeaderlessTopicDiagnostic("MCU status", *diagnostic_updater_,
                                                                  FrequencyStatusParam(&expected_mcu_status_frequency_, &expected_mcu_status_frequency_, 0.01, 5.0)));

        // Frequency report on inbound drive messages.
        min_cmd_drive_freq_ = 10;
        max_cmd_drive_freq_ = 10;
        diag_cmd_drive_freq_.reset(new HeaderlessTopicDiagnostic("Drive command", *diagnostic_updater_,
                                                                 FrequencyStatusParam(&min_cmd_drive_freq_, &max_cmd_drive_freq_, 0.01, 5.0)));

        diagnostic_updater_->add("Motion Safety", this, &MotionSafety::diagnostic);

        // More specialized monitoring for encoders.
        encoders_monitor_.reset(new EncodersMonitor(nh_));
        motor_drivers_monitor_.reset(new MotorsMonitor(nh_));
        diagnostic_updater_->add("Encoders", encoders_monitor_.get(), &EncodersMonitor::diagnostic);

        // Rate-of-change limiter for wheel speed commands.
        accel_limiters_[0].reset(new DriveChangeLimiter(max_accel_ / radius_, &grizzly_msgs::Drive::front_left));
        accel_limiters_[1].reset(new DriveChangeLimiter(max_accel_ / radius_, &grizzly_msgs::Drive::front_right));
        accel_limiters_[2].reset(new DriveChangeLimiter(max_accel_ / radius_, &grizzly_msgs::Drive::rear_left));
        accel_limiters_[3].reset(new DriveChangeLimiter(max_accel_ / radius_, &grizzly_msgs::Drive::rear_right));
}

void MotionSafety::setStop(const std::string reason, bool estop=false, bool fault=false)
{
        ROS_ERROR_STREAM("Stop transition occurring for reason: " << reason);
        if (estop || fault)
        {
                std_msgs::Bool msg;
                msg.data = true;
                pub_estop_.publish(msg);
                state_ = fault ? MotionStates::Fault : MotionStates::PendingStopped;
        }
        else
        {
                state_ = MotionStates::Stopped;
        }
        reason_ = reason;
}

void MotionSafety::checkFaults()
{
        // Precharge fault
        if (last_mcu_status_)
        {
                if (!(last_mcu_status_->error & grizzly_msgs::RawStatus::ERROR_BRK_DET))
                        last_non_precharge_time_ = last_mcu_status_->header.stamp;
                if (last_mcu_status_->header.stamp - last_non_precharge_time_ > ros::Duration(4.0))
                {
                        // Pre-charging in excess of 4 seconds signals a serious electrical failure.
                        setStop("Precharge persisted for more than four seconds.", true, true);
                }
        }

        // Encoders fault
        if (encoders_monitor_->detectFailedEncoder())
        {
                setStop("Encoder failure detected.", true, true);
        }

        // Motor driver fault

}

void MotionSafety::watchdogCallback(const ros::TimerEvent&)
{
        // Messages to be published to the MCU at each run of this function.
        grizzly_msgs::Ambience ambience;
        std_msgs::Bool estop;
        estop.data = false;

        checkFaults();

        bool encoders_ok = encoders_monitor_->ok();
        bool motor_controllers_ok = motor_drivers_monitor_->ok();

        if (state_ == MotionStates::Stopped)
        {
                if (ros::Time::now() - last_commanded_movement_time_ < ros::Duration(0.1) &&
                    !isEstopped())
                {
                        state_ = MotionStates::Starting;
                        transition_to_moving_time_ = ros::Time::now() + starting_duration_;
                }
        }

        if (state_ == MotionStates::Starting)
        {
                ambience.beacon = ambience.headlight = ambience.taillight = ambience.beep =
                                                                                    grizzly_msgs::Ambience::PATTERN_DFLASH;
                if (ros::Time::now() > transition_to_moving_time_ && !last_mcu_status_->error)
                        state_ = MotionStates::Moving;
                if (ros::Time::now() - last_commanded_movement_time_ > ros::Duration(0.1))
                        setStop("Command messages stale.");
                if (!encoders_ok)
                        setStop("Encoders not okay.");
                //if(!motor_controllers_ok)
                //  setStop("Motor controllers not okay.");
                if (isEstopped())
                        setStop("External estop.");
        }

        if (state_ == MotionStates::Moving)
        {
                ambience.beacon = ambience.beep =
                                          grizzly_msgs::Ambience::PATTERN_FLASH;
                if (!encoders_ok)
                        setStop("Encoders not okay.", true, true);
                if (isEstopped())
                        setStop("External estop.");
                //if (!motor_controllers_ok)
                //  setStop("Motor controllers not okay.", true);

                // There are two levels of timeout at work when Grizzly is in motion. The immediate
                // timeout is 110ms, enforced in the Roboteq firmware:
                //   https://github.com/g/roboteq/blob/master/roboteq_driver/mbs/script.mbs#L10
                //
                // This is the secondary timeout, which switches the statemachine back to the Stopped
                // state, which requires the starting_duration_ wait before moving again. This delay
                // is only necessary when the vehicle has been stationary/uncommanded for a short period.
                if (ros::Time::now() - last_commanded_movement_time_ > ros::Duration(3.0))
                        setStop("Command messages stale.");
        }

        if (state_ == MotionStates::PendingStopped)
        {
                estop.data = true;
                // Three conditions to exit this state:
                //   - Vehicle must be nonmoving, according to the encoders
                //   - Vehicle must not be receiving motion commands (none in the last second)
                //   - Vehicle must be in the estop state, requiring a reset
                if (!encoders_monitor_->moving() &&
                    ros::Time::now() - last_commanded_movement_time_ > ros::Duration(1.0) &&
                    isEstopped())
                {
                        state_ = MotionStates::Stopped;
                }
        }

        if (state_ == MotionStates::Fault)
        {
                estop.data = true;
        }

        diagnostic_updater_->update();
        pub_ambience_.publish(ambience);
        pub_estop_.publish(estop);
}

void MotionSafety::estopCallback(const std_msgs::BoolConstPtr& msg)
{
        if (msg->data == true) {
                // Publish an immediate message, then also signal the state transition.
                pub_estop_.publish(msg);
                if (state_ != MotionStates::Fault) state_ = MotionStates::PendingStopped;
        }
}

/**
 * Manages a pass-through of Grizzly Drive messages, ensuring that the appropriate
 * delays are observed before allowing the chassis to move, including activating
 * the chassis lights and beeper. Also monitors encoders for possible failure.
 */
void MotionSafety::driveCallback(const grizzly_msgs::DriveConstPtr& drive_commanded)
{
        diag_cmd_drive_freq_->tick();

        // This signals the main loop function to begin a transition to the Moving state.
        if (!grizzly_msgs::isStationary(*drive_commanded.get()))
                last_commanded_movement_time_ = drive_commanded->header.stamp;

        grizzly_msgs::Drive drive_safe;
        drive_safe.header = drive_commanded->header;

        if (state_ == MotionStates::Moving)
        {
                for (int wheel = 0; wheel < 4; wheel++)
                        accel_limiters_[wheel]->apply(drive_commanded.get(), &drive_safe);
        }

        pub_safe_drive_.publish(drive_safe);
}

bool MotionSafety::isEstopped()
{
        return last_mcu_status_ &&
               (last_mcu_status_->error & grizzly_msgs::RawStatus::ERROR_ESTOP_RESET);
}

void MotionSafety::mcuStatusCallback(const grizzly_msgs::RawStatusConstPtr& status)
{
        diag_mcu_status_freq_->tick();
        last_mcu_status_ = status;
}

void MotionSafety::diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
        std::string state_str;
        int level = 0;
        switch(state_)
        {
        case MotionStates::Stopped:
                state_str = "Stopped";
                break;
        case MotionStates::Starting:
                state_str = "Starting";
                reason_.clear();
                break;
        case MotionStates::Moving:
                state_str = "Moving";
                reason_.clear();
                break;
        case MotionStates::PendingStopped:
                state_str = "PendingStopped";
                level = 1;
                break;
        case MotionStates::Fault:
                state_str = "Faulted";
                level = 2;
                break;
        default:
                state_str = "Unknown";
                level = 2;
                break;
        }
        stat.summaryf(level, "Vehicle State %s", state_str.c_str());
        if (!reason_.empty()) {
                stat.summaryf(level, "Vehicle State %s (Reason: %s)", state_str.c_str(), reason_.c_str());
        }
        stat.add("reason", reason_);
        stat.add("state", state_);
        stat.add("last move command (seconds)", (ros::Time::now() - last_commanded_movement_time_).toSec());
        stat.add("vehicle in motion", encoders_monitor_->moving());
}

/**
 * Node entry point.
 */
int main(int argc, char ** argv)
{
        ros::init(argc, argv, "grizzly_motion_safety");
        ros::NodeHandle nh("");
        MotionSafety ms(&nh);
        ros::spin();
}
