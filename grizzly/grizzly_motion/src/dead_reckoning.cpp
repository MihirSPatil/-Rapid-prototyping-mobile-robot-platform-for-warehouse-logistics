/**
   Software License Agreement (BSD)

   \file      dead_reckoning.cpp
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
#include "tf/transform_datatypes.h"
#include "grizzly_motion/dead_reckoning.h"
#include <angles/angles.h>

using Eigen::Vector2f;

/**
 * Open-loop mapping between linear/angular commands and individual wheel speed
 * commands. Currently very naive, but in the future may provide some further
 * intelligence, though not closed-loop control.
 */

bool DeadReckoning::next(const grizzly_msgs::DriveConstPtr& encoders, nav_msgs::Odometry *odom, sensor_msgs::JointState *joints) {
  bool success = false;
  VectorDrive wheels_speed_ = grizzly_msgs::vectorFromDriveMsg(*encoders);
  uint8_t wheels_num_ = wheels_speed_.size();
  uint8_t joints_num_ = wheels_num_; // add front_axle_joint

  if (!initialize) {
    last_joint_pos_.resize(joints_num_, 0.0);
    last_time_ = encoders->header.stamp;
    yaw_ = 0.0;
    initialize = true;
    ROS_INFO("initialization complete.");
  }

  // Angular velocity per-side in rad/s, velocity per-size in m/s
  Vector2f avg_rotations((encoders->front_left + encoders->rear_left) / 2,
                         (encoders->front_right + encoders->rear_right) / 2);
  Vector2f vels = avg_rotations * radius_;

  joints->position.resize(joints_num_, 0.0);
  joints->effort.resize(joints_num_, 0);

  for (uint8_t i = 0; i < wheels_num_; i++) {
    joints->velocity.push_back(wheels_speed_[i]);
    joints->name.push_back("joint_" + grizzly_msgs::nameFromDriveIndex(i) + "_wheel");
  }

  joints->velocity.push_back(0);
  //joints->name.push_back("front_axle_joint");


  ros::Duration dt = encoders->header.stamp - last_time_;

  if (dt <= max_dt_) {
    // Integrate position based on previous yaw and speed
    position_.x += cos(yaw_) * twist_.linear.x * dt.toSec();
    position_.y += sin(yaw_) * twist_.linear.x * dt.toSec();

    // Update heading by integrating previous angular velocity.
    yaw_ += twist_.angular.z * dt.toSec();

    // Update linear and angular velocity
    twist_.linear.x = vels.mean();
    twist_.angular.z = (vels[1] - vels[0]) / width_;

    // Timestamp from encoder message, set frames correctly.
    odom->header = encoders->header;
    odom->header.frame_id = "odom";
    odom->child_frame_id = "base_link";
    odom->pose.pose.position = position_;
    odom->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_);
    odom->twist.twist = twist_;

    Eigen::Map<Eigen::MatrixXd> poseCov(odom->pose.covariance.data(), 6, 6);
    Eigen::Map<Eigen::MatrixXd> twistCov(odom->twist.covariance.data(), 6, 6);

    if (fabs(twist_.linear.x) <= 1e-3 && fabs(twist_.angular.z) <= 1e-3) {
      poseCov = ODOM_POSE_COVAR_NOMOVE;
      twistCov = ODOM_TWIST_COVAR_NOMOVE;
      joints->position = last_joint_pos_;
    } else {
      poseCov = ODOM_POSE_COVAR_MOTION;
      twistCov = ODOM_TWIST_COVAR_MOTION;

      // update joint's position
      for (uint8_t i = 0; i < wheels_num_; i++) {
        joints->position[i] = last_joint_pos_[i] + dt.toSec() * joints->velocity[i];
        joints->position[i] = angles::normalize_angle(joints->position[i]);
      }
    }

    joints->header = encoders->header;
    success = true;
  } else {
    static bool first = true;
    ROS_WARN_COND(first, "Gap between encoders messages is too large, no odom generated.");
    first = false;
  }

  last_time_ = encoders->header.stamp;
  last_joint_pos_ = joints->position;
  last_vels_ = vels;
  return success;
}
