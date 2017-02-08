/**
Software License Agreement (BSD)

\file      grizzly_msgs/eigen.h
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

#ifndef GRIZZLY_MSGS_EIGEN_H
#define GRIZZLY_MSGS_EIGEN_H

#include "grizzly_msgs/Drive.h"
#include <Eigen/Core>
#include <stdexcept>
#include <string>

typedef Eigen::Vector4f VectorDrive;
static const float default_stationary_threshold(0.001);

namespace grizzly_msgs
{

namespace Drives
{
  enum {
    FrontLeft = 0,
    FrontRight = 1,
    RearLeft = 2,
    RearRight = 3
  };
}

static inline VectorDrive vectorFromDriveMsg(const Drive& msg)
{
  return VectorDrive(
      msg.front_left,
      msg.front_right,
      msg.rear_left,
      msg.rear_right);
}

static inline void fillDriveMsgFromVector(const VectorDrive& vec, Drive* msg)
{
  msg->front_left = vec[Drives::FrontLeft];
  msg->front_right = vec[Drives::FrontRight];
  msg->rear_left = vec[Drives::RearLeft];
  msg->rear_right = vec[Drives::RearRight];
}

static inline Drive driveMsgFromVector(const VectorDrive& vec)
{
  Drive msg;
  fillDriveMsgFromVector(vec, &msg);
  return msg;
}

static inline std::string nameFromDriveIndex(VectorDrive::Index field)
{
  switch(field)
  {
    case Drives::FrontLeft: return "front_left";
    case Drives::FrontRight: return "front_right";
    case Drives::RearLeft: return "rear_left";
    case Drives::RearRight: return "rear_right";
    default:
      throw std::out_of_range("Passed field number not in range 0..3");
  }
}

static inline bool isStationary(const VectorDrive& vec, float threshold=default_stationary_threshold)
{ 
  return vec.cwiseAbs().maxCoeff() < threshold;
}

static inline bool isStationary(const Drive& msg, float threshold=default_stationary_threshold)
{
  return isStationary(vectorFromDriveMsg(msg), threshold);
}

}

#endif
