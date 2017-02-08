/**
Software License Agreement (BSD)

\file      change_limiter.h
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

#ifndef GRIZZLY_MOTION_CHANGE_LIMITER_H
#define GRIZZLY_MOTION_CHANGE_LIMITER_H

template<class Msg>
class ChangeLimiter
{
public:
  ChangeLimiter(double max_change_per_second, float Msg::*field_ptr)
    : max_change_per_second_(max_change_per_second), field_(field_ptr)
  {
  }

  void apply(const Msg* msg_in, Msg* msg_out)
  {
    double diff_secs = (msg_in->header.stamp - last_msg_.header.stamp).toSec();
    if (diff_secs <= 0) {
      ROS_ERROR("Change limiter passed sequential drive messages with zero or negative timestamp differential.");
      return;
    }
    if (diff_secs > 0.1) {
      // If time difference is too great, make output field zero.
      msg_out->*field_ = 0;
    } else {
      double diff_value = msg_in->*field_ - last_msg_.*field_;
      double max_change = max_change_per_second_ * diff_secs;
      if (fabs(diff_value) < max_change) {
        msg_out->*field_ = msg_in->*field_;
      } else if (diff_value > 0) {
        msg_out->*field_ = last_msg_.*field_ + max_change;
      } else {
        msg_out->*field_ = last_msg_.*field_ - max_change;
      }
    }
    last_msg_ = *msg_out;
  }

  void setMaxChange(double max_change_per_second)
  {
    max_change_per_second_ = max_change_per_second;
  }

protected:
  double max_change_per_second_;
  float Msg::*field_;
  Msg last_msg_;
};

#endif
