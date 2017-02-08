#include "gtest/gtest.h"
#include "grizzly_motion/encoders_monitor.h"

#include "ros/console.h"

class EncodersMonitorFixture : public ::testing::Test 
{
public:
  virtual void SetUp() {
    ros::Time::init();
    start = ros::Time::now();
    em.encoders_timeout = ros::Duration(0.05);
    em.encoder_fault_time_to_failure = ros::Duration(0.5);
    em.encoder_speed_error_diff_threshold = 0.5;

    drive.reset(new grizzly_msgs::Drive);
    enc.reset(new grizzly_msgs::Drive);
  }

  void setStamps(ros::Time t) {
    drive->header.stamp = t;
    enc->header.stamp = t;
  }

  ros::Time start;
  EncodersMonitor em;

  grizzly_msgs::DrivePtr drive;
  grizzly_msgs::DrivePtr enc;
};

TEST_F(EncodersMonitorFixture, no_fault_when_encoders_match_commanded)
{
  // 100 seconds in ROS time.
  for (int step = 0; step < 2000; step++)
  {
    setStamps(start + ros::Duration(0.05) * step);
    VectorDrive vec(step, 2000 - step, step, 2000 - step);
    grizzly_msgs::fillDriveMsgFromVector(vec, drive.get());
    em.encodersCallback(drive);
    em.driveCallback(drive);
    EXPECT_TRUE(em.ok());
  } 
}

TEST_F(EncodersMonitorFixture, no_fault_when_two_encoders_stall)
{
  // 100 seconds in ROS time.
  for (int step = 0; step < 2000; step++)
  {
    setStamps(start + ros::Duration(0.05) * step);
    VectorDrive vec(step, step, step, step);
    grizzly_msgs::fillDriveMsgFromVector(vec, drive.get());
    if (step > 1500) {
      // Simulate two motors stalling under load.
      vec[2] = 0;
      vec[3] = 0;
    }
    grizzly_msgs::fillDriveMsgFromVector(vec, enc.get());
    em.driveCallback(drive);
    em.encodersCallback(enc);
    EXPECT_TRUE(em.ok());
  } 
}

TEST_F(EncodersMonitorFixture, no_fault_in_partial_stall)
{
  // 100 seconds in ROS time.
  for (int step = 0; step < 2000; step++)
  {
    setStamps(start + ros::Duration(0.05) * step);
    VectorDrive vec(step, -step, step, -step);
    grizzly_msgs::fillDriveMsgFromVector(vec, drive.get());
    if (step > 1500) {
      // Simulate partial stall on one wheel.
      vec[2] = 10;
    }
    grizzly_msgs::fillDriveMsgFromVector(vec, enc.get());
    em.driveCallback(drive);
    em.encodersCallback(enc);
    EXPECT_TRUE(em.ok());
  } 
}

TEST_F(EncodersMonitorFixture, no_fault_brief_dropout)
{
  // 100 seconds in ROS time.
  for (int step = 0; step < 2000; step++)
  {
    setStamps(start + ros::Duration(0.05) * step);
    VectorDrive vec(step, -step, step, -step);
    grizzly_msgs::fillDriveMsgFromVector(vec, drive.get());
    if (step > 1500 && step < 1508) {
      // Momentary encoder loss.
      vec[2] = 0;
    }
    grizzly_msgs::fillDriveMsgFromVector(vec, enc.get());
    em.driveCallback(drive);
    em.encodersCallback(enc);
    EXPECT_TRUE(em.ok());
  } 
}

TEST_F(EncodersMonitorFixture, fault_when_one_encoder_fails)
{
  // 100 seconds in ROS time.
  for (int step = 0; step < 2000; step++)
  {
    setStamps(start + ros::Duration(0.05) * step);
    VectorDrive vec(step, -step, step, -step);
    grizzly_msgs::fillDriveMsgFromVector(vec, drive.get());
    if (step > 1500) {
      // Simulate encoder failure on one wheel.
      vec[2] = 0.0001;
    }
    grizzly_msgs::fillDriveMsgFromVector(vec, enc.get());
    em.driveCallback(drive);
    em.encodersCallback(enc);
    bool res = em.ok();
    if (step < 1510) EXPECT_TRUE(res);
    if (step > 1510) EXPECT_FALSE(res);
  } 
}

TEST_F(EncodersMonitorFixture, timeout)
{
  // Before the first encoders is received, should definitely be not-ok.
  EXPECT_FALSE(em.ok());

  // Receive one which is clearly out of date, still not ok.
  grizzly_msgs::DrivePtr msg(new grizzly_msgs::Drive);
  msg->header.stamp = ros::Time::now() - ros::Duration(1.0);
  em.encodersCallback(msg);
  EXPECT_FALSE(em.ok());

  // Receive an up-to-date one, should now be okay.
  msg->header.stamp = ros::Time::now();
  em.encodersCallback(msg);
  EXPECT_TRUE(em.ok());

  // Out of date again.
  msg->header.stamp = ros::Time::now() - ros::Duration(0.5);
  em.encodersCallback(msg);
  EXPECT_FALSE(em.ok());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
