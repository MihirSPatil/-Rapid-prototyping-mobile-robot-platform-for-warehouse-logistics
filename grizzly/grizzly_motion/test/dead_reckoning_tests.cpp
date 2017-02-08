#include "gtest/gtest.h"
#include "grizzly_motion/dead_reckoning.h"

#include "ros/console.h"

TEST(DeadReckoning, simple_straight)
{
  ros::Time::init();
  ros::Time start = ros::Time::now();
  double wheel_radius = 0.2;

  DeadReckoning dr(1.0, wheel_radius);
  nav_msgs::Odometry odom;
  sensor_msgs::JointState joints;
  grizzly_msgs::DrivePtr encoders(new grizzly_msgs::Drive);
  encoders->header.stamp = start;
  encoders->front_left = encoders->rear_left = 1.1;
  encoders->front_right = encoders->rear_right = 1.1;  // rad/s
  dr.next(encoders, &odom, &joints);

  // 10 seconds of travel at 50 Hz
  for(int step = 0; step < 500; step++)
  {
    encoders->header.stamp += ros::Duration(0.02);
    EXPECT_TRUE(dr.next(encoders, &odom, &joints));
  } 

  // Expected travel is time * rad/s * wheel radius
  EXPECT_NEAR(10 * 1.1 * wheel_radius, odom.pose.pose.position.x, 0.005);
  EXPECT_NEAR(0.0, odom.pose.pose.position.y, 0.005);
  EXPECT_EQ(odom.header.stamp, encoders->header.stamp);
}

TEST(DeadReckoning, curved_path)
{
  ros::Time::init();
  ros::Time start = ros::Time::now();
  double wheel_radius = 0.1;
  double vehicle_width = 1.0;

  // The travels specified here drive a 1m-wide robot around 1/4 of a 3m circle. The
  // left wheel has the longer path, so the path curved into quadrant 4.
  double left_travel = 3.5*2 * M_PI / 4;
  double right_travel = 2.5*2 * M_PI / 4;

  DeadReckoning dr(vehicle_width, wheel_radius);
  nav_msgs::Odometry odom;
  sensor_msgs::JointState joints;
  grizzly_msgs::DrivePtr encoders(new grizzly_msgs::Drive);
  encoders->header.stamp = start;
  encoders->front_left = encoders->rear_left = left_travel / wheel_radius / 10.0;
  encoders->front_right = encoders->rear_right = right_travel / wheel_radius / 10.0;
  dr.next(encoders, &odom, &joints);

  // 10 seconds of travel
  for(int step = 0; step < 500; step++)
  {
    encoders->header.stamp += ros::Duration(0.02);
    EXPECT_TRUE(dr.next(encoders, &odom, &joints));
    ROS_DEBUG_STREAM("ENC " << encoders->front_left << " " << encoders->front_right);
    ROS_DEBUG_STREAM("POSE " << odom.pose.pose.position.x << " " << odom.pose.pose.position.y);
  } 

  // Check that the final position matches our expectations, given the wheel velocities
  // and timeframe which were commanded.
  EXPECT_NEAR(3, odom.pose.pose.position.x, 0.05);
  EXPECT_NEAR(-3, odom.pose.pose.position.y, 0.05);
  EXPECT_EQ(odom.header.stamp, encoders->header.stamp);
}

TEST(DeadReckoning, timeout)
{
  ros::Time::init();
  ros::Time start = ros::Time::now();
  
  nav_msgs::Odometry odom;
  sensor_msgs::JointState joints;
  grizzly_msgs::DrivePtr encoders(new grizzly_msgs::Drive);
  encoders->header.stamp = start;

  DeadReckoning dr(1.0, 1.0);
  dr.next(encoders, &odom, &joints);

  // This one should not time out.
  encoders->header.stamp += ros::Duration(0.09);
  EXPECT_TRUE(dr.next(encoders, &odom, &joints));

  // This one should.
  encoders->header.stamp += ros::Duration(0.11);
  EXPECT_FALSE(dr.next(encoders, &odom, &joints));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
