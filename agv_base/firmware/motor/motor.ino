/*
   rosserial Subscriber Example
*/

#include <ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

double linear, rotation;
double gain = 30;
double invert = -1;

void TwistCb( const geometry_msgs::Twist & msg) {
  linear = msg.linear.x * gain;
  rotation = msg.angular.z * gain;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &TwistCb );

void setup()
{
  pinMode(12, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(22, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  double mag = 0;
  if (abs(linear) > 0) {
    mag = abs(linear);
  } 
  else if (abs(rotation) > 0) {
    mag = abs(rotation);
  }
  analogWrite(12, mag);
  analogWrite(11, mag);
  analogWrite(10, mag);
  analogWrite(9, mag);

  if (rotation == 0) {
    if (linear > 0) {
      digitalWrite(29, HIGH);
      digitalWrite(27, HIGH);
      digitalWrite(25, HIGH);
      digitalWrite(22, HIGH);
    } else if (linear < 0) {
      digitalWrite(29, LOW);
      digitalWrite(27, LOW);
      digitalWrite(25, LOW);
      digitalWrite(22, LOW);
    }
  } else {
    if (rotation > 0) {
      digitalWrite(29, HIGH);
      digitalWrite(27, HIGH);
      digitalWrite(25, LOW);
      digitalWrite(22, LOW);
    } else if (rotation < 0) {
      digitalWrite(29, LOW);
      digitalWrite(27, LOW);
      digitalWrite(25, HIGH);
      digitalWrite(22, HIGH);
    }
  }
  nh.spinOnce();
}


