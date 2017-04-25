#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Encoder.h>
#include <MPU6050.h>
#include <math.h>
#include <geometry_msgs/Twist.h>



std_msgs::String enc_msg;
ros::Publisher odom_pub("odom_pub", &enc_msg);

std_msgs::String imu_msg;
ros::Publisher pub_imu("imu_msg", &imu_msg);

ros::NodeHandle nh;

Encoder encl1(5, 6);
Encoder encl2(3, 4);
Encoder encr1(52, 50);
Encoder encr2(7, 8);

MPU6050 mpu;

char imu_str[150];
char odom_str[50];


double linear, rotation,gain = 40;

void TwistCb( const geometry_msgs::Twist & msg) {
  linear = msg.linear.x * gain;
  rotation = msg.angular.z * gain;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &TwistCb );


void setup() {


  pinMode(12, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(22, OUTPUT);

  nh.initNode();
  nh.advertise(odom_pub);
  nh.advertise(pub_imu);
  nh.subscribe(sub);

  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
    {
     Serial.println("Could not find a valid MPU6050 sensor, check wiring!");

    }

   digitalWrite(13, LOW);

}



void loop() {
  
  long leftEn1 = encl1.read();
  long leftEn2 = encl2.read();
  long rightEn1 = encr1.read();
  long rightEn2 = encr2.read();
  /*digitalWrite(13,HIGH);
  delay(10);
  digitalWrite(13,LOW);*/
  char l1[10];
  dtostrf(leftEn1, 3, 2, l1);
  char l2[10];
  dtostrf(leftEn2, 3, 2, l2);
  char r1[10];
  dtostrf(rightEn1, 3, 2, r1);
  char r2[10];
  dtostrf(rightEn2, 3, 2, r2);
  
  String s = String(l1) + "," + String(l2) + "," + String(r1) +  "," + String(r2);
  s.toCharArray(odom_str, 50);
  enc_msg.data = odom_str;
  
  odom_pub.publish(&enc_msg);



  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
    
    
  char acc_x[50];
  dtostrf(rawAccel.XAxis, 3, 2, acc_x);

  char acc_y[50];
  dtostrf(rawAccel.YAxis, 3, 2, acc_y);

  char acc_z[50];
  dtostrf(rawAccel.ZAxis, 3, 2, acc_z);

  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();

    
  char gyro_x[50];
  dtostrf(normGyro.XAxis, 3, 2, gyro_x);

  char gyro_y[50];
  dtostrf(normGyro.YAxis, 3, 2, gyro_y);

  char gyro_z[50];
  dtostrf(normGyro.ZAxis, 3, 2, gyro_z);

  String s1 = String(acc_x) + "," + String(acc_y) + "," + String(acc_z) + "," + String(gyro_x) + "," + String(gyro_y) + "," + String(gyro_z);
  s1.toCharArray(imu_str, 150);
  imu_msg.data = imu_str;
  pub_imu.publish(&imu_msg);


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
      digitalWrite(29, LOW);
      digitalWrite(27, HIGH);
      digitalWrite(25, HIGH);
      digitalWrite(22, LOW);
    } else if (linear < 0) {
      digitalWrite(29, HIGH);
      digitalWrite(27, LOW);
      digitalWrite(25, LOW);
      digitalWrite(22, HIGH);
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
