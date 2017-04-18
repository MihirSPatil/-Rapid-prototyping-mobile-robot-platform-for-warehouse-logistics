#include <Arduino.h>

/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#include <ros.h>
#include <std_msgs/String.h>
#include <Encoder.h>
#include <MPU6050.h>



std_msgs::String enc_msg;
ros::Publisher odom_pub("odom_pub",&enc_msg);

std_msgs::String imu_msg;
ros::Publisher pub_imu("imu_msg", &imu_msg);

ros::NodeHandle nh;

Encoder encl1(5, 6);
Encoder encl2(7,8);
Encoder encr1(2,3);
Encoder encr2(9,10);



MPU6050 mpu;

char imu_str[150];
char odom_str[50];


void setup() {

  nh.initNode();
  nh.advertise(odom_pub);
  nh.advertise(pub_imu);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");

  }

  digitalWrite(13, LOW);

  //mpu.calibrateGyro();

  //mpu.setThreshold(3);

}



void loop() {
    long leftEn1 = encl1.read();
    long leftEn2 = encl2.read();
    long rightEn1 = encr1.read();
    long rightEn2 = encr2.read();

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
    enc_msg.data=odom_str;

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
    odom_pub.publish(&enc_msg);

    nh.spinOnce();
}
