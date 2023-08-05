#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include "ros/time.h"
#include <geometry_msgs/Point.h>

ros::NodeHandle nh;


const int pulPin1 = 3;
const int dirPin1 = 4;
const int enPin1 = 5;
const int pulPin2 = 6;
const int dirPin2 = 7;
const int enPin2 = 8;
const int pulPin3 = 9;
const int dirPin3 = 10;
const int enPin3 = 11;
float stp1;
float stp2;
float stp3;
//float deg[] = {80, 80, 80};
#define res 1.8
const float og=25;

void temp(const geometry_msgs::Point &msg)
{ 
  stp1 = msg.x/res;
  stp2 = msg.y/res;
  stp3 = msg.z/res; 

  origin();
  
//  //origin();
//  loop1();
//  loop2();
//  loop3();
//  delay(4000);
//  //back();
//  origin();
  //delay(5000);
  //back();
 
 
  
  
  //nh.loginfo("a");
  //return a
  //ROS_INFO("%f",&a)
  //nh.loginfo("%f",msg>-geometry_msgs::Point::float64 x);
  //ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[].x, msg->points[].y);
  //char result[8];
 // dtostrf(points[0].positions[0], 6, 2, result);
  //sprintf("points[0].positions[0] =%s", msg.point);
  //nh.loginfo(log_msg);
  //nh.loginfo(sub3.data);
  
}


ros::Subscriber<geometry_msgs::Point> sub3("mid_point_weed_final", &temp);

void setup()
{

  nh.initNode();
  nh.subscribe(sub3);

  pinMode(pulPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  digitalWrite(enPin1, LOW);

  pinMode(pulPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);
  digitalWrite(enPin2, LOW);

  pinMode(pulPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(enPin3, OUTPUT);
  digitalWrite(enPin3, LOW);



 // stp1 = deg[0] / res;
 // stp2 = deg[1] / res;
 // stp3 = deg[2] / res;
}

void loop()
{
  //loop1();
  //loop2();
  //loop3();
  //delay(3000);
  
  //  loopback1();
  //delay(5000);
  //origin();
  //hault();
  //delay(10000);
  //origin();
  nh.spinOnce();
  //delay(15000);

}

void loop1()
{
  digitalWrite(dirPin1, LOW);
  for (float x = 0; x < stp1; x++)
  {
    digitalWrite(pulPin1, HIGH);
    delayMicroseconds(5000);
    digitalWrite(pulPin1, LOW);
    delayMicroseconds(5000);
  }
}

void loop2()
{
  digitalWrite(dirPin2, HIGH);
  for (float x = 0; x < stp2; x++)
  {
    digitalWrite(pulPin2, HIGH);
    delayMicroseconds(5000);
    digitalWrite(pulPin2, LOW);
    delayMicroseconds(5000);
  }
}

void loop3()
{
  digitalWrite(dirPin3, LOW);
  for (float x = 0; x < stp3; x++)
  {
    digitalWrite(pulPin3, HIGH);
    delayMicroseconds(5000);
    digitalWrite(pulPin3, LOW);
    delayMicroseconds(5000);
  }
}

void hault()
{
  for (float x = 0; x < 400; x++)
  {
    digitalWrite(pulPin1, LOW);
    delayMicroseconds(5000);
    digitalWrite(pulPin1, LOW);
    delayMicroseconds(5000);
  }

  for (float x = 0; x < 400; x++)
  {
    digitalWrite(pulPin1, LOW);
    delayMicroseconds(5000);
    digitalWrite(pulPin1, LOW);
    delayMicroseconds(5000);
  }

  for (float x = 0; x < 400; x++)
  {
    digitalWrite(pulPin1, LOW);
    delayMicroseconds(5000);
    digitalWrite(pulPin1, LOW);
    delayMicroseconds(5000);
  }
}

void origin()
{
  Serial.println("HI");
  digitalWrite(dirPin1, HIGH);
  for (float x = 0; x < 80.0 ; x++)
  {
    digitalWrite(pulPin1, HIGH);
    delayMicroseconds(10000);
    digitalWrite(pulPin1, LOW);
    delayMicroseconds(10000);
  }

  digitalWrite(dirPin2, LOW);
  for (float x = 0; x < 80.0 ; x++)
  {
    digitalWrite(pulPin2, HIGH);
    delayMicroseconds(10000);
    digitalWrite(pulPin2, LOW);
    delayMicroseconds(10000);
  }

  digitalWrite(dirPin3, HIGH);
  for (float x = 0; x < 80.0 ; x++)
  {
    digitalWrite(pulPin3, HIGH);
    delayMicroseconds(10000);
    digitalWrite(pulPin3, LOW);
    delayMicroseconds(10000);
  }
  Serial.println("BYE");
  //hault();
//  delay(2000);
//  loop1();
//  loop2();
//  loop3();  
}

void back()
{
  Serial.println("HI");
  digitalWrite(dirPin1, LOW);
  for (float x = 0; x < stp1; x++)
  {
    digitalWrite(pulPin1, HIGH);
    delayMicroseconds(5000);
    digitalWrite(pulPin1, LOW);
    delayMicroseconds(5000);
  }

  digitalWrite(dirPin2, HIGH);
  for (float x = 0; x < stp2; x++)
  {
    digitalWrite(pulPin2, HIGH);
    delayMicroseconds(5000);
    digitalWrite(pulPin2, LOW);
    delayMicroseconds(5000);
  }

  digitalWrite(dirPin3, LOW);
  for (float x = 0; x < stp3; x++)
  {
    digitalWrite(pulPin3, HIGH);
    delayMicroseconds(5000);
    digitalWrite(pulPin3, LOW);
    delayMicroseconds(5000);
  }
  Serial.println("BYE");
  //delay(2000);
  //origin();
}
