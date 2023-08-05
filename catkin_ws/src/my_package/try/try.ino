/*
Project Name: 	Autonomous weed detection and removal system
Author List:    Swapnil Patil
Filename: 		Arduino Firmware
Functions: 		messageCb, temp, Motors_init, MotorLF, MotorRF, MotorLB, MotorRB, MotorForward
Global Variables:	count, speed, flag, PWM1, PWM2, PWM3, PWM4, LF_DIR, RF_DIR, LB_DIR, RB_DIR, BRK1, BRK2, BRK3, BRK4, w_l, w_r, wheel_rad, wheel_sep, speed_ang, speed_lin
*/

//Arduino header file
#include <ArduinoHardware.h>
//Main ROS header file for node handling
#include <ros.h>
//temporary hold incoming messages
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include "ros/time.h"

// int count = 0;

int speed = 35;

int flag = 0;

//for Pulse Width Modulation
#define PWM1 12
#define PWM2 10
#define PWM3 11
#define PWM4 9

//for direction pin
#define LF_DIR 44
#define RF_DIR 28
#define LB_DIR 40
#define RB_DIR 24

//for break pin
#define BRK1 46
#define BRK2 26
#define BRK3 42
#define BRK4 22

double w_r = 0, w_l = 0;

double wheel_rad = 0.0325, wheel_sep = 0.295;

//creating ros node handler
ros::NodeHandle nh;

double speed_ang = 0, speed_lin = 0;

/*

Function Name: 	messageCb
Input: 		
Output:   Calculate angular and linear speed
Logic:    This function is used for calculation of angular and linear speed and store the data into variables and callback when it requires anywhere in program
Example Call:
*/

void messageCb(const geometry_msgs::Twist &msg)
{
    speed_ang = msg.angular.z;
    speed_lin = msg.linear.x;

    w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
    w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
}

/*

Function Name: 	temp
Input: 	  
Output:   flag value will change when function is called.
Logic:    The flag variable which is gloabal is set as zero and it's changes after call the function to actuate motor particular distance.
Example Call:

*/

void temp(const std_msgs::Int8 &msg)
{
    //flag is use to drive all motor forward for particular distance
    //MotorForward();
    flag=1;
}

//creating Subscriber function with callback function temp
ros::Subscriber<std_msgs::Int8> sub1("go_ahead", &temp);

//creating Subscriber object with call function messageCb
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

//define all function
void Motors_init();
void MotorLF(int Pulse_Width1);
void MotorRF(int Pulse_Width2);
void MotorLB(int Pulse_Width1);
void MotorRB(int Pulse_Width2);
void MotorForward(int Pulse_Width3);

void setup()
{
    //to call function one by one in loop
    Motors_init();

    nh.initNode();

    nh.subscribe(sub);

    nh.subscribe(sub1);
}

void loop()
{
    //To actuate Front Left motor
    MotorLF(w_l * 10);
    //To actuate Front Right motor
    MotorRF(w_r * 10);
    //To actuate Back Left motor
    MotorLB(w_l * 10);
    //To actuate Back Right motor
    MotorRB(w_r * 10);
    //To drive bot forward to a particular distance
    MotorForward();

    nh.spinOnce();
}

/*

Function Name: 	Motors_init
Input: 		
Output:   Set arduino pins as OUTPUT or INPUT & write pin status HIGH or LOW
Logic:    
Example Call: Motors_init() which is call in void setup()

*/

void Motors_init()
{

    ///setting all pins mode
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(PWM3, OUTPUT);
    pinMode(PWM4, OUTPUT);

    pinMode(RF_DIR, OUTPUT);
    pinMode(RB_DIR, OUTPUT);
    pinMode(LB_DIR, OUTPUT);
    pinMode(LF_DIR, OUTPUT);

    pinMode(BRK1, OUTPUT);
    pinMode(BRK2, OUTPUT);
    pinMode(BRK3, OUTPUT);
    pinMode(BRK4, OUTPUT);

    digitalWrite(BRK1, LOW);
    digitalWrite(BRK2, LOW);
    digitalWrite(BRK3, LOW);
    digitalWrite(BRK4, LOW);

    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
    digitalWrite(PWM3, LOW);
    digitalWrite(PWM4, LOW);

    digitalWrite(RF_DIR, LOW);
    digitalWrite(RB_DIR, LOW);
    digitalWrite(LB_DIR, LOW);
    digitalWrite(LF_DIR, LOW);
}

/*

Function Name: 	MotorLF
Input: 	   pulse width modulation 
Output:    Actuate Left Front Motor according to the PWM signal
Logic:     PWM signal is pass throgh if statement if satisfies then any if statement it gives output to interms of HIGH or LOW
Example Call: 

*/

void MotorLF(int Pulse_Width1)
{
    if (Pulse_Width1 > 0)
    {

        analogWrite(PWM1, Pulse_Width1);

        digitalWrite(LF_DIR, HIGH);
    }

    if (Pulse_Width1 < 0)
    {

        Pulse_Width1 = abs(Pulse_Width1);

        analogWrite(PWM1, Pulse_Width1);

        digitalWrite(LF_DIR, LOW);
    }

    if (Pulse_Width1 == 0)
    {

        analogWrite(PWM1, Pulse_Width1);

        digitalWrite(LF_DIR, LOW);
    }
}

/*

Function Name: 	MotorRF
Input: 	   pulse width modulation 
Output:    Actuate Right Front Motor according to the PWM signal
Logic:     PWM signal is pass throgh if statement if satisfies then any if statement it gives output to interms of HIGH or LOW
Example Call: 

*/

void MotorRF(int Pulse_Width2)
{

    if (Pulse_Width2 > 0)
    {

        analogWrite(PWM2, Pulse_Width2);

        digitalWrite(RF_DIR, HIGH);
    }

    if (Pulse_Width2 < 0)
    {

        Pulse_Width2 = abs(Pulse_Width2);

        analogWrite(PWM2, Pulse_Width2);

        digitalWrite(RF_DIR, LOW);
    }

    if (Pulse_Width2 == 0)
    {

        analogWrite(PWM2, Pulse_Width2);

        digitalWrite(RF_DIR, LOW);
    }
}

/*

Function Name: 	MotorLB
Input: 	   pulse width modulation 
Output:    Actuate Left Back Motor according to the PWM signal
Logic:     PWM signal is pass throgh if statement if satisfies then any if statement it gives output to interms of HIGH or LOW
Example Call: 

*/

void MotorLB(int Pulse_Width1)
{
    if (Pulse_Width1 > 0)
    {

        analogWrite(PWM3, Pulse_Width1);

        digitalWrite(LB_DIR, HIGH);
    }

    if (Pulse_Width1 < 0)
    {

        Pulse_Width1 = abs(Pulse_Width1);

        analogWrite(PWM3, Pulse_Width1);

        digitalWrite(LB_DIR, LOW);
    }

    if (Pulse_Width1 == 0)
    {

        analogWrite(PWM3, Pulse_Width1);

        digitalWrite(LB_DIR, LOW);
    }
}

/*

Function Name: 	MotorRB
Input: 	   pulse width modulation 
Output:    Actuate Right Back Motor according to the PWM signal
Logic:     PWM signal is pass throgh if statement if satisfies then any if statement it gives output to interms of HIGH or LOW
Example Call: 

*/

void MotorRB(int Pulse_Width2)
{
    if (Pulse_Width2 > 0)
    {

        analogWrite(PWM4, Pulse_Width2);

        digitalWrite(RB_DIR, HIGH);
    }

    if (Pulse_Width2 < 0)
    {

        Pulse_Width2 = abs(Pulse_Width2);

        analogWrite(PWM4, Pulse_Width2);

        digitalWrite(RB_DIR, LOW);
    }

    if (Pulse_Width2 == 0)
    {

        analogWrite(PWM4, Pulse_Width2);

        digitalWrite(RB_DIR, LOW);
    }
}

/*

Function Name: 	MotorForward
Input: 	   speed value as PWM,, flag value to satsfy if statement
Output:    Actuate all four motors in forward direction to a particular distance to match the delta arm centre point and camera's centre point
Logic:     It depends upon flag value, whenever its 1  motors are actuated
Example Call: 

*/

void MotorForward()
{
    if (flag == 1)
   {
        //nh.loginfo to print the message on the terminal
        nh.loginfo("Motor on");
        //define  speed data to PWM pin
        analogWrite(PWM1, speed);
        analogWrite(PWM2, speed);
        analogWrite(PWM3, speed);
        analogWrite(PWM4, speed);
        digitalWrite(RF_DIR, HIGH);
        digitalWrite(RB_DIR, HIGH);
        digitalWrite(LF_DIR, HIGH);
        digitalWrite(LB_DIR, HIGH);

      
        delay(1800);

        digitalWrite(BRK1, LOW);
        digitalWrite(BRK2, LOW);
        digitalWrite(BRK3, LOW);
        digitalWrite(BRK4, LOW);
        nh.loginfo("Motor off");
        flag = 0;
    }
}
