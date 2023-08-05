#!/usr/bin/env python

                            #  Project Name: 	autonomus_weed_detection_and_removal_system
                            #  Author List: 		Tushar Kurne
                            #  Filename: 		get_angles.py
                            #  Functions: 		angle_yz, find_angles
                            #  Global Variables:	e,f,re,rf,sqrt3,pi,sin120,cos120,tan60,sin30,tan30

import rospy
from rospy_tutorials.msg import Floats
import numpy as np 
import math 
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point

e  =  173.0 #radius of end effector
f  =  354.0 #radius of stationary base plate
re =  277.0 #length of fore arms of delta manipulator
rf =  178.0 #length of bicep of delta manipulator


sqrt3  = math.sqrt(3.0)
pi     = 3.141592653
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60  = sqrt3
sin30  = 0.5
tan30  = 1.0 / sqrt3


                            # Function Name angle_yz
                            # Input: 		x,y and z on cordinatee of the point 
                            # Output: 		returns angle theta in degrees 
                            # Logic: 		find out the theta for single frame
                            # Example Call: status = angle_yz(x0,y0,z0)		
def angle_yz(x0, y0, z0, theta=None):
    y1 = -0.5*0.57735*f # f/2 * tg 30
    y0 -= 0.5*0.57735*e # shift center to edge
    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1) / (2.0*z0)
    b = (y1-y0) / z0
    
    d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf)
    if d<0:
        return [1,0] # theta is non existing for those coordinates

    yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1) # horizontal distance of elbow from centre 
    zj = a + b*yj #vertical distance of elbow from centre
    theta = math.atan(-zj / (y1-yj)) * 180.0 / pi + (180.0 if yj>y1 else 0.0)
    return [0,theta] # return theta


                            # Function Name find_angles
                            # Input: 		data of type point 
                            # Output: 		publishes angles required to reach particular poisition
                            # Logic: 		initailly one theta of one frame is calculated as it is after that each theta is calculated by multiplying the frame by rotation matrix of angle 120 degree
                            # Example Call: callback function hence no need to call individuallly

def find_angles(data):
    
    theta1 = 0 #initial angles 
    theta2 = 0 #initial angles 
    theta3 = 0 #initial angles 
    x0=data.x-177 #transformation from frame of camera to delta 
    y0=data.y+132 #transformation from frame of camera to delta
    z0=data.z #transformation from frame of camera to delta
    status = angle_yz(x0,y0,z0) #calling yz on 1st frame of first stepper

    if status[0] == 0:
        theta1 = status[1]
        status = angle_yz(x0*cos120 + y0*sin120,
                                   y0*cos120-x0*sin120,
                                   z0,
                                   theta2) #finding theta after multiplying by roatation matrix
    if status[0] == 0:
        theta2 = status[1]
        status = angle_yz(x0*cos120 - y0*sin120,
                                   y0*cos120 + x0*sin120,
                                   z0,
                                   theta3 )#finding theta after multiplying by roatation matrix
    theta3 = status[1]

    # print(theta1,theta2,theta3)
    centre=Point()
    centre.x=theta1 
    centre.y=theta2
    centre.z=theta3
    weed_centre.publish(centre) #publishes angles in theta for corresponding cordinate
    
if __name__=='__main__':
    rospy.init_node('get_angles')
    cordinate_sub=rospy.Subscriber("mid_point",Point,find_angles,queue_size=10)
    weed_centre=rospy.Publisher("mid_point_weed_final",Point,queue_size=10)
    rospy.spin()

 
