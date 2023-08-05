#!/usr/bin/env python

import math
import time

# Specific geometry for bitbeambot:
# http://flic.kr/p/cYaQah
e  =  26.0
f  =  69.0
re = 128.0
rf =  88.0

# Trigonometric constants
s      = 165*2
sqrt3  = math.sqrt(3.0)
pi     = 3.141592653
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60  = sqrt3
sin30  = 0.5
tan30  = 1.0 / sqrt3

x0 =12
y0 =78
z0 =78


# Inverse kinematics
# Helper functions, calculates angle theta1 (for YZ-pane)

y1 = -0.5*0.57735*f # f/2 * tg 30
y0 -= 0.5*0.57735*e # shift center to edge
    # z = a + b*y
a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1) / (2.0*z0)
b = (y1-y0) / z0

    # discriminant
d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf)
print(d)
# if d<0:
    # return [1,0] # non-existing povar.  return error, theta
23
yj = (y1 - a*b - math.sqrt(abs(d))) / (b*b + 1) # choosing outer povar
zj = a + b*yj
theta = math.atan(-zj / (y1-yj)) * 180.0 / pi + (180.0 if yj>y1 else 0.0)
    
# return [0,theta] # return error

print(theta)

