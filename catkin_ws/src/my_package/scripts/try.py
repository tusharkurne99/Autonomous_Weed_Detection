#!/usr/bin/env python

import math
import cmath

e = 115.0
f = 457.3
re = 232.0
rf = 112.0

sqrt3 = math.sqrt(3)
pi = 3.141592654
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1 / sqrt3

x0 = 1
y0 = 1
z0 = 1

aux = 0
def delta_calcAngleYZ(x0, y0, z0):
    theta = 0
    y1 = -0.5 * 0.57735 * f
    y0 -= 0.5 * 0.57735 * e

    a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0)
    b = (y1 - y0) / z0
    d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)

    print (d)

    if (d < 0):
        
        return [-1]

    yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1)
    zj = a + b*yj

    theta = 180.0*atan(-zj/(y1 - yj)) / pi + (if yj>y1:
        180.0:0.0)

    if (aux == 1):
    t1 = theta

    if (aux == 2):
    t2 = theta

    if (aux == 3)
    t3 = theta

    return 0


def delta_calcInverse(x0, y0, z0):
    aux = 1
    status = delta_calcAngleYZ(x0, y0, z0)
    aux = 2
    if (status == 0):
    status = delta_calcAngleYZ(x0*cos120 + y0*sin120, yo*cos120 - x0*sin120, z0)
    aux == 3
    if (status == 0):
    status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120 + x0*sin120, z0)
    print (status)



if __name__ == '__main__':
    delta_calcAngleYZ(x0, y0, z0)
    delta_calcInverse(x0, y0, z0)
