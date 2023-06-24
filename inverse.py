import numpy as np 
from numpy import linalg

import cmath
from math import *

global mat
mat = np.matrix

global d1, a2, a3
d1 = 70
a2 = 120
a3 = 120

def tinh_theta1(xE, yE, zE):
    theta1 = atan2(xE,yE)
    c1 = cos(theta1)
    s1 = sin(theta1)

    c3 = (pow((xE/c1),2)+pow((zE-d1),2)-pow(a3,2)-pow(a2,2))/(2*a3*a2)
    s3 = -sqrt(1-pow(c3,2))
    theta3 = atan2(s3,c3)

    c2 = (a2*(xE/c1)+a3*(c3*(xE/c1)+s3*(zE-d1)))/(pow((xE/c1),2)+pow((zE-d1),2))
    s2 = (a2*(zE-d1)+a3*(c3*(zE-d1)-s3*(xE/c1)))/(pow((xE/c1),2)+pow((zE-d1),2))
    theta2 = atan2(s2,c2)

    theta = mat([theta1, theta2, theta3])

    return theta


y = tinh_theta1(100,-120,100)
print(y)