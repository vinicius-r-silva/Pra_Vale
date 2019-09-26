#!/usr/bin/env python

from math import *
import cmath

x = 600
y = 600
z = 600


#d1 = 0.1273
#d2 = 0.0954
#l0 = 0.0746
#l1 = 0.4251
#l2 = 0.3921
#l3 = 0.0948
#l4 = 0.2570

d0 = 367
l1 = 425 #425.1
l2 = 392 #392.1
l3 = 41  #40.9

t0 = 0
t1 = 0
t2 = 0
t3 = 0
t4 = 0
t5 = 0

base_dist = sqrt(x*x + y*y)
t0 = atan2(y, x) + acos(d0/base_dist)
t4 = -pi/2
t5 = 0

pow
d = sqrt(x*x*x*x + 2*x*x*y*y + y*y*y*y - d0*d0)
d_const = base_dist - d0
z_const = z - l3
l_const = sqrt(d_const*d_const + z_const*z_const)

#print("l1*l1: " + str(l1*l1) + "    l2*l2: " + str(l2*l2) + "    l_const*l_const: " + str(l_const*l_const))
#print("l1*l1 - l2*l2 + l_const*l_const: " + str(l1*l1 - l2*l2 + l_const*l_const))
#print("2*l1*l_const: " + str(2*l1*l_const))
#print("t1_const: acos(" + str((l1*l1 - l2*l2 + l_const*l_const) / (2*l1*l_const)) + ")\n")

t1_const = acos((l1*l1 - l2*l2 + l_const*l_const) / (2*l1*l_const))
t2_const = acos((l1*l1 + l2*l2 - l_const*l_const) / (2*l1*l2))
t3_const = acos((-l1*l1 + l2*l2 + l_const*l_const) / (2*l2*l_const))

s1 = acos(z_const/l_const)
s2 = acos(d_const/l_const)
s3 = pi/2 - s2

t1 = pi - t2_const
t3 = t3_const + s1
t2 = t1_const - s3

print(str(t0) + ", " + str(t1) + ", " + str(t2) + ", " + str(t3) + ", " + str(t4) + ", " + str(t5))