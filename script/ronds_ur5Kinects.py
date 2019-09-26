#!/usr/bin/env python

from math import *
import cmath

x = 4
y = 4
z = 6

d1 = 0.1273
d2 = 0.0954
l0 = 0.0746
l1 = 0.4251
l2 = 0.3921
l3 = 0.0948
l4 = 0.2570




D = sqrt(pow(x - l4, 2) + pow(y - l3, 2))

test = (D*D - l2*l2 - (l0+l1)*(l0+l1)) / 2
print ("t2 = acos(" + str(test) + ")\n")
t2 = acos(test)

test = (l2/D)*sin(t2)
print ("phi = asin(" + str(test) + ")\n")
phi = asin(test)


t0 = pi/2
t1 = atan2((y-l0-l3)/D , (x-l4)/D)
t3 = -(t1 - phi)
t4 = -pi/2
t5 = 0

print(t0 + ", " + t1 + ", " + t2 + ", " + t3 + ", " + t4 + ", " + t5)