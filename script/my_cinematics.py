#!/usr/bin/env python

from math import pow
from math import pi
from math import sqrt
from math import atan2
from math import acos
#import cmath

x = 500
y = 700
z = 400

#-------------------CONST----------------#
probe_lenght = 150 #279
d0 = 275 #367
l1 = 425 #425.1
l2 = 392 #392.1
l3 = 126  #40.9

probe_lenght_pow = probe_lenght*probe_lenght
l1_pow = l1*l1
l2_pow = l2*l2

t4 = -pi/2
t5 = 0

#-------------------CALCS----------------#

x_pow = x*x
y_pow = y*y
z_pow = z*z

base_dist = sqrt(x_pow + y_pow)
d = sqrt(x_pow + y_pow - probe_lenght_pow)
t0 = atan2(y, x) + acos(probe_lenght/base_dist)

d_const = d - d0
z_const = z - l3

l_const_pow = d_const*d_const + z_const*z_const
l_const = sqrt(l_const_pow)

print("base_dist: " + str(base_dist) + "   d: " + str(d) + "   d_const: " + str(d_const) )
print("l1_pow: " + str(l1_pow) + "    l2_pow: " + str(l2_pow) + "    l_const_pow: " + str(l_const_pow))
print("l1_pow - l2_pow + l_const_pow: " + str(l1_pow - l2_pow + l_const_pow))
print("2*l1*l_const: " + str(2*l1*l_const))
print("t1_const: acos(" + str((l1_pow - l2_pow + l_const_pow) / (2*l1*l_const)) + ")\n")

print ("z_const: " + str(z_const) + ",  d_const: " + str(d_const) + "\n")

t1_const = acos((l1_pow - l2_pow + l_const_pow) / (2*l1*l_const))
t2_const = acos((l1_pow + l2_pow - l_const_pow) / (2*l1*l2))
#t3_const = acos((-l1_pow + l2_pow + l_const_pow) / (2*l2*l_const))
t3_const = pi - t1_const - t2_const
#print ("t1_const: " + str(t1_const) + ",  t2_const: " + str(t2_const) +  ",  t3_const: " + str(t3_const) + "\n")


s1 = acos(z_const/l_const)
s2 = acos(d_const/l_const)
s3 = pi/2 - s2

t1 = pi/2 - (t1_const + s2)
t2 = pi - t2_const
t3 = -(t3_const + s1)

print(str(t0) + ", " + str(t1) + ", " + str(t2) + ", " + str(t3) + ", " + str(t4) + ", " + str(t5))