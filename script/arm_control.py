#!/usr/bin/env python


import rospy
from rosi_defy.msg import ManipulatorJoints
from math import pi
from math import pow
from math import sqrt
from math import atan2
from math import acos

#-------------------CONST----------------#
probe_lenght = 150 #279
d0 = 275 #367
l1 = 425 #425.1
l2 = 392 #392.1
l3 = 111  #126

probe_lenght_pow = probe_lenght*probe_lenght
l1_pow = l1*l1
l2_pow = l2*l2

t5 = 0

def cinematicaInversa(x, y, z):
    x_pow = x*x
    y_pow = y*y
    #z_pow = z*z

    y_const = y - d0
    y_const_pow = y_const*y_const

    base_dist = sqrt(x_pow + y_const_pow)
    d = sqrt(x_pow + y_const_pow - probe_lenght_pow)

    t0 = atan2(y_const, x) + acos(probe_lenght/base_dist)
    t4 = -t0
    # if (t4 > pi):
    #     t4 -= pi
    # elif(t4 < -pi):
    #     t4 += pi

    d_const = d
    z_const = z - l3

    l_const_pow = d_const*d_const + z_const*z_const
    l_const = sqrt(l_const_pow)

    t1_const = acos((l1_pow - l2_pow + l_const_pow) / (2*l1*l_const))
    t2_const = acos((l1_pow + l2_pow - l_const_pow) / (2*l1*l2))
    t3_const = pi - t1_const - t2_const

    s1 = acos(z_const/l_const)
    s2 = acos(d_const/l_const)
    #s3 = pi/2 - s2

    t1 = pi/2 - (t1_const + s2)
    t2 = pi - t2_const
    t3 = -(t3_const + s1)

    return [t0, t1, t2, t3, t4, t5]



def callback(data):
    print(data)
    pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)
    pos = cinematicaInversa(400, 400, 400)
    #pos = [2.04466973108, 0.636854893566, 1.10748279154, -1.74433768511, -1.5, 0]
    #pos = [2.04466973108, 0.636854893566, 1.10748279154, -1.74433768511, -2.04466973108, 0]
    #pub.publish(joint_variable = pos)
    pub.publish(joint_variable = [pi/2,0,0,0,pi/2,0])

def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, callback)

	rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()
