#!/usr/bin/env python


import rospy
from sensor_msgs.msg import geometry_msgs
from sensor_msgs.msg import Imu
from math import asin
from math import acos
from math import atan2
from math import pi

def callback(data):
    #print(data)
    qx = data.orientation.x
    qy = data.orientation.y
    qz = data.orientation.z
    qw = data.orientation.w
    
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    x = atan2(t0, t1)

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    y = asin(t2)

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    z = atan2(t4, t3) + pi/2

    print("x: " + str(x) + ",   y: " + str(y) + ",   z: " + str(z))

    #print("asin(z): " + str(asin(z)) + "   | asin(w): " + str(asin(w)) )
    

def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/sensor/imu", Imu, callback)

	rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()
