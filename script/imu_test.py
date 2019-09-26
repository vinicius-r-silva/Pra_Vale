#!/usr/bin/env python


import rospy
from sensor_msgs.msg import geometry_msgs
from sensor_msgs.msg import Imu
from math import asin
from math import acos

def callback(data):
    #print(data)
    z = data.orientation.z
    print("asin: " + str(asin(z)) + "   | acos: " + str(acos(z)) )
    

def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/sensor/imu", Imu, callback)

	rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()
