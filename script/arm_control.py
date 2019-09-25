#!/usr/bin/env python


import rospy
from rosi_defy.msg import ManipulatorJoints

PI = 3.14

def callback(data):
    print(data)
    pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)
    
    pub.publish(joint_variable = [1.90891567046, 0.888681284086, 0.285968146792, -1.17464943088, -1.57079632679, 0])


def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, callback)

	rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()
