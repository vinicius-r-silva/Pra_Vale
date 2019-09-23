#!/usr/bin/env python


import rospy
from rosi_defy.msg import ManipulatorJoints

PI = 3.14

def callback(data):
    print(data)
    pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)
    
    pub.publish(joint_variable = [PI/2, 0, 0, 0, PI/2, 0])


def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, callback)

	rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()
