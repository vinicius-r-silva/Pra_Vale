#!/usr/bin/env python
from cv_bridge import CvBridge
import numpy as np
import rospy
import cv2
import time

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray



pub = rospy.Publisher('/pra_vale/arm_move', Int32MultiArray, queue_size=10)


def callback(data):
    global pub
    arm_move = [10,0,0]
    pub.publish(data = arm_move)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensor/ur5toolCam", Image, callback)

    rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")

listener()
