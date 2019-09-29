#!/usr/bin/env python

import cv2
import time
import rospy
import numpy as np
from cv_bridge import CvBridge

from std_msgs.msg import Int32
from sensor_msgs.msg import Image
#from std_msgs.msg import Int32MultiArray


#-------------------GLOBAL VARIABLES----------------# 
#enabled = True

#publish joint values to arm.py
pub = rospy.Publisher('/pra_vale/cam_found_fire', Int32, queue_size=10)

#ur5Cam Callback
#find fire in the given image and calculates the x,y coordinates of the fire
def ur5_callback(data):
    global pub
    # global enabled
    # if(not enabled):
    #     return

    #get the image
    bridge=CvBridge()
    frame = cv2.flip(cv2.cvtColor(bridge.imgmsg_to_cv2(data),cv2.COLOR_BGR2RGB),1)

    #Calculates RGB threshold to find fire
    mask=cv2.inRange(frame,(0,175,210),(64,255,255))
    
    # Find contours:
    contours, im = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if (len(contours) == 0):
        return
    #Get biggest object found
    fire=max(contours,key=cv2.contourArea)
    
    #Check size of object
    if(cv2.contourArea(fire)>100):
		# Draw contour:
        cv2.drawContours(frame, [fire], 0, (0, 0, 255), 3)

		# Calculate image moments of the detected contour
        M = cv2.moments(fire)

		# Print center:
        x = M['m10'] / M['m00']
        cv2.line(frame, (frame.shape[1]/2,0), (frame.shape[1]/2, frame.shape[0]), (255,0,0), 1)
        cv2.line(frame, ((int)(x),0), ((int)(x), frame.shape[0]), (0,255,0), 1)

        x = frame.shape[1]/2 - x
        print(x)
        
        #cv2.imshow("Threshold",mask)
        cv2.imshow("Camera",frame)
        cv2.waitKey(1)

        if(x > 6):
            x = x/3
            if x > 50:
                x = x/3
        elif(x > 3):
            x = x/2
            
        #publishes to the arm node
        pub.publish(data = x)


# def findFire_enabled(data):
#     global enabled
#     enabled = data.data
#     print("5cam is: " + str(enabled))


def listener():
    rospy.init_node('findFire', anonymous=True)

    rospy.Subscriber("/sensor/ur5toolCam", Image, ur5_callback)
    #rospy.Subscriber("/pra_vale/findFire_enabled", Bool, findFire_enabled)

    rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")

listener()
