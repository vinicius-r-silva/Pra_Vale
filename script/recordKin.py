#!/usr/bin/env python

from cv_bridge import CvBridge
import numpy as np
import imutils
import rospy
import cv2
import rospkg


# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# list all packages, equivalent to rospack list
rospack.list() 


from sensor_msgs.msg import Image

#Load the template, convert it to grayscale, and detect edges

# get the file path for pra_vale

out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (480,640))



# loop over the images to find the template in
def kin_callback(data):
	global out

	bridge=CvBridge()
	frame = cv2.flip(cv2.cvtColor(bridge.imgmsg_to_cv2(data),cv2.COLOR_BGR2RGB),1)

	out.write(frame)

	cv2.imshow('frame',frame)

	k=cv2.waitKey(50)
	print(k)
	if(k==ord("q")):
		rospy.signal_shutdown("Finished Recording")
		print("Closing")
		out.release()
		exit()



def listener():
	rospy.init_node('findFire', anonymous=True)

	rospy.Subscriber("/sensor/kinect_rgb", Image, kin_callback)

	rospy.spin()

#main

print("Recording")
listener()