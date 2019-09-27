#!/usr/bin/env python
from cv_bridge import CvBridge
import numpy as np
import rospy
import cv2
import time

from sensor_msgs.msg import Image

start_time = time.time() - 1
end_time  = time.time()
fps = 0

def callback(data):
	global start_time
	#rospy.loginfo(rospy.get_caller_id())
	print("\n\n")
#	print(data.width)
#	print("\n#\n")
#	print(data.height)
#	print("\n#\n")
#	print(data.encoding)
#	print("\n#\n")

	bridge=CvBridge()
	frame = cv2.flip(cv2.cvtColor(bridge.imgmsg_to_cv2(data),cv2.COLOR_BGR2RGB),1)


	mask=cv2.inRange(frame,(0,175,210),(64,255,255))
	#result=cv2.bitwise_and(frame,frame,mask=mask)

	# Find contours:
	im, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

	fire=max(contours,key=cv2.contourArea)
	
	if(cv2.contourArea(fire)>100):
		# Draw contours:
		cv2.drawContours(frame, [fire], 0, (0, 0, 255), 3)

		# Calculate image moments of the detected contour
		M = cv2.moments(fire)

		# Print center (debugging):
		
		print("Area : '{}'".format(cv2.contourArea(fire)))
		print("center X : '{}'".format(round(M['m10'] / M['m00'])))
		print("center Y : '{}'".format(round(M['m01'] / M['m00'])))



	

	cv2.imshow("Threshold",mask)
	cv2.imshow("Camera",frame)

	cv2.moveWindow('Camera',1080,0)
	cv2.moveWindow('Threshold',1080,535)
	#cv2.imwrite("Camera.jpg",cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))

	end_time = time.time()
	fps = 1 / (end_time - start_time)
	print(fps)

	cv2.waitKey(1)
	start_time = time.time()
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/sensor/ur5toolCam", Image, callback)

	rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()
