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



	params = cv2.SimpleBlobDetector_Params()
	params.filterByArea=True
	params.minArea=2700
	params.filterByCircularity = False
	params.filterByInertia = False
	params.filterByConvexity = False
	params.filterByColor = False
	params.blobColor = z



	detector = cv2.SimpleBlobDetector_create(params)	

	mask=cv2.inRange(frame,(0,175,210),(64,255,255))
	result=cv2.bitwise_and(frame,frame,mask=mask)

	keypoints = detector.detect(mask)

	for blob in keypoints:
		print("\nFIRE:")
		print(blob.pt)
		print(blob.size)

	frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


	cv2.imshow("Threshold",result)
	cv2.imshow("Camera",frame)

	cv2.moveWindow('Camera',0,0)
	cv2.moveWindow('Threshold',0,535)
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
