#!/usr/bin/env python
from cv_bridge import CvBridge
import defines as defs
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
from std_msgs.msg import Int32

state = defs.NOTHING

global scaleList, stair

#Load the template, convert it to grayscale, and detect edges

# get the file path for pra_vale
stair = cv2.imread(rospack.get_path('pra_vale') + '/resources/print.png')
stair = cv2.cvtColor(stair, cv2.COLOR_BGR2GRAY)
stair = cv2.Canny(stair, 50, 200)

scaleList=np.linspace(1.0, 0.2, 15).tolist()



#callback function called when a node requires a state change
def set_state(data):
	global state
	state = data.data

# loop over the images to find the template in
def kin_callback(data):
	global state, scaleList, stair

	if(state & (1 << defs.HOKUYO_READING | 1 << defs.INITIAL_SETUP)):
		return

	bridge=CvBridge()
	image = cv2.flip(cv2.cvtColor(bridge.imgmsg_to_cv2(data),cv2.COLOR_BGR2RGB),1)


	crop=image[image.shape[0]/2:image.shape[0],:int(image.shape[1]*0.75)]
	r,g,b=cv2.split(crop)

	mask=(cv2.add(cv2.subtract(r,b),cv2.subtract(b,r))[:]<25)
	mask=np.logical_and(mask,r[:]>70)


	crop=cv2.bitwise_and(crop,crop,mask=mask.astype(np.uint8))
	#cv2.imshow("Hist",crop)
	#cv2.moveWindow("Hist",800,1200)
	#cv2.waitKey(10)
	gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
	found = None

	# loop over the scales of the image
	for scale in scaleList:
		#print scale
		# resize the image according to the scale, and keep track
		# of the ratio of the resizing
		template = imutils.resize(stair, width=int(stair.shape[1] * scale))
	
		
		# if the resized image is smaller than the template, then break
		# from the loop
		if gray.shape[0] < template.shape[0] or gray.shape[1] < template.shape[1]:
			print("resized fail", scale)
			break

		# detect edges in the resized, grayscale image and apply template
		# matching to find the template in the image
		edged = cv2.Canny(gray, 50, 200)
		#cv2.imshow("edge",edged)
		#cv2.waitKey(10)
		result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
		(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

		
		# if we have found a new maximum correlation value, then ipdate
		# the bookkeeping variable
		if (found is None or maxVal > found[0]) and maxVal > 7500000:
			found = (maxVal, maxLoc)
			(tH, tW) = template.shape[:2]

	if(found !=None):
		# unpack the bookkeeping varaible and compute the (x, y) coordinates
		# of the bounding box based on the resized ratio
		(chance, maxLoc) = found
		#print chance
		start = (maxLoc[0]   , maxLoc[1] + image.shape[0]/2)
		end = (maxLoc[0] + tW, maxLoc[1] + tH + image.shape[0]/2)
		
		#print(float(maxLoc[0] + tW/2)/image.shape[1]/2, float(maxLoc[1] + tH/2 + image.shape[0]/2)/image.shape[1]/2)
		state |= 1 << defs.FOUND_STAIR
		state_publisher = rospy.Publisher('/pra_vale/set_state', Int32, queue_size=1)
		state_publisher.publish(data = state)

		# draw a bounding box around the detected result and display the image
		cv2.rectangle(image, start, end, (0, 0, 255), 2)
	cv2.imshow("Detection",image)
	cv2.waitKey(1)
def listener():
	rospy.init_node('findStair', anonymous=True)

	rospy.Subscriber("/sensor/kinect_rgb", Image, kin_callback)
	rospy.Subscriber("/pra_vale/estados", Int32, set_state)
	
	rospy.spin()

#main

print("Escada launched")
listener()