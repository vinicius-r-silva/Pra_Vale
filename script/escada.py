#!/usr/bin/env python
from cv_bridge import CvBridge
import defines as defs
import numpy as np
import rospy
import cv2
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Int32


CUT_SCALE = [0.5, 0.1]
NOTA_MAX  = 50


# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# list all packages, equivalent to rospack list
rospack.list()

state=defs.NOTHING

#Load the template, convert it to grayscale, and detect edges
# get the file path for pra_vale
stair = cv2.imread(rospack.get_path('pra_vale') + '/resources/escada.png')
stair = cv2.cvtColor(stair, cv2.COLOR_BGR2GRAY)

scaleList=np.linspace(1.6, 0.5, 15).tolist()

state_publisher = rospy.Publisher('/pra_vale/def_state', Int32, queue_size=15)

#callback function called when a node requires a state change
def set_state(data):
	global state
	state = data.data

def resize(image, width = None, height = None, inter = cv2.INTER_AREA):
	# initialize the dimensions of the image to be resized and
	# grab the image size
	dim = None
	(h, w) = image.shape[:2]

	# if both the width and height are None, then return the
	# original image
	if width is None and height is None:
		return image

	# check to see if the width is None
	if width is None:
		# calculate the ratio of the height and construct the
		# dimensions
		r = height / float(h)
		dim = (int(w * r), height)

	# otherwise, the height is None
	else:
		# calculate the ratio of the width and construct the
		# dimensions
		r = width / float(w)
		dim = (width, int(h * r))

	# resize the image
	resized = cv2.resize(image, dim, interpolation = inter)

	# return the resized image
	return resized

# loop over the images to find the template in
def kin_callback(data):
	#Constants
	global NOTA_MAX, CUT_SCALE

	#Variables
	global scaleList, state, stair, state_publisher

	if(state & (1 << defs.HOKUYO_READING | 1 << defs.INITIAL_SETUP | 1 << defs.IN_STAIR)):
		return

	bridge=CvBridge()
	image = cv2.flip(cv2.cvtColor(bridge.imgmsg_to_cv2(data),cv2.COLOR_BGR2RGB),1)


	crop=image[int(image.shape[0]*CUT_SCALE[0]):image.shape[0], int(image.shape[1]*CUT_SCALE[1]):int(image.shape[1]*(1-CUT_SCALE[1]))]
	r,g,b=cv2.split(crop)

	mask=(cv2.add(cv2.subtract(r,b),cv2.subtract(b,r))[:]<25)
	mask=np.logical_and(mask,r[:]>70)


	crop=cv2.bitwise_and(crop,crop,mask=mask.astype(np.uint8))
	
	if defs.DEBUGGING:
		cv2.imshow("escada_Hist",crop)
		cv2.moveWindow("escada_Hist",1920,1200)
		cv2.waitKey(10)
	
	gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
	found = None

	# loop over the scales of the image
	for scale in scaleList:
		#print scale
		# resize the image according to the scale, and keep track
		# of the ratio of the resizing
		template = resize(stair, width=int(stair.shape[1] * scale))
	
		
		# if the resized image is smaller than the template, then break
		# from the loop
		if gray.shape[0] < template.shape[0] or gray.shape[1] < template.shape[1]:
			print("resized fail", scale)
			break

		
		result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
		(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
		maxVal*=100
		
		# if we have found a new maximum correlation value, then ipdate
		# the bookkeeping variable
		if (found is None or maxVal > found[0]) and maxVal > NOTA_MAX:
			#print maxVal
			found = (maxVal, maxLoc)
			(tH, tW) = template.shape[:2]
	if(found !=None):
		# unpack the bookkeeping varaible and compute the (x, y) coordinates
		# of the bounding box based on the resized ratio
		(chance, maxLoc) = found
		#print chance
		start = (maxLoc[0] + int(image.shape[0]*CUT_SCALE[1])     , maxLoc[1] + int(image.shape[0]*CUT_SCALE[0]))
		end   = (maxLoc[0] + int(image.shape[0]*CUT_SCALE[1]) + tW, maxLoc[1] + int(image.shape[0]*CUT_SCALE[0]) + tH)
		
		#state_publisher.publish(data = defs.FOUND_STAIR)

		# draw a bounding box around the detected result and display the image
		cv2.rectangle(image, start, end, (0, 0, 255), 2)
	else:
		state_publisher.publish(data = (-1)*defs.FOUND_STAIR)
	if defs.DEBUGGING:
		cv2.imshow("escada_Detection",image)
		cv2.moveWindow("escada_Detection",1920,0)
		cv2.waitKey(1)
def listener():
	rospy.init_node('findStair', anonymous=True)

	rospy.Subscriber("/sensor/kinect_rgb", Image, kin_callback)
	rospy.Subscriber("/pra_vale/estados", Int32, set_state)
	
	rospy.spin()

#main

print("Escada launched")
listener()