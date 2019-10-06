#!/usr/bin/env python
from cv_bridge import CvBridge
import defines as defs
import numpy as np
import rospy
import cv2
import rospkg

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

#consts
NOTA_MAX  = 120


state = defs.NOTHING

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# list all packages, equivalent to rospack list
rospack.list() 


#Load the template, convert it to grayscale, and detect edges
# get the file path for pra_vale
beam = cv2.imread(rospack.get_path('pra_vale') + '/resources/beam.png')
beam = cv2.cvtColor(beam, cv2.COLOR_BGR2GRAY)

scaleList=np.linspace(1.5, 0.1, 15).tolist()

arm_move = rospy.Publisher('/pra_vale/arm_move', Int32MultiArray, queue_size=10)

touched=0

#callback function called when a node requires a state change
def close(data):
	global state,touched
	state = data.data
	if(~state & (1 << defs.BEAM_FIND) and touched):
		rospy.signal_shutdown("Finished job")
		print("FINSHED BEAM")
		exit()


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

def beam_callback(data):
	#Constants
	global NOTA_MAX, CUT_SCALE

	#Variables
	global scaleList, state, beam, arm_move,touched

	if(state & (1 << defs.HOKUYO_READING | 1 << defs.INITIAL_SETUP)):
		return

	bridge=CvBridge()
	image = cv2.flip(cv2.cvtColor(bridge.imgmsg_to_cv2(data),cv2.COLOR_BGR2RGB),1)


	r,g,b=cv2.split(image)

	mask=(cv2.add(cv2.subtract(r,b),cv2.subtract(b,r))[:]<25)
	mask=np.logical_and(mask,r[:]>200)


	hist=cv2.bitwise_and(image,image,mask=mask.astype(np.uint8))
	
	if defs.DEBUGGING:
		cv2.imshow("beam_Hist",hist)
		cv2.moveWindow("beam_Hist",1920,1200)
		cv2.waitKey(10)
	
	gray = cv2.cvtColor(hist, cv2.COLOR_BGR2GRAY)
	found = None

	# loop over the scales of the image
	for scale in scaleList:
		# resize the image according to the scale, and keep track
		# of the ratio of the resizing
		template = resize(beam, width=int(beam.shape[1] * scale))
	
		
		# if the resized image is smaller than the template, then break
		# from the loop
		if gray.shape[0] < template.shape[0] or gray.shape[1] < template.shape[1]:
			print("resized fail", scale)
			break

		
		result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF)
		
		(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
		
		maxVal/=1000000

		
		# if we have found a new maximum correlation value, then update
		# the bookkeeping variable
		if (found is None or maxVal > found[0]) and maxVal > NOTA_MAX:
			#print(maxVal)
			found = (maxVal, maxLoc)
			(tH, tW) = template.shape[:2]

	if(found !=None):
		# unpack the bookkeeping varaible and compute the (x, y) coordinates
		# of the bounding box based on the resized ratio
		(chance, maxLoc) = found
		
		start = (maxLoc[0]     , maxLoc[1])
		end   = (maxLoc[0] + tW, maxLoc[1] + tH)
		
		state |= 1 << defs.FOUND_STAIR
		state_publisher = rospy.Publisher('/pra_vale/set_state', Int32, queue_size=1)
		state_publisher.publish(data = state)

		error = ((start[0]+end[1])/2.0,(start[0]+end[1])/2.0)

		if(error>10):
			error = 10
		elif(error<10):
			error = -10

		arm_move.publish(data = [error,0,0])

		touched=1
		
		# draw a bounding box around the detected result and display the image
		cv2.rectangle(image, start, end, (0, 0, 255), 2)

	if defs.DEBUGGING:
		cv2.imshow("beam_Detection",image)
		cv2.moveWindow("beam_Detection",1920,0)
		cv2.waitKey(1)


def listener():
	rospy.init_node('hokuyo', anonymous=True)

	rospy.Subscriber("/sensor/ur5toolCam", Image, beam_callback)
	rospy.Subscriber("/pra_vale/estados", Int32, close)
	
	rospy.spin()


#main
print("hokuyo launched")
listener()