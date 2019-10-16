#!/usr/bin/env python
from cv_bridge import CvBridge
import defines as defs
import numpy as np
import rospy
import cv2
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Int32


DEBUGGING=True
NOTA_MAX= 120
CUT_SCALE=[0.5, 0.1]


state = defs.NOTHING

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
# list all packages, equivalent to rospack list
rospack.list() 


#callback function called when a node requires a state change
def set_state(data):
	global state
	state = data.data




# loop over the images to find the template in
def kin_callback(data):
	#Constants
	global DEBUGGING, CUT_SCALE

	#Variables
	global state

	if(state & (1 << defs.HOKUYO_READING | 1 << defs.INITIAL_SETUP)):
		return

	if(~state & (1 << defs.IN_STAIR)):
		return

	bridge=CvBridge()
	image = cv2.flip(cv2.cvtColor(bridge.imgmsg_to_cv2(data),cv2.COLOR_BGR2RGB),1)


	r,g,b=cv2.split(image)

	mask=(cv2.add(cv2.subtract(r,b),cv2.subtract(b,r))[:]<25)
	mask=np.logical_and(mask,r[:]>70)


	image=cv2.bitwise_and(image,image,mask=mask.astype(np.uint8))

	
	
	area = (100.0*sum(sum(mask))/float(image.shape[0]*image.shape[1]))
	
	print "Porcentagem de pixels cinzas: ",area

	if DEBUGGING:
		cv2.imshow("escada_end_Hist",image)
		cv2.moveWindow("escada_end_Hist",1920,1200)
		cv2.waitKey(10)

	if(area<10):
		state |= 1 << defs.END_STAIR
		state_publisher = rospy.Publisher('/pra_vale/set_state', Int32, queue_size=1)
		state_publisher.publish(data = state)
	else:
		state &= ~(1 << defs.END_STAIR)
		state_publisher = rospy.Publisher('/pra_vale/set_state', Int32, queue_size=1)
		state_publisher.publish(data = state)

def listener():
	rospy.init_node('findStair', anonymous=True)

	rospy.Subscriber("/sensor/kinect_rgb", Image, kin_callback)
	rospy.Subscriber("/pra_vale/estados", Int32, set_state)
	
	rospy.spin()

#main

print("Escada launched")
listener()
