#!/usr/bin/env python

import rospy
from math import pi
from math import pow
from math import acos
from math import sqrt
from math import atan2
from getkey import getkey, keys
from rosi_defy.msg import ManipulatorJoints


#use a,s,d,f,g,h,z,x,c,c,v,b,n keys to move the joints of the arm
def listener():
    rospy.init_node('arm_joint', anonymous=True)

    #step (in rads) wich the arm joint position is incremented
    step = 0.01
    
    #Publishes to the ur5 joints topic
    arm_publisher = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)
    
    #initial pose
    pos = [4.577346777463163, -0.1397862826679197, -0.9876880430729198, 1.1274743257408393, -7.718939431052956, 0]

    while not rospy.is_shutdown():    
        #read a key and check if it means a increment
        #a,z increments/decrements the first   joint
        #s,x increments/decrements the seconds joint
        #d,c increments/decrements the third   joint
        #f,v increments/decrements the fourth  joint
        #g,b increments/decrements the fifth   joint
        #h,n increments/decrements the sixth   joint
        key = getkey()
        if key == 'a':
            pos[0] += step
        if key == 'z':
            pos[0] -= step
            
        if key == 's':
            pos[1] += step
        if key == 'x':
            pos[1] -= step

        if key == 'd':
            pos[2] += step
        if key == 'c':
            pos[2] -= step

        if key == 'f':
            pos[3] += step
        if key == 'v':
            pos[3] -= step

        if key == 'g':
            pos[4] += step
        if key == 'b':
            pos[4] -= step

        if key == 'h':
            pos[5] += step
        if key == 'n':
            pos[5] -= step

        if key == '.':
            print ("finished")
            break
        
        print(str(pos[0]) + ", " + str(pos[1]) + ", " + str(pos[2]) + ", " + str(pos[3]) + ", " + str(pos[4]) + ", " + str(pos[5]))
        arm_publisher.publish(joint_variable = pos)
        #node_sleep_rate.sleep()


#main
print("arm joint launched")
listener()


