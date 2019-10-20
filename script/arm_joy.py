#!/usr/bin/env python

import rospy
from getkey import getkey
from std_msgs.msg import Int32MultiArray
import defines as defs
from rosi_defy.msg import ManipulatorJoints  

import new_arm as arm


#use a,w,s,d,e,q keys to move the arm
def listener():
    rospy.init_node('arm_joy', anonymous=True)
    #publish joint values to ur5 arm
    arm_publisher = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)

    #step (in mm) wich the arm position is incremented
    step = 1
    pos = [0,0,0]

    state = 1 << defs.ROBOT_CLOCKWISE 
    x = 320
    y = 71
    z = 925

    node_sleep_rate = rospy.Rate(10)
    while not rospy.is_shutdown():   

        #read a key and check if it means a increment
        #w,s increments/decrements at x axis
        #a,d increments/decrements at y axis
        #e,q increments/decrements at z axis
        key = getkey()
        if key in ('wsadqe'):
            print ('Key:' + key)
            if key == 'w':
                x += step
            if key == 's':
                x += -step

            if key == 'a':
                y += step
            if key == 'd':
                y += -step
            
            if key == 'e':
                z += step
            if key == 'q':
                z += -step

        #'.' key finishes the program
        if key == '.':
            print ("finished")
            break
        
        #publishes the colected key
        arm.x = x
        arm.y = y
        arm.z = z

        print([x,y,z])
        pos = arm.cinematicaInversa(state)
        arm_publisher.publish(joint_variable = pos)

        node_sleep_rate.sleep()


#main
print("arm_joy launched")
listener()


