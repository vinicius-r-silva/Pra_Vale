#!/usr/bin/env python

import rospy
from getkey import getkey
from std_msgs.msg import Int32MultiArray


#use a,w,s,d,e,q keys to move the arm
def listener():
    rospy.init_node('arm_joy', anonymous=True)
    arm_publisher = rospy.Publisher('/pra_vale/arm_move', Int32MultiArray, queue_size=10)

    #step (in mm) wich the arm position is incremented
    step = 10
    pos = [0,0,0]

    node_sleep_rate = rospy.Rate(10)
    while not rospy.is_shutdown():    
        x = 0
        y = 0
        z = 0

        #read a key and check if it means a increment
        #w,s increments/decrements at x axis
        #a,d increments/decrements at y axis
        #e,q increments/decrements at z axis
        key = getkey()
        if key in ('wsadqe'):
            print ('Key:' + key)
            if key == 'w':
                x = step
            if key == 's':
                x = -step

            if key == 'a':
                y = step
            if key == 'd':
                y = -step
            
            if key == 'e':
                z = step
            if key == 'q':
                z = -step

        #'.' key finishes the program
        if key == '.':
            print ("finished")
            break
        
        #publishes the colected key
        pos[0] = x
        pos[1] = y
        pos[2] = z
        arm_publisher.publish(data = pos)

        node_sleep_rate.sleep()


#main
print("arm_joy launched")
listener()


