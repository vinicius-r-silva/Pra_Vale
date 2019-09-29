#!/usr/bin/env python3
#sudo pip3 install rospkg catkin_pkg

import rospy
import readchar
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import pi
from math import pow
from math import sqrt
from math import atan2
from math import acos

def listener():
    rospy.init_node('listener', anonymous=True)

    step = 10
    global x
    global y
    global z

    #rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, callback)
    #rospy.Subscriber('/joy', Joy, callback_Joy)

    arm_publisher = rospy.Publisher('/pra_vale/arm_move', Int32MultiArray, queue_size=10)


    node_sleep_rate = rospy.Rate(3)
    pos = [0,0,0]
    while not rospy.is_shutdown():    
        x = 0
        y = 0
        z = 0

        key = readchar.readkey()
        key = key.lower()
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

        if key == '.':
            print ("finished")
            break
        
        pos[0] = x
        pos[1] = y
        pos[2] = z

        arm_publisher.publish(data = pos)
        node_sleep_rate.sleep()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()


