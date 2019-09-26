#!/usr/bin/env python3

#sudo pip3 install rospkg catkin_pkg

import rospy
import readchar
from rosi_defy.msg import ManipulatorJoints
from sensor_msgs.msg import Joy
from math import pi
from math import pow
from math import sqrt
from math import atan2
from math import acos



def listener():
    rospy.init_node('listener', anonymous=True)

    step = 0.01
    global x
    global y
    global z

    #rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, callback)
    #rospy.Subscriber('/joy', Joy, callback_Joy)
    
    pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)
    #pos = [pi/2,0,0,0,-pi/2,0]
    pos = [4.577346777463163, -0.1397862826679197, -0.9876880430729198, 1.1274743257408393, -7.718939431052956, 0]
    #node_sleep_rate = rospy.Rate(10)
    while not rospy.is_shutdown():    
        #pos = cinematicaInversa()
        #pub.publish(joint_variable = pos)
        key = readchar.readkey()
        key = key.lower()
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
        pub.publish(joint_variable = pos)
        #node_sleep_rate.sleep()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()


