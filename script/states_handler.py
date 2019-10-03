##!/usr/bin/env python

import rospy
import defines as defs

from std_msgs.msg import Int32



#global variable
state =  (1 << defs._INITIAL_SETUP) | (1 << defs._ARM_CHANGING_POSE)



def print_state():
    global state
    string = ""

    #enable velodyne detection path planning
    if (state & (1 << defs._ENABLE_VELODYME)):
        string += ("  _ENABLE_VELODYME")

    #moves the robot arm when the position desired is far away
    if (state & (1 << defs._ARM_CHANGING_POSE)):
        print ("_ARM_CHANGING_POSE")

    #keeps the robot arm aligned to the track
    if (state & (1 << defs._FOLLOW_TRACK)):
        print ("_FOLLOW_TRACK")

    #signalize that fire was found in front of the robot
    if (state & (1 << defs._FOUND_FIRE_FRONT)):
        print ("_FOUND_FIRE_FRONT")
    
    #signalize that fire was found in the right of the robot
    if (state & (1 << defs._FOUND_FIRE_RIGHT)):
        print ("_FOUND_FIRE_RIGHT")
    
    #when the arm is touching the fire
    if (state & (1 << defs._FOUND_FIRE_TOUCH)):
        print ("_FOUND_FIRE_TOUCH")  
    
    #detects the fire easel with hokuyo
    if (state & (1 << defs._SETTING_UP_HOKUYO)):
        print ("_SETTING_UP_HOKUYO")

    if (state & (1 << defs._INITIAL_SETUP)):
        string += ("  _INITIAL_SETUP")

    if (state & (1 << defs._ROBOT_ROTATION)):
        string += ("  _ROBOT_ROTATION")
    
    print(string)



#callback function called when a node requires a state change
def set_state(data):
    global state
    state = data.data

    #print the state change
    print("\n\n\nstate changed: \n\n")
    print_state()
    print("\n\n\n")



#main
if __name__ == '__main__':
    rospy.init_node('state_handler', anonymous=True)
    rospy.Subscriber("/pra_vale/set_state", Int32, set_state)
    pub = rospy.Publisher('/pra_vale/estados', Int32, queue_size=1)

    print_state()
    node_sleep_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(data = state)
        node_sleep_rate.sleep()