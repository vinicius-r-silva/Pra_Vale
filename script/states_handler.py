#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

#defined states
_NOTHING = 0
_ENABLE_VELODYME = 1
_ARM_CHANGING_POSE = 2
_FOLLOW_TRACK = 3
_FOUND_FIRE_FRONT = 4
_FOUND_FIRE_RIGHT = 5
_FOUND_FIRE_TOUCH = 6
_SETTING_UP_HOKUYO = 7
_INITIAL_SETUP     = 8


#global variable
state =  (1 << _INITIAL_SETUP) | (1 << _ARM_CHANGING_POSE)


#callback function called when a node requires a state change
def set_state(data):
    global state
    state = data.data

    #print the state change
    print("\n\n\nstate changed: \n\n")
    if (state & (1 << _ENABLE_VELODYME)):
        print ("_ENABLE_VELODYME")

    if (state & (1 << _ARM_CHANGING_POSE)):
        print ("_ARM_CHANGING_POSE")

    if (state & (1 << _FOLLOW_TRACK)):
        print ("_FOLLOW_TRACK")

    if (state & (1 << _FOUND_FIRE_FRONT)):
        print ("_FOUND_FIRE_FRONT")

    if (state & (1 << _FOUND_FIRE_RIGHT)):
        print ("_FOUND_FIRE_RIGHT")

    if (state & (1 << _FOUND_FIRE_TOUCH)):
        print ("_FOUND_FIRE_TOUCH")  

    if (state & (1 << _SETTING_UP_HOKUYO)):
        print ("_SETTING_UP_HOKUYO")
    print("\n\n\n")



#main
if __name__ == '__main__':
    rospy.init_node('state_handler', anonymous=True)
    rospy.Subscriber("/pra_vale/set_state", Int32, set_state)
    pub = rospy.Publisher('/pra_vale/estados', Int32, queue_size=1)

    print("state_handler launched")
    if (state & (1 << _ENABLE_VELODYME)):
        print ("_ENABLE_VELODYME")

    if (state & (1 << _ARM_CHANGING_POSE)):
        print ("_ARM_CHANGING_POSE")

    if (state & (1 << _FOLLOW_TRACK)):
        print ("_FOLLOW_TRACK")

    if (state & (1 << _FOUND_FIRE_FRONT)):
        print ("_FOUND_FIRE_FRONT")

    if (state & (1 << _FOUND_FIRE_RIGHT)):
        print ("_FOUND_FIRE_RIGHT")

    if (state & (1 << _FOUND_FIRE_TOUCH)):
        print ("_FOUND_FIRE_TOUCH")  

    if (state & (1 << _SETTING_UP_HOKUYO)):
        print ("_SETTING_UP_HOKUYO")

    if (state & (1 << _INITIAL_SETUP)):
        print ("_SETTING_UP_HOKUYO")

    node_sleep_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(data = state)
        node_sleep_rate.sleep()
