#!/usr/bin/env python

import rospy
import defines as defs
from std_msgs.msg import Int32



#global variable
state =  (1 << defs._INITIAL_SETUP) | (1 << defs._ARM_CHANGING_POSE) | (1 << defs._ROBOT_DIR_RIGHT)



def print_state():
    global state
    string = ""
    if (state & (1 << defs._ENABLE_VELODYME)):
        string += ("  _ENABLE_VELODYME")

    if (state & (1 << defs._ARM_CHANGING_POSE)):
        string += ("  _ARM_CHANGING_POSE")

    if (state & (1 << defs._FOLLOW_TRACK)):
        string += ("  _FOLLOW_TRACK")

    if (state & (1 << defs._FOUND_FIRE_FRONT)):
        string += ("  _FOUND_FIRE_FRONT")

    if (state & (1 << defs._FOUND_FIRE_RIGHT)):
        string += ("  _FOUND_FIRE_RIGHT")

    if (state & (1 << defs._FOUND_FIRE_TOUCH)):
        string += ("  _FOUND_FIRE_TOUCH")  

    if (state & (1 << defs._SETTING_UP_HOKUYO)):
        string += ("  _SETTING_UP_HOKUYO")

    if (state & (1 << defs._INITIAL_SETUP)):
        string += ("  _INITIAL_SETUP")

    if (state & (1 << defs._ROBOT_ROTATION)):
        string += ("  _ROBOT_ROTATION")

    if (state & (1 << defs._HOKUYO_READING)):
        string += ("  _HOKUYO_READING")

    if (state & (1 << defs._ROBOT_DIR_RIGHT)):
        string += ("  _ROBOT_DIR_RIGHT")

    if (state & (1 << defs._LEAVING_FIRE)):
        string += ("  _LEAVING_FIRE")

    if (state & (1 << defs._NOTHING)):
        string += ("  _NOTHING")
    
    print(string)



#callback function called when a node requires a state change
def set_state(data):
    global state
    state = data.data

    #print the state change
    # print("\n\n\nstate changed: \n\n")
    # print_state()
    # print("\n\n\n")



#main
if __name__ == '__main__':
    rospy.init_node('state_handler', anonymous=True)
    rospy.Subscriber("/pra_vale/set_state", Int32, set_state)
    pub = rospy.Publisher('/pra_vale/estados', Int32, queue_size=1)

    print_state()
    node_sleep_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        print_state()
        pub.publish(data = state)
        node_sleep_rate.sleep()
