#!/usr/bin/env python

import rospy
import defines as defs

from std_msgs.msg import Int32



#global variable
state =  (1 << defs.INITIAL_SETUP) | (1 << defs.ARM_CHANGING_POSE) | (1 << defs.ROBOT_CLOCKWISE)



def print_state():
    global state
    string = ""

    if(state == 1 << defs.ROBOT_CLOCKWISE or state == 1 << defs.ROBOT_ANTICLOCKWISE):
        state |= 1 << defs.ENABLE_VELODYME

    #enable velodyne detection path planning
    if (state & (1 << defs.ENABLE_VELODYME)):
        string += ("  ENABLE_VELODYME")

    #moves the robot arm when the position desired is far away
    if (state & (1 << defs.ARM_CHANGING_POSE)):
        string += ("  ARM_CHANGING_POSE")

    #signalize that fire was found in front of the robot
    if (state & (1 << defs.FOUND_FIRE_FRONT)):
        string += ("  FOUND_FIRE_FRONT")
    
    #signalize that fire was found in the right of the robot
    if (state & (1 << defs.FOUND_FIRE_RIGHT)):
        string += ("  FOUND_FIRE_RIGHT")
    
    #when the arm is touching the fire
    if (state & (1 << defs.FOUND_FIRE_TOUCH)):
        string += ("  FOUND_FIRE_TOUCH")  
    
    #detects the fire easel with hokuyo
    if (state & (1 << defs.SETTING_UP_HOKUYO)):
        string += ("  SETTING_UP_HOKUYO")

    #used in the start of the simulation
    if (state & (1 << defs.INITIAL_SETUP)):
        string += ("  INITIAL_SETUP")

    #when the robot is rotatating along it's own z axis
    if (state & (1 << defs.ROBOT_ROTATION)):
        string += ("  ROBOT_ROTATION")

    #used to enable the HOKUYO processing
    if (state & (1 << defs.HOKUYO_READING)):
        string += ("  HOKUYO_READING")

    #when the robot is following the track with CLOKWISE direction
    if (state & (1 << defs.ROBOT_CLOCKWISE)):
        string += ("  ROBOT_CLOKWISE")

    #when the robot is following the track with CLOKWISE direction
    if (state & (1 << defs.ROBOT_ANTICLOCKWISE)):
        string += ("  ROBOT_ANTICLOKWISE")

    #when the robot just finished touching a roll on fire
    if (state & (1 << defs.LEAVING_FIRE)):
        string += ("  LEAVING_FIRE")

    #when the robot is doing NOTHING
    if (state & (1 << defs.NOTHING)):
        string += ("  NOTHING")

    #when the stairs where found
    if (state & (1 << defs.FOUND_STAIR)):
        string += ("  FOUND_STAIR")

    #when the stairs where found
    if (state & (1 << defs.NARROW_PATH)):
        string += ("  NARROW_PATH")

    #when the stairs where found
    if (state & (1 << defs.IN_STAIR)):
        string += ("  IN_STAIR")
    
    #when the stairs end
    if (state & (1 << defs.END_STAIR)):
        string += ("  END_STAIR")
    
    #when the stairs end
    if (state & (1 << defs.BEAM_FIND)):
        string += ("  BEAM_FIND")
        
    if (state & (1 << defs.CLIMB_STAIR)):
        string += ("  CLIMB_STAIR")
    
    print(string)



#callback function called when a node requires a state change
def set_state(data):
    global state
    
    if(state == data.data):
        return

    print state, " ",data.data

    if(state & (1 << defs.BEAM_FIND)):
        state = data.data
        state |= 1 << defs.BEAM_FIND
    else:
        state = data.data


    #print the state change (debug)
    if(defs.DEBUGGING):
        print_state()


#callback function called when a node requires a state change
def def_state(data):
    global state
    state_to_change = abs(data.data)
    if(data.data > 0):
        state |= 1 << state_to_change
    else:
        state &= ~(1 << state_to_change)

    #print the state change (debug)
    #print("state changed: ")
    if(defs.DEBUGGING):
        print_state()


def bean_state(data):
    global state
    if data.data == 0:
        state &= ~(1 << defs.BEAM_FIND)
    else:
        state |= 1 << defs.BEAM_FIND




#main
if __name__ == '__main__':
    rospy.init_node('state_handler', anonymous=True)
    rospy.Subscriber("/pra_vale/set_state", Int32, set_state)
<<<<<<< HEAD
    rospy.Subscriber("/pra_vale/beam_finder", Int32, bean_state)
=======
    rospy.Subscriber("/pra_vale/def_state", Int32, def_state)
>>>>>>> 86fc1aa052b5252f695cc411f44231c251ad6325
    pub = rospy.Publisher('/pra_vale/estados', Int32, queue_size=1)

    print_state() #print the intial state
    node_sleep_rate = rospy.Rate(10)

    while not rospy.is_shutdown(): #always keep publishing the state
        #print_state()
        pub.publish(data = state)
        node_sleep_rate.sleep()