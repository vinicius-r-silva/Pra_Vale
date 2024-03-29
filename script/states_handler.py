#!/usr/bin/env python

import rospy
import defines as defs

from std_msgs.msg import Int32



#Global variable of all states
#Each byte is a state
state =  (1 << defs.INITIAL_SETUP) | (1 << defs.ARM_CHANGING_POSE) | (1 << defs.ROBOT_CLOCKWISE) | (1 << defs.ROBOT_ON_THE_LEFT) 



def print_state():
    global state
    string = ""

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

    #when you are climbing the stairs
    if (state & (1 << defs.CLIMB_STAIR)):
        string += ("  CLIMB_STAIR")

    if (state & (1 << defs.BEAM_FIND)):
        string += ("  BEAM_FIND")

    #when the robot is on the left side of the track
    if (state & (1 << defs.ROBOT_ON_THE_LEFT)):
        string += ("  ROBOT_ON_THE_LEFT")
    else:
        string += ("  ROBOT_ON_THE_RIGHT")

    #when a fire was found by the UR5 cam
    if (state & (1 << defs.FIRE_FOUND_BY_CAM)):
        string += ("  FIRE_FOUND_BY_CAM")

    #force velodyne to execute
    if (state & (1 << defs.FORCE_VELODYME)):
        string += ("  FORCE_VELODYME")
        
    
    print(string)



#callback function called when a node requires a state change
def set_state(data):
    global state

    print (state, " ",data.data)

    state = data.data

    #print the state change (debug)
    print("state changed is deprecated")
    if(defs.PRINT_STATES):
        print_state()


#callback function called when a node requires a state change
def def_state(data):
    global state
    state_changed = False
    state_to_change = abs(data.data)

    if(data.data > 0 and (not state & (1 << state_to_change))):
        state |= 1 << state_to_change
        state_changed = True
    elif(data.data < 0 and (state & (1 << state_to_change))):
        state &= ~(1 << state_to_change)
        state_changed = True

    #print the state change (debug)
    #print("state changed: ")
    if(state_changed and defs.PRINT_STATES):
        print_state()



#main
if __name__ == '__main__':
    rospy.init_node('state_handler', anonymous=True)
    rospy.Subscriber("/pra_vale/set_state", Int32, set_state)
    rospy.Subscriber("/pra_vale/def_state", Int32, def_state)
    pub = rospy.Publisher('/pra_vale/estados', Int32, queue_size=1)

    print("state handler launched")
    print_state() #print the intial state
    node_sleep_rate = rospy.Rate(10)

    while not rospy.is_shutdown(): #always keep publishing the state
        #print_state()
        pub.publish(data = state)
        node_sleep_rate.sleep()