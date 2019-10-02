#!/usr/bin/env python

import rospy
from math import pi
import defines as defs
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

actual_z_angle = 0
desired_z_angle = 0
state = defs._NOTHING

half_pi = pi/2

_FRONT_ANGLE = 0
_RIGHT_ANGLE = 1
_LEFT_ANGLE  = 2

robot_on_right = [0,0,0]
robot_on_left = [0,0,0]

robot_on_right[_FRONT_ANGLE] = half_pi
robot_on_right[_LEFT_ANGLE] =  -half_pi
robot_on_right[_RIGHT_ANGLE] = 0

robot_on_left[_FRONT_ANGLE] = -half_pi
robot_on_left[_LEFT_ANGLE] =  0
robot_on_left[_RIGHT_ANGLE] = half_pi

rotation_direction = defs._CLOCKWISE
rotation_type = defs._NOTHING

#callback function called when a node requires a state change
def set_state(data):
    global state
    global half_pi
    global rotation_type
    global desired_z_angle
    global rotation_direction

    state = data.data
    if(state & (1 << defs._ROBOT_ROTATION) and rotation_type == defs._NOTHING):
        if(state & (1 << defs._FOUND_FIRE_RIGHT)):        


            desired_z_angle = robot_on_right[_FRONT_ANGLE]
            rotation_type = defs._TRACK_ON_FRONT

            if(actual_z_angle > -half_pi and actual_z_angle < half_pi):
                rotation_direction = defs._CLOCKWISE  
    
            else:
                rotation_direction = defs._ANTI_CLOCKWISE  
    



def imu_callback(data):
    global state
    global rotation_type
    global actual_z_angle

    actual_z_angle = data.data[2]
    if (rotation_type == defs._NOTHING):
        return

    rosi_speed = [0,0,0,0]
    if(rotation_direction == defs._CLOCKWISE):
        rosi_speed = [-2,-2,2,2]
    else:
        rosi_speed = [2,2,-2,-2]
    
    if (rotation_type == defs._TRACK_ON_FRONT):
        if (abs(actual_z_angle - desired_z_angle) < defs._MAX_JOINT_ANGLE_DIFF):
            rosi_speed = [0,0,0,0]
            rotation_type = defs._NOTHING
            state = state & ~(1 << defs._ROBOT_ROTATION) 
    

    speed_publisher = rospy.Publisher("/pra_vale/rosi_speed", Float32MultiArray, queue_size = 1)
    speed_publisher.publish(data = rosi_speed)

    if(not (state & 1 << defs._ROBOT_ROTATION)):
        state_publisher = rospy.Publisher('/pra_vale/estados', Int32, queue_size=1)
        state_publisher.publish(data = state)
        


    




#main
if __name__ == '__main__':
    # global half_pi
    # global robot_on_left
    # global robot_on_right

    rospy.init_node('robot_roation', anonymous=True)
    rospy.Subscriber("/pra_vale/estados", Int32, set_state)
    rospy.Subscriber("/pra_vale/imu", Float32MultiArray, imu_callback)

    rospy.spin()