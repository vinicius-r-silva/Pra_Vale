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
_FOLLOW_ANGLE = 1

robot_on_right = [0,0]
robot_on_left = [0,0]

robot_on_left[_FRONT_ANGLE] = half_pi
robot_on_left[_FOLLOW_ANGLE] = 0

robot_on_right[_FRONT_ANGLE] = -half_pi
robot_on_right[_FOLLOW_ANGLE] = half_pi

rotation_direction = defs._CLOCKWISE

_NO_ANGLE = -10

#callback function called when a node requires a state change
def set_state(data):
    global state
    global half_pi
    global desired_z_angle
    global rotation_direction

    state = data.data
    track_on_right = False
    if(actual_z_angle > -half_pi and actual_z_angle < half_pi):
        track_on_right = True


    if(state & (1 << defs._ROBOT_ROTATION) and desired_z_angle == _NO_ANGLE):
        if (state & 1 << defs._ROBOT_DIR_RIGHT):
            if(state & (1 << defs._FOUND_FIRE_RIGHT)):       
                rotation_direction = defs._CLOCKWISE  
                if(track_on_right):
                    desired_z_angle = robot_on_left[_FRONT_ANGLE]
                else:
                    desired_z_angle = robot_on_right[_FRONT_ANGLE]

            elif(state & (1 << defs._FOUND_FIRE_TOUCH)):
                rotation_direction = defs._ANTI_CLOCKWISE
                if(track_on_right):
                    desired_z_angle = robot_on_left[_FOLLOW_ANGLE]
                else:
                    desired_z_angle = robot_on_right[_FOLLOW_ANGLE]
                


    


def imu_callback(data):
    global state
    global actual_z_angle
    global desired_z_angle

    actual_z_angle = data.data[2]
    if (desired_z_angle == _NO_ANGLE):
        return

    rosi_speed = [0,0,0,0]
    if(rotation_direction == defs._CLOCKWISE):
        rosi_speed = [-2,-2,2,2]
    else:
        rosi_speed = [2,2,-2,-2]
    
    if (abs(actual_z_angle - desired_z_angle) < defs._MAX_JOINT_ANGLE_DIFF):
        rosi_speed = [0,0,0,0]
        desired_z_angle = _NO_ANGLE
        state &= ~(1 << defs._ROBOT_ROTATION) 
        state_publisher = rospy.Publisher('/pra_vale/set_state', Int32, queue_size=1)
        state_publisher.publish(data = state)
    

    speed_publisher = rospy.Publisher("/pra_vale/rosi_speed", Float32MultiArray, queue_size = 1)
    speed_publisher.publish(data = rosi_speed)
        


    




#main
if __name__ == '__main__':
    # global half_pi
    # global robot_on_left
    # global robot_on_right

    rospy.init_node('robot_roation', anonymous=True)
    rospy.Subscriber("/pra_vale/estados", Int32, set_state)
    rospy.Subscriber("/pra_vale/imu", Float32MultiArray, imu_callback)

    rospy.spin()