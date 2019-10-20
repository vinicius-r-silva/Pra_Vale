#!/usr/bin/env python

import rospy
from math import pi
import defines as defs
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray


#-----------------------CONSTS----------------------#   
#used in the robot_on_left/right vector
_FRONT_ANGLE = 0
_FOLLOW_ANGLE = 1

#when the robot is not rotating, the desired angle is equal to _NO_ANGLE
_NO_ANGLE = -10

#used multiple times, saves times from dividing every time
half_pi = pi/2


#-------------------GLOBAL VARIABLES----------------#    
actual_z_angle = 0
desired_z_angle = 0
state = defs.NOTHING 

#robot_on_right e robot_on_left defines wich angles the robot has to move to
robot_on_right = [0,0]
robot_on_left = [0,0]

#set defined angles to the robot direction
robot_on_left[_FRONT_ANGLE] = half_pi
robot_on_left[_FOLLOW_ANGLE] = 0

robot_on_right[_FRONT_ANGLE] = -half_pi
robot_on_right[_FOLLOW_ANGLE] = half_pi

#defines the rotation direction of the robot
rotation_direction = defs.ROBOT_CLOCKWISE

#publish robot state changes to others ROS packages
state_publisher = rospy.Publisher('/pra_vale/def_state', Int32, queue_size=15)


#callback function called when a node requires a state change
def get_state(data):
    global state
    global half_pi
    global desired_z_angle
    global rotation_direction

    #get robot current state
    state = data.data

    #check if the track is on the rigth side or in the left side of the robot
    track_on_right = False
    if(actual_z_angle > -half_pi and actual_z_angle < half_pi):
        track_on_right = True

    #if the robot current state contains "ROBOT_ROTATION" and desired_z_angle was not initialize yet, initialize it
    if(state & (1 << defs.ROBOT_ROTATION) and desired_z_angle == _NO_ANGLE):
        #if the robot is Clockwise, check what type of roation is required
        if (state & 1 << defs.ROBOT_CLOCKWISE):
            #if FOUND_FIRE_RIGHT is set, then is necessary to rotate the robot to be facing the fire
            if(state & (1 << defs.FOUND_FIRE_RIGHT)):       
                rotation_direction = defs.ROBOT_CLOCKWISE
                if(track_on_right):
                    desired_z_angle = robot_on_left[_FRONT_ANGLE]
                else:
                    desired_z_angle = robot_on_right[_FRONT_ANGLE]

            #if FOUND_FIRE_TOUCH is set, then is necessary to rotate the robot to the fire be in it's left side
            elif(state & (1 << defs.FOUND_FIRE_TOUCH)):
                rotation_direction = defs.ROBOT_ANTICLOCKWISE
                if(track_on_right):
                    desired_z_angle = robot_on_left[_FOLLOW_ANGLE]
                else:
                    desired_z_angle = robot_on_right[_FOLLOW_ANGLE]
                


    
#get robot actual euler angles
#if the the robot is currently rotating, then check if the rotation is over
def imu_callback(data):
    global state
    global actual_z_angle
    global desired_z_angle
    global state_publisher

    #get robot z axis orientation
    actual_z_angle = data.data[2]

    if(state & (1 << defs.ROBOT_CLOCKWISE)):
        if(actual_z_angle > -half_pi and actual_z_angle < half_pi and (not state & (1 << defs.ROBOT_ON_THE_LEFT))):
            state_publisher.publish(data = defs.ROBOT_ON_THE_LEFT)
        if(actual_z_angle < -half_pi or actual_z_angle > half_pi and (state & (1 << defs.ROBOT_ON_THE_LEFT))):
            state_publisher.publish(data = -defs.ROBOT_ON_THE_LEFT)
    else:
        if(actual_z_angle > -half_pi and actual_z_angle < half_pi and (not state & (1 << defs.ROBOT_ON_THE_LEFT))):
            state_publisher.publish(data = -defs.ROBOT_ON_THE_LEFT)
        if(actual_z_angle < -half_pi or actual_z_angle > half_pi and (state & (1 << defs.ROBOT_ON_THE_LEFT))):
            state_publisher.publish(data = defs.ROBOT_ON_THE_LEFT)


    #if the robot isn't rotating, then return
    if (desired_z_angle == _NO_ANGLE):
        return

    #otherwise, keeps the robot rotating or stop the rotation when the robot reachs certain angle
    rosi_speed = [0,0,0,0]
    if(rotation_direction == defs.ROBOT_CLOCKWISE):
        rosi_speed = [-2,-2,2,2]
    else:
        rosi_speed = [2,2,-2,-2]
    
    if (abs(actual_z_angle - desired_z_angle) < defs.MAX_JOINT_ANGLE_DIFF):
        rosi_speed = [0,0,0,0]
        desired_z_angle = _NO_ANGLE
        state_publisher = rospy.Publisher('/pra_vale/def_state', Int32, queue_size=1)
        state_publisher.publish(data = -defs.ROBOT_ROTATION)
    
    #publish the robot wheels speed
    speed_publisher = rospy.Publisher("/pra_vale/rosi_speed", Float32MultiArray, queue_size = 1)
    speed_publisher.publish(data = rosi_speed)
        


    




#main
if __name__ == '__main__':

    rospy.init_node('robot_roation', anonymous=True)
    rospy.Subscriber("/pra_vale/estados", Int32, get_state)
    rospy.Subscriber("/pra_vale/imu", Float32MultiArray, imu_callback)

    rospy.spin()