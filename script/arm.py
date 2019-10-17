#!/usr/bin/env python

import rospy
from math import pi
from math import pow
from math import sqrt
from math import atan2
from math import acos
from math import asin
from math import sin
from math import cos
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from rosi_defy.msg import RosiMovement
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints  
import defines as defs
import numpy as np

#-----------------------CONSTS----------------------#   
#const received form the ur5_Cam when no fire is detected
_FIRE_NOT_FOUND = 1000

#consts for the initial pose for each case
_FIRE_TOUCH_X_VALUE =  450
_FIRE_TOUCH_Y_VALUE =  0
_FIRE_TOUCH_Z_VALUE =  410
_FIRE_FOUND_X_VALUE =  320
_FIRE_FOUND_Y_VALUE =  -70
_FIRE_FOUND_Z_VALUE =  800
_NARROW_PATH_X_VALUE =  400
_NARROW_PATH_Y_VALUE =  80
_NARROW_PATH_Z_VALUE =  900
_FIRE_NOT_FOUND_X_VALUE =  320
_FIRE_NOT_FOUND_Y_VALUE =  -70
_FIRE_NOT_FOUND_Z_VALUE =  925

#defines the max distance from the robot to the fire to check if the arm is capable of reaching it
_TOUCH_FIRE_FRONT_DISTANCE = 70

#the hokuyo it's the center of the arm, but the torque pointer it isn't
#thus, this defines how much the HOKUYO has to deslocate in the x axis to the pointer be in the center
_HOKUYO_DISPLACEMENT = -3

_IMU_TILT_ERROR = 0.1



#-------------------GLOBAL VARIABLES----------------#    
#publish joint values to ur5 arm
arm_publisher = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)

#publish robot state changes to others ROS packages
state_publisher = rospy.Publisher('/pra_vale/def_state', Int32, queue_size=15)

#publishes ROSI speed
rosi_speed_publisher = rospy.Publisher("/pra_vale/rosi_speed", Float32MultiArray, queue_size = 1)

#used to check wether the y axis tilt is going to be detect by the imu or by the ur5 camera
get_tilt_y_from_imu = False

#it's true when a fire was detect by the ur5 camera
fire_found = False

#the hokuyo distance when the hokuyo node is not processing
hokuyo_distance = -1

#torque value given from
torque_value = 0

robot_last_orientation = defs.ROBOT_CLOCKWISE

#arm sizes
probe_lenght = 71 #279
d0 = 275 #367
l1 = 425 #425.1
l2 = 392 #392.1
l3 = 111  #126

#consts
probe_lenght_pow = probe_lenght*probe_lenght
l1_pow = l1*l1
l2_pow = l2*l2

#initial pose
x = _FIRE_NOT_FOUND_X_VALUE
y = _FIRE_NOT_FOUND_Y_VALUE
z = _FIRE_NOT_FOUND_Z_VALUE
last_x = x
last_y = y
last_z = z
tilt_x = 0
tilt_y = 0
tilt_z = 0

#conts wich show wich state the system is currently, see at the states defines
state = defs.NOTHING

#current desired arm joint angles
joint_angles = [0,0,0,0,0,0]

camera_tilt = 0.0

leaving_fire_cont = 0

narrow_path = False


#return UR5 arm joints angles given the desired position
def cinematicaInversa():
    global x
    global y
    global z

    global last_x
    global last_y
    global last_z

    global tilt_x
    global tilt_y
    global tilt_z

    global state
    global joint_angles
    global state_publisher

    #print("x: " + str(x) + "  y: " + str(y) + "  z: " + str(z))

    if(x < 0):
        print("erro: posicao fora do alcance do braco robotico")

        #change to the last valid position
        x = last_x
        y = last_y
        z = last_z
        #recalculates joint angles to the last valid position
        return joint_angles


    #a error is given when the desired position is invalid
    try:
        #main calculations
        x_const = x - d0
        x_const_pow = x_const*x_const

        y_const = y
        y_const_pow = y_const*y_const

        base_dist = sqrt(x_const_pow + y_const_pow)
        d = sqrt(x_const_pow + y_const_pow - probe_lenght_pow)

        d_const = d
        z_const = z - l3

        l_const_pow = d_const*d_const + z_const*z_const
        l_const = sqrt(l_const_pow)

        t1_const = acos((l1_pow - l2_pow + l_const_pow) / (2*l1*l_const))
        t2_const = acos((l1_pow + l2_pow - l_const_pow) / (2*l1*l2))
        t3_const = pi - t1_const - t2_const

        s1 = acos(z_const/l_const)
        s2 = acos(d_const/l_const)

        #joint angles
        if(state & (1 << defs.ROBOT_CLOCKWISE)):
            joint_angles[0] = -pi/2 - acos(probe_lenght/base_dist) + atan2(y_const, x_const)
            joint_angles[1] = -pi/2 + (t1_const + s2)
            joint_angles[2] = -pi + t2_const
            joint_angles[3] = t3_const + s1
            joint_angles[4] = -joint_angles[0] - pi
        else:
            joint_angles[0] = acos(probe_lenght/base_dist) + atan2(y_const, x_const)
            joint_angles[1] = pi/2 - (t1_const + s2)
            joint_angles[2] = pi - t2_const
            joint_angles[3] = -t3_const - s1
            joint_angles[4] = -joint_angles[0]


        #check if t0 and t4 angles exceeds 2*pi
        if(joint_angles[0] > 2*pi):
            joint_angles[0] -= 2*pi
        elif(joint_angles[0] < -2*pi):
            joint_angles[0] += 2*pi
        # if(joint_angles[4] > 2*pi):
        #     joint_angles[4] -= 2*pi
        # elif(joint_angles[4] < -2*pi):
        #     joint_angles[4] += 2*pi

        #configure tilt
        joint_angles[0] += tilt_z
        joint_angles[1] += tilt_x
        joint_angles[5] = tilt_y

        #update last postion
        last_x = x
        last_y = y
        last_z = z

        # print the joint angles for debuging
        # print(joint_angles)
        #print("x: " + str(x) + ",  y: " + str(y) + ",  z: " + str(z))            
        return joint_angles
    
    except:
        print("erro: posicao fora do alcance do braco robotico")

        #change to the last valid position
        x = last_x
        y = last_y
        z = last_z
        #recalculates joint angles to the last valid position
        return joint_angles

#receive an absolute x,y,z position and sets in the simulation
def arm_pos(data):
    global state
    if(state & 1 << defs.ARM_CHANGING_POSE):
        return

    global x
    global y
    global z
    global arm_publisher
    #print(data) #debug

    x = data.data[0]
    y = data.data[1]
    z = data.data[2]
    
    #get the joints angles
    pos = cinematicaInversa()
    #publish joints angles
    arm_publisher.publish(joint_variable = pos)


#callback from the ur5 camera when it detects fire
#receive incremental x,y,z position and sets in the simulation
#the ur5 camera publishes the x distance between the arm and the fire
def arm_move(data):
    #globals
    global x
    global y
    global z
    global state
    global fire_found
    global narrow_path
    global arm_publisher
    global state_publisher
    global rosi_speed_publisher
    global leaving_fire_cont


    #if the arm is changing position or the robot is rotating, the arm it's not supose to change its position
    if((state & (1 << defs.ARM_CHANGING_POSE | 1 << defs.ROBOT_ROTATION | 1 << defs.CLIMB_STAIR))):
        return        
    #print(data) #debug

    x_move = data.data[0]
    y_move = data.data[1]
    z_move = data.data[2]

    #if the robot it's on right side of the track, the x and y axis are swaped
    if(not state & (1 << defs.ROBOT_CLOCKWISE)):
        temp = x_move
        x_move = y_move
        y_move = temp

    #to debug only
    print("xmove : " + str(x_move) + "  y_move: " + str(y_move))
    
    #update arm position if there isn't a fire
    # if((x_move == _FIRE_NOT_FOUND or y_move == _FIRE_NOT_FOUND) and not state & (1 << defs.FOUND_FIRE_FRONT | 1 << defs.FOUND_FIRE_TOUCH)):
    #     print("aqui 1")
    #     #if the arm just finished leaving the fire location
    #     #return the arm normal position of the arm
    #     if(fire_found):
    #         fire_found = False
    #         # state |= 1 << defs.ARM_CHANGING_POSE
    #         # state_publisher.publish(data = state)
    #         state_publisher.publish(data = defs.ARM_CHANGING_POSE)

    #         if(narrow_path):
    #             x = _NARROW_PATH_X_VALUE
    #             y = _NARROW_PATH_Y_VALUE
    #             z = _NARROW_PATH_Z_VALUE
    #         else:
    #             x = _FIRE_NOT_FOUND_X_VALUE
    #             y = _FIRE_NOT_FOUND_Y_VALUE
    #             z = _FIRE_NOT_FOUND_Z_VALUE

    #     #otherwise, just move the arm to the required position
    #     else:
    #         x += x_move
    #         y += y_move
    #         z += z_move

    #     #if the is still leaving the fire, keep the LEAVING_FIRE state up
    #     if (state & (1 << defs.LEAVING_FIRE)):
    #         if(leaving_fire_cont > 10):
    #             # state &= ~(1 << defs.LEAVING_FIRE)
    #             # state_publisher.publish(data = state)
    #             state_publisher.publish(data = -defs.LEAVING_FIRE)
    #             fire_found = False

    #         else:
    #             leaving_fire_cont += 1

    #if the arm is detecting a fire, but didn't touch it yet
    #elif(not state & (1 << defs.LEAVING_FIRE)):
    if(not state & (1 << defs.LEAVING_FIRE)):
        #if the arm just detected the fire, change it's position 
        if(not fire_found):
            fire_found = True
            # state |= 1 << defs.ENABLE_VELODYME | 1 << defs.SETTING_UP_HOKUYO | 1 << defs.ARM_CHANGING_POSE
            # state_publisher.publish(data = state)
            state_publisher.publish(data = defs.ENABLE_VELODYME)
            state_publisher.publish(data = defs.SETTING_UP_HOKUYO)
            state_publisher.publish(data = defs.ARM_CHANGING_POSE)

            rosi_speed_publisher.publish(data = [0,0,0,0,0,0])
            x = _FIRE_FOUND_X_VALUE
            y = _FIRE_FOUND_Y_VALUE
            z = _FIRE_FOUND_Z_VALUE
            if(state & (1 << defs.ROBOT_ANTICLOCKWISE)):
                z -= 10
                x += 100
        else:
            if(not state & (1 << defs.FOUND_FIRE_FRONT | 1 << defs.FOUND_FIRE_TOUCH)):
                x += x_move
                y += y_move
                z += z_move
                #print("move x : " + str(x))
            

            if(state & (1 << defs.SETTING_UP_HOKUYO)):
                if(x_move == 0 and y_move == 0):
                    # state |= 1 << defs.HOKUYO_READING
                    # state_publisher.publish(data = state)
                    state_publisher.publish(data = defs.HOKUYO_READING)
                    rosi_speed_publisher.publish(data = ([0,0,0,0]))

                elif(x_move < 0 or y_move > 0):
                    # state &= ~(1 << defs.ENABLE_VELODYME)
                    # state |= 1 << defs.SETTING_UP_HOKUYO
                    # state_publisher.publish(data = state)
                    state_publisher.publish(data = -defs.ENABLE_VELODYME)
                    state_publisher.publish(data =  defs.SETTING_UP_HOKUYO)
                    rosi_speed_publisher.publish(data = ([-0.2,-0.2,-0.2,-0.2]))
                
                elif(x_move < 10 and y_move > -10):
                    # state &= ~(1 << defs.ENABLE_VELODYME)
                    # state |= 1 << defs.SETTING_UP_HOKUYO
                    # state_publisher.publish(data = state)
                    state_publisher.publish(data = -defs.ENABLE_VELODYME)
                    state_publisher.publish(data =  defs.SETTING_UP_HOKUYO)
                    rosi_speed_publisher.publish(data = ([0.2,0.2,0.2,0.2]))


            elif(x < 300 and (state & (1 << defs.FOUND_FIRE_RIGHT))):
                z = _FIRE_FOUND_Z_VALUE
                rosi_speed_publisher.publish(data = [0,0,0,0])

                # state &= ~(1 << defs.ENABLE_VELODYME)
                # state |= 1 << defs.ROBOT_ROTATION 
                # state_publisher.publish(data = state)
                state_publisher.publish(data = -defs.ENABLE_VELODYME)
                state_publisher.publish(data =  defs.ROBOT_ROTATION)

    #get the joints angles
    pos = cinematicaInversa()
    #publish joints angles
    arm_publisher.publish(joint_variable = pos)


#callback from the IMU sensor, make the ur5 arm follow the track automatically
def arm_imu(data):
    global state
    if(state & (1 << defs.ARM_CHANGING_POSE | 1 << defs.CLIMB_STAIR)):
        return
  
    global tilt_x
    global tilt_y
    global tilt_z
    global arm_publisher
    global get_tilt_y_from_imu

    temp_y = 0.0
    angles = data.data
    if(state & (1 << defs.ROBOT_CLOCKWISE)):
        tilt_x = angles[0]
        temp_y = -angles[1] #+ 0.065
        tilt_z = angles[2]
    else:
        tilt_x = -angles[0]
        temp_y = -angles[1] - 0.065
        tilt_z = angles[2]
        if(tilt_z > pi):
            tilt_z = tilt_z - 2*pi
    
    if get_tilt_y_from_imu:
        tilt_y = temp_y
    else:
        tilt_y = camera_tilt

        if tilt_y > temp_y + _IMU_TILT_ERROR:
            tilt_y = temp_y + _IMU_TILT_ERROR

        elif tilt_y < temp_y - _IMU_TILT_ERROR:
            tilt_y = temp_y - _IMU_TILT_ERROR
    
        
    pos = cinematicaInversa()
    #publish joints angles
    arm_publisher.publish(joint_variable = pos)



#callback from the ur5 camera, make the ur5 arm follow the track automatically
def arm_tilt(data):
    global state
    global camera_tilt 
    global get_tilt_y_from_imu
    if((state & (1 << defs.ARM_CHANGING_POSE | 1 << defs.CLIMB_STAIR))): #or (state & 1 << defs.ROBOT_ROTATION)):
        return

    if(data.data == -1 or state & (1 << defs.FOUND_FIRE_FRONT) or state & (1 << defs.FOUND_FIRE_TOUCH)):
        get_tilt_y_from_imu = True
    else:
        if(get_tilt_y_from_imu == True):
            get_tilt_y_from_imu = False
            camera_tilt = 0.0
        
        camera_tilt += data.data #- 0.1


#get arm joint angles
#used when it's necessary to the robot to wait the arm reachs it's designed position
def arm_current_position(data):
    global state
    global joint_angles
    global rosi_speed_publisher
    global state_publisher
         
    if(state & (1 << defs.ARM_CHANGING_POSE)):

        #if all joint angles are in the correct places, remove the _ARM_CHANGING_POSE from the states
        for x in range(len(joint_angles)):
            if(abs(joint_angles[x] - data.joint_variable[x]) > defs.MAX_JOINT_ANGLE_DIFF):
                return
        

        if(state & (1 << defs.INITIAL_SETUP)):
            # state &= ~(1 << defs.ARM_CHANGING_POSE) & ~(1 << defs.INITIAL_SETUP)
            # state |= (1 << defs.ENABLE_VELODYME)
            state_publisher.publish(data = -defs.ARM_CHANGING_POSE)
            state_publisher.publish(data = -defs.INITIAL_SETUP)
            state_publisher.publish(data =  defs.ENABLE_VELODYME)
        else:
            # state = state & ~(1 << defs.ARM_CHANGING_POSE)
            state_publisher.publish(data = -defs.ARM_CHANGING_POSE)


        # state_publisher.publish(data = state)

    # if(((~state) & (1 << defs.CLIMB_STAIR)) and np.all(data.joint_variable == 0)):
    #     # state &= ~(1 << defs.CLIMB_STAIR)
    #     state_publisher.publish(data = -defs.CLIMB_STAIR)

            

def hokuyo_distance_callback(data):
    global x
    global y
    global z
    global state
    global fire_found
    global torque_value
    global state_publisher
    global hokuyo_distance
    global leaving_fire_cont
    hokuyo_distance = data.data[0]

    x_temp = 0
    y_temp = 0
    x_move = 0
    y_move = 0
    if(state & 1 << defs.ROBOT_CLOCKWISE):
        x_temp = x
        y_temp = y
    else:
        x_temp = y
        y_temp = x

    if(state & (1 << defs.FOUND_FIRE_RIGHT) or state & (1 << defs.FOUND_FIRE_TOUCH) or state & (1 << defs.FOUND_FIRE_FRONT)):
        x_dist = data.data[1]
        if(x_dist > 10):
            x_dist = 10
        elif(x_dist < -10):
            x_dist = -10

        
        #print("getting distance by hokuyo. x_dist: " + str(x_dist) + "  plus displacement: " + str(x_dist + _HOKUYO_DISPLACEMENT) + "   x: " + str(x))
        x_move += (int)(x_dist) + _HOKUYO_DISPLACEMENT


    print ("\nhokuyo distance: " + str (hokuyo_distance))


    if(not state & (1 << defs.ENABLE_VELODYME | 1 << defs.ROBOT_ROTATION) and state & (1 << defs.FOUND_FIRE_RIGHT)):
        # state &= ~(1 << defs.FOUND_FIRE_RIGHT)
        # state |= 1 << defs.FOUND_FIRE_FRONT
        # state_publisher.publish(data = state)
        state_publisher.publish(data = -defs.FOUND_FIRE_RIGHT)
        state_publisher.publish(data =  defs.FOUND_FIRE_FRONT)


    elif (state & (1 << defs.SETTING_UP_HOKUYO)):
        if(hokuyo_distance > _TOUCH_FIRE_FRONT_DISTANCE):
            # state &= ~(1 << defs.SETTING_UP_HOKUYO) 
            # state |= (1 << defs.ENABLE_VELODYME) | (1 << defs.FOUND_FIRE_RIGHT) | (1 << defs.HOKUYO_READING)
            state_publisher.publish(data = -defs.SETTING_UP_HOKUYO)
            state_publisher.publish(data =  defs.ENABLE_VELODYME)
            state_publisher.publish(data =  defs.FOUND_FIRE_RIGHT)
            state_publisher.publish(data =  defs.HOKUYO_READING)
        else:
            # state &= ~(1 << defs.SETTING_UP_HOKUYO | 1 << defs.ENABLE_VELODYME) 
            # state |=  (1 << defs.FOUND_FIRE_TOUCH) | (1 << defs.HOKUYO_READING)
            state_publisher.publish(data = -defs.SETTING_UP_HOKUYO)
            state_publisher.publish(data = -defs.ENABLE_VELODYME)
            state_publisher.publish(data =  defs.FOUND_FIRE_TOUCH)
            state_publisher.publish(data =  defs.HOKUYO_READING)
        # state_publisher.publish(data = state)

    elif (state & (1 << defs.FOUND_FIRE_FRONT) and hokuyo_distance > _TOUCH_FIRE_FRONT_DISTANCE):
        rosi_speed_publisher.publish(data = [2,2,2,2])
    
    elif(hokuyo_distance <= _TOUCH_FIRE_FRONT_DISTANCE and state & (1 << defs.FOUND_FIRE_FRONT)):
        rosi_speed_publisher.publish(data = [0,0,0,0])

        # state &= ~(1 << defs.FOUND_FIRE_FRONT)
        # state |= (1 << defs.ROBOT_ROTATION) | (1 << defs.FOUND_FIRE_TOUCH)
        # state_publisher.publish(data = state)
        state_publisher.publish(data = -defs.FOUND_FIRE_FRONT)
        state_publisher.publish(data =  defs.ROBOT_ROTATION)
        state_publisher.publish(data =  defs.FOUND_FIRE_TOUCH)
    
    elif(state & (1 << defs.FOUND_FIRE_TOUCH) and not state & (1 << defs.ROBOT_ROTATION)):
        if(state & (1 << defs.ROBOT_CLOCKWISE) and (x_temp > 400 or x_temp < 300)):
            if (x_temp > 400):
                rosi_speed_publisher.publish(data = ([0.5,0.5,0.5,0.5]))
            
            elif (x_temp < 300):
                rosi_speed_publisher.publish(data = ([-0.5,-0.5,-0.5,-0.5]))

        elif(not (state & (1 << defs.ROBOT_CLOCKWISE)) and (x_temp < -50 or x_temp > 0)):
            if (x_temp < -50):
                rosi_speed_publisher.publish(data = ([0.5,0.5,0.5,0.5]))
            
            elif (x_temp > 0):
                rosi_speed_publisher.publish(data = ([-0.5,-0.5,-0.5,-0.5]))
        
        elif (torque_value < 0.15 and not state & (1 << defs.LEAVING_FIRE)):
            y_move -= 4
            if (z > _FIRE_TOUCH_Z_VALUE):
                z -= 2
            rosi_speed_publisher.publish(data = [0,0,0,0])

        elif(torque_value >= 0.1 and not state & (1 << defs.LEAVING_FIRE)):
            leaving_fire_cont = 0
            # state |=  1 << defs.LEAVING_FIRE
            # state_publisher.publish(data = state)
            state_publisher.publish(data = defs.LEAVING_FIRE)
            fire_found = False
        
        elif(state & (1 << defs.LEAVING_FIRE) and hokuyo_distance < 25):
            if hokuyo_distance > 15:
                y_move += 10
            else:
                y_move += 5


        elif(state & (1 << defs.LEAVING_FIRE) and hokuyo_distance >= 25):
            if(narrow_path):
                x = _NARROW_PATH_X_VALUE
                y = _NARROW_PATH_Y_VALUE
                z = _NARROW_PATH_Z_VALUE
            else:
                x = _FIRE_NOT_FOUND_X_VALUE
                y = _FIRE_NOT_FOUND_Y_VALUE
                z = _FIRE_NOT_FOUND_Z_VALUE

            pos = cinematicaInversa()
            arm_publisher.publish(joint_variable = pos)

            # state &= ~(1 << defs.FOUND_FIRE_TOUCH | 1 << defs.HOKUYO_READING)
            # state |= 1 << defs.ENABLE_VELODYME  | 1 << defs.ARM_CHANGING_POSE
            # state_publisher.publish(data = state)
            state_publisher.publish(data = -defs.FOUND_FIRE_TOUCH)
            state_publisher.publish(data = -defs.HOKUYO_READING)
            state_publisher.publish(data =  defs.ENABLE_VELODYME)
            state_publisher.publish(data =  defs.ARM_CHANGING_POSE)
            return
    
    if(state & 1 << defs.ROBOT_CLOCKWISE):
        x = x_temp + x_move
        y = y_temp + y_move
    else:
        x = y_temp - y_move
        y = x_temp + x_move

    pos = cinematicaInversa()
    arm_publisher.publish(joint_variable = pos)

    #print("xtemp: " + str(x_temp) + ", ytemp: " + str(y_temp) + ",  xmove: " + str(x_move) + ",  ymove: " + str(y_move))




def state_callback(data):
    global x
    global y
    global z
    global state
    global narrow_path
    global arm_publisher
    global state_publisher
    global rosi_speed_publisher
    global robot_last_orientation
    state = data.data
    #print ("state: " + str(state))

    if (state & (1 << defs.INITIAL_SETUP)):
        global arm_publisher
        global state_publisher
        pos = cinematicaInversa()
        arm_publisher.publish(joint_variable = pos)

        speed = [0,0,0,0]
        rosi_speed_publisher.publish(data = speed)
    

    #if the arm is it in
    if ((state & (1 << defs.NARROW_PATH | 1 << defs.IN_STAIR)) and not narrow_path):
        narrow_path = True
        x = _NARROW_PATH_X_VALUE
        y = _NARROW_PATH_Y_VALUE
        z = _NARROW_PATH_Z_VALUE
        pos = cinematicaInversa()
        arm_publisher.publish(joint_variable = pos)

        # state |= 1 << defs.ARM_CHANGING_POSE
        # state_publisher.publish(data = state)
        state_publisher.publish(data = defs.ARM_CHANGING_POSE)

    elif(not state & (1 << defs.NARROW_PATH | 1 << defs.IN_STAIR) and narrow_path):
        narrow_path = False
        x = _FIRE_NOT_FOUND_X_VALUE
        y = _FIRE_NOT_FOUND_Y_VALUE
        z = _FIRE_NOT_FOUND_Z_VALUE
        pos = cinematicaInversa()
        arm_publisher.publish(joint_variable = pos)

    if (state & (1 << defs.CLIMB_STAIR)):
        arm_publisher.publish(joint_variable = [0,0,0,0,0,0])

    if(not state & (1 << robot_last_orientation)):
        if(robot_last_orientation == defs.ROBOT_CLOCKWISE):
            robot_last_orientation = defs.ROBOT_ANTICLOCKWISE
        else:
            robot_last_orientation = defs.ROBOT_CLOCKWISE

        x = _FIRE_NOT_FOUND_X_VALUE
        y = _FIRE_NOT_FOUND_Y_VALUE
        z = _FIRE_NOT_FOUND_Z_VALUE
        pos = cinematicaInversa()
        arm_publisher.publish(joint_variable = pos)
        state_publisher.publish(data = defs.ARM_CHANGING_POSE)
        rosi_speed_publisher.publish(data = [0,0,0,0,0,0])



def torque_callback(data):
    global torque_value
    torque_value = data.twist.linear.z


#main
def listener():
    rospy.init_node('arm', anonymous=True)

    global x
    global y
    global z

    x = _FIRE_NOT_FOUND_X_VALUE
    y = _FIRE_NOT_FOUND_Y_VALUE
    z = _FIRE_NOT_FOUND_Z_VALUE
    pos = cinematicaInversa()
    
    #receives the current position of the arm
    #used when it's necessa
    rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, arm_current_position)

    rospy.Subscriber("/pra_vale/imu", Float32MultiArray, arm_imu)
    rospy.Subscriber('/pra_vale/arm_tilt', Float32, arm_tilt)
    rospy.Subscriber("/pra_vale/estados", Int32, state_callback)
    rospy.Subscriber('/pra_vale/arm_move', Int32MultiArray, arm_move)
    rospy.Subscriber("/pra_vale/hokuyo_distance", Int32MultiArray, hokuyo_distance_callback)
    rospy.Subscriber("/ur5/forceTorqueSensorOutput", TwistStamped, torque_callback)

    rospy.spin()


#main
print("arm launched")

# for cont1 in range(2000):
#     x = 320
#     y = 1000 -cont1
#     z = 925

#     pos = cinematicaInversa()



listener()


