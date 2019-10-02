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
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints  


#-----------------------CONSTS----------------------#   
#const received form the ur5_Cam when no fire is detected
_FIRE_NOT_FOUND = 1000

#const for the initial pose for each case
_FIRE_FOUND_X_VALUE =  400
_FIRE_FOUND_Y_VALUE = -150
_FIRE_FOUND_Z_VALUE =  600
_FIRE_NOT_FOUND_X_VALUE =  400
_FIRE_NOT_FOUND_Y_VALUE = -150
_FIRE_NOT_FOUND_Z_VALUE =  850

#const for minimum angle difference betwwen the current angle and the desired angle
# used to calculates if a joint of the arm is in place
_MAX_JOINT_ANGLE_DIFF = 0.1

#enable consts
#used on the state topic
_NOTHING = 0
_ENABLE_VELODYME = 1
_ARM_CHANGING_POSE = 2 #makes the arm ignore all icoming requests to position changes
_FOLLOW_TRACK = 3
_FOUND_FIRE_FRONT = 4
_FOUND_FIRE_RIGHT = 5
_FOUND_FIRE_TOUCH = 6
_SETTING_UP_HOKUYO = 7
_INITIAL_SETUP     = 8

_HOKUYO_STATE_DISABLE = 0
_HOKUYO_STATE_READING = 1
_HOKUYO_STATE_FOLLOWING = 2
_HOKUYO_START_TO_FOLLOW_FIRE = -2


#-------------------GLOBAL VARIABLES----------------#    
#publish joint values to ur5 arm
arm_publisher = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)

hokuyo_publisher = rospy.Publisher('/pra_vale/hokuyo_state', Int32, queue_size=1)
state_publisher = rospy.Publisher('/pra_vale/set_state', Int32, queue_size=1)
rosi_speed_publisher = rospy.Publisher("/pra_vale/rosi_speed", Float32MultiArray, queue_size = 1)

#const used to check wether the y axis tilt is going to be detect by the imu or by the ur5 camera
get_tilt_y_from_imu = False

#it's true when a fire was detect by the ur5 camera
fire_found = False

#enable hokuyo
#hokuyo_publisher = rospy.Publisher('/pra_vale/enable_hokuyo', Int8, queue_size=10)
#hokuyo_enabled = 0
hokuyo_distance = -1

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
x = 400
y = -150
z = 850
last_x = x
last_y = y
last_z = z
tilt_x = 0
tilt_y = 0
tilt_z = 0

#conts wich show wich state the system is currently, see at the states defines
state = _NOTHING

#current desired arm joint angles
joint_angles = [0,0,0,0,0,0]

#it makes the arm ignores all requests to position changes
#used when a fire is detect and it's necessary rotate the robot
#goes to false when the robot rotates 90 degrees
wait_robot_rotation = False

desired_robot_roatation = 0




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

    global joint_angles

    print("x: " + str(x) + "  y: " + str(y) + "  z: " + str(z))

    if(x < 0):
        print("erro: posicao fora do alcance do braco robotico")

        #change to the last valid position
        x = last_x
        y = last_y
        z = last_z
        #recalculates joint angles to the last valid position
        return cinematicaInversa()


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
        joint_angles[0] = -pi/2 - acos(probe_lenght/base_dist) + atan2(y_const, x_const)
        joint_angles[1] = -pi/2 + (t1_const + s2)
        joint_angles[2] = -pi + t2_const
        joint_angles[3] = t3_const + s1
        joint_angles[4] = -joint_angles[0] - pi

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

        #print the joint angles 
        #print(joint_angles)
        return joint_angles
    
    except:
        print("erro: posicao fora do alcance do braco robotico")

        #change to the last valid position
        x = last_x
        y = last_y
        z = last_z
        #recalculates joint angles to the last valid position
        return cinematicaInversa()


#receive an absolute x,y,z position and sets in the simulation
def arm_pos(data):
    global state
    if(state & 1 << _ARM_CHANGING_POSE):
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
    global state
    global wait_robot_rotation
    global rosi_speed_publisher
    global desired_robot_roatation
    if((state & (1 << _ARM_CHANGING_POSE)) or wait_robot_rotation):
        return
        
    global x
    global y
    global z
    global fire_found
    global arm_publisher
    #print(data) #debug

    
    if(data.data[0] == _FIRE_NOT_FOUND):
        if(fire_found):
            fire_found = False
            state |= 1 << _ARM_CHANGING_POSE
            state_publisher.publish(data = state)

            x = _FIRE_NOT_FOUND_X_VALUE
            y = _FIRE_NOT_FOUND_Y_VALUE
            z = _FIRE_NOT_FOUND_Z_VALUE
        else:
            x += data.data[0]
            y += data.data[1]
            z += data.data[2]
    else:
        if(not fire_found):
            fire_found = True
            state = 1 << _ENABLE_VELODYME | 1 << _SETTING_UP_HOKUYO | 1 << _ARM_CHANGING_POSE
            state_publisher.publish(data = state)

            rosi_speed_publisher.publish(data = [0,0,0,0,0,0])
            x = _FIRE_FOUND_X_VALUE
            y = _FIRE_FOUND_Y_VALUE
            z = _FIRE_FOUND_Z_VALUE
        else:
            if((not state & (1 << _FOUND_FIRE_FRONT)) and (not state & (1 << _FOUND_FIRE_TOUCH))):
                x += data.data[0]
                z += data.data[2]
            
            y += data.data[1]

            if(state & (1 << _SETTING_UP_HOKUYO)):
                if(data.data[0] == 0):
                    #state = 1 << _ENABLE_VELODYME | 1 << _FOUND_FIRE_RIGHT
                    rosi_speed_publisher.publish(data = ([0,0,0,0]))
                    hokuyo_publisher.publish(data = _HOKUYO_STATE_READING)

                    # state = 1 << _NOTHING
                    # state_publisher.publish(data = state)
                    #save hokuyo distance
                elif(data.data[0] < 0):
                    state = 1 << _SETTING_UP_HOKUYO
                    state_publisher.publish(data = state)
                    rosi_speed_publisher.publish(data = ([-0.25,-0.25,-0.25,-0.25]))
                elif(data.data[0] < 10):
                    state = 1 << _SETTING_UP_HOKUYO
                    state_publisher.publish(data = state)
                    rosi_speed_publisher.publish(data = ([0.5,0.5,0.5,0.5]))


            elif(x < 300 and (state & (1 << _FOUND_FIRE_RIGHT))):
                z = _FIRE_FOUND_Z_VALUE
                state = 1 << _FOUND_FIRE_RIGHT
                state_publisher.publish(data = state)
                wait_robot_rotation = True
                desired_robot_roatation = -10

    #get the joints angles
    pos = cinematicaInversa()
    #publish joints angles
    arm_publisher.publish(joint_variable = pos)


#callback from the IMU sensor, make the ur5 arm follow the track automatically
def arm_imu(data):
    global state
    if(state & 1 << _ARM_CHANGING_POSE):
        return
  
    global tilt_x
    global tilt_y
    global tilt_z
    global arm_publisher
    global get_tilt_y_from_imu

    angles = data.data
    tilt_x = angles[0]
    tilt_z = angles[2]

    if get_tilt_y_from_imu:    
        tilt_y = angles[1]
        
    pos = cinematicaInversa()
    #publish joints angles
    arm_publisher.publish(joint_variable = pos)



#callback from the ur5 camera, make the ur5 arm follow the track automatically
def arm_tilt(data):
    global state
    global wait_robot_rotation
    if((state & 1 << _ARM_CHANGING_POSE) or wait_robot_rotation):
        return
        
    global tilt_y 
    global get_tilt_y_from_imu
    # get_tilt_y_from_imu = True
    # return

    if(data.data == -1 or state & (1 << _FOUND_FIRE_FRONT) or state & (1 << _FOUND_FIRE_TOUCH)):
        get_tilt_y_from_imu = True
    else:
        if(get_tilt_y_from_imu == True):
            get_tilt_y_from_imu = False
            tilt_y = 0
            
        tilt_y += data.data



def arm_current_position(data):
    global state
    global joint_angles
    global wait_robot_rotation
    global rosi_speed_publisher
    global desired_robot_roatation
    global state_publisher
         
    if(state & (1 << _ARM_CHANGING_POSE)):
        print ("wait_pose_change")
        for x in range(len(joint_angles)):
            if(abs(joint_angles[x] - data.joint_variable[x]) > _MAX_JOINT_ANGLE_DIFF):
                return
        

        if (wait_robot_rotation):
            desired_robot_roatation = data.joint_variable[0] + (pi/2)
            print("joint: " + str(data.joint_variable[0]) + "  desired: " + str(desired_robot_roatation))

        if(state & (1 << _INITIAL_SETUP)):
            state = (1 << _ENABLE_VELODYME) | (1 << _FOLLOW_TRACK)# | (1 << _ARM_CHANGING_POSE)
        else:
            state = state & ~(1 << _ARM_CHANGING_POSE)
        state_publisher.publish(data = state)

    elif(wait_robot_rotation):
        print ("wait_robot_rotation")
        if(desired_robot_roatation == -10):
            state |= 1 << _ARM_CHANGING_POSE
            state_publisher.publish(data = state)
            return
            

        print("joint: " + str(data.joint_variable[0]) + "  desired: " + str(desired_robot_roatation))
        print (abs(desired_robot_roatation - data.joint_variable[0]))
        
        if(abs(desired_robot_roatation - data.joint_variable[0]) < _MAX_JOINT_ANGLE_DIFF):
            state = 1 << _FOUND_FIRE_FRONT
            state_publisher.publish(data = 1 << _FOUND_FIRE_FRONT)
            wait_robot_rotation = False

            rosi_speed_publisher.publish(data = ([0,0,0,0]))
        else:
            rosi_speed_publisher.publish(data = ([-2,-2, 2, 2]))
            

def hokuyo_distance_callback(data):
    global x
    global state
    global state_publisher
    global hokuyo_distance
    hokuyo_distance = data.data[0]

    if(state & (1 << _FOUND_FIRE_RIGHT) or state & (1 << _FOUND_FIRE_TOUCH)):
        x_dist = data.data[1]
        if(x_dist > 10):
            x_dist = 10
        elif(x_dist < -10):
            x_dist = -10

        x += (int)(x_dist)
        print("getting distance by hokuyo. x_dist: " + str(x_dist))

    print ("hokuyo distance: " + str (hokuyo_distance))

    if (state & (1 << _SETTING_UP_HOKUYO)):
        state = (1 << _ENABLE_VELODYME) | (1 << _FOUND_FIRE_RIGHT)
        state_publisher.publish(data = state)


def state_callback(data):
    global state
    state = data.data
    #print ("state: " + str(state))
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
        global arm_publisher
        global state_publisher
        pos = cinematicaInversa()
        arm_publisher.publish(joint_variable = pos)


#main
def listener():
    rospy.init_node('arm', anonymous=True)

    global x
    global y
    global z

    x = _FIRE_NOT_FOUND_X_VALUE
    y = _FIRE_NOT_FOUND_Y_VALUE
    z = _FIRE_NOT_FOUND_Z_VALUE
    # arm_publisher.publish(joint_angles = pos)

    #state_publisher.publish(data = (1 << _ENABLE_VELODYME) | (1 << _FOLLOW_TRACK))
    
    #receives the current position of the arm
    #used when it's necessa
    rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, arm_current_position)

    rospy.Subscriber("/pra_vale/imu", Float32MultiArray, arm_imu)
    rospy.Subscriber('/pra_vale/arm_tilt', Float32, arm_tilt)
    rospy.Subscriber("/pra_vale/estados", Int32, state_callback)
    rospy.Subscriber('/pra_vale/arm_move', Int32MultiArray, arm_move)
    rospy.Subscriber("/pra_vale/hokuyo_distance", Int32MultiArray, hokuyo_distance_callback)

    rospy.spin()


#main
print("arm launched")
listener()


