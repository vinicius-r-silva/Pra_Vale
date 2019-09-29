#!/usr/bin/env python

import rospy
from math import pi
from math import pow
from math import sqrt
from math import atan2
from math import acos
from math import asin
#from std_msgs.msg import Int8
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
#from std_msgs.msg import Int32MultiArray
from rosi_defy.msg import ManipulatorJoints


#-------------------GLOBAL VARIABLES----------------#    
#publish joint values to ur5 arm
arm_publisher = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)

#enable hokuyo
#hokuyo_publisher = rospy.Publisher('/pra_vale/enable_hokuyo', Int8, queue_size=10)
#hokuyo_enabled = 0

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
z = 600
last_x = x
last_y = y
last_z = z
tilt_x = 0
tilt_y = 0
tilt_z = 0

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

    print("x: " + str(x) + "  y: " + str(y) + "  z: " + str(z))

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

        t1 = pi/2 - (t1_const + s2)
        t2 = pi - t2_const
        t3 = -(t3_const + s1)

        #joint angles
        t0 = -pi/2 - acos(probe_lenght/base_dist) + atan2(y_const, x_const)
        t1 = -t1
        t2 = -t2
        t3 = -t3
        t4 = -t0 - pi
        t5 = tilt_y

        #check if t0 and t4 angles exceeds 2*pi
        if(t0 > 2*pi):
            t0 -= 2*pi
        elif(t0 < -2*pi):
            t0 += 2*pi
        # if(t4 > 2*pi):
        #     t4 -= 2*pi
        # elif(t4 < -2*pi):
        #     t4 += 2*pi

        t0 += tilt_z
        t1 += tilt_x

        #update last postion
        last_x = x
        last_y = y
        last_z = z

        #print the joint angles 
        print(str(t0) + ", " + str(t1) + ", " + str(t2) + ", " + str(t3) + ", " + str(t4) + ", " + str(t5))
        return [t0, t1, t2, t3, t4, t5]
    
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


#receive incremental x,y,z position and sets in the simulation
def arm_move(data):
    global x
    global y
    global z
    global arm_publisher
    #print(data) #debug

    x += data.data[0]
    y += data.data[1]
    z += data.data[2]
    
    #get the joints angles
    pos = cinematicaInversa()
    #publish joints angles
    arm_publisher.publish(joint_variable = pos)


#callback from the IMU sensor, make the ur5 arm follow the track automatically
def arm_tilt(data):
    global tilt_x
    global tilt_y
    global tilt_z
    global arm_publisher

    #get imu quartenions
    qx = data.orientation.x
    qy = data.orientation.y
    qz = data.orientation.z
    qw = data.orientation.w
    
    #transform quartenions to euler angles
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    tilt_x = atan2(t0, t1)  #x euler angle

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    tilt_y = asin(t2)  #y euler angle

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    tilt_z = atan2(t4, t3) + pi/2  #z euler angle
    
    #get the joints angles
    pos = cinematicaInversa()
    #publish joints angles
    arm_publisher.publish(joint_variable = pos)


#callback from the ur5 camera when it detects fire
#the ur5 camera publishes the x distance between the arm and the fire
def cam_found_fire(data):
    x_data = data.data
    global x
    x += x_data


def listener():
    rospy.init_node('arm', anonymous=True)
    
    #rospy.Subscriber('/pra_vale/arm_pos', Int32MultiArray, arm_pos)
    #rospy.Subscriber('/pra_vale/arm_move', Int32MultiArray, arm_move)
    rospy.Subscriber('/pra_vale/cam_found_fire', Int32, cam_found_fire)
    rospy.Subscriber("/sensor/imu", Imu, arm_tilt)

    rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()


