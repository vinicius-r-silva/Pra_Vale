import rospy
from math import pi as _pi
from math import pow as _pow
from math import sqrt as _sqrt
from math import atan2 as _atan2
from math import acos as _acos
from math import asin as _asin
from math import sin as _sin
from math import cos as _cos
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import ManipulatorJoints     
import defines as defs

#-----------------------CONSTS----------------------#   
#consts for the initial pose for each case
FIRE_TOUCH_Z_VALUE =  410
FIRE_TOUCH_Z_VALUE_IN_STAIRS =  770

FIRE_FOUND_X_VALUE =  320
FIRE_FOUND_Y_VALUE =  -70
FIRE_FOUND_Z_VALUE =  840
NARROW_PATH_X_VALUE =  400
NARROW_PATH_Y_VALUE =  80
NARROW_PATH_Z_VALUE =  900
FIRE_NOT_FOUND_X_VALUE =  320
FIRE_NOT_FOUND_Y_VALUE =  -70
FIRE_NOT_FOUND_Z_VALUE =  925
IN_STAIRS_X_VALUE =  346
IN_STAIRS_Y_VALUE =  29
IN_STAIRS_Z_VALUE =  927

IMU_TILT_ERROR = 0.1

IN_STAIRS_DISPLACEMENT = -_pi/7

#-------------------GLOBAL VARIABLES----------------#    
#arm sizes
_Probe_lenght = 71 #279
_D0 = 275 #367
_L1 = 425 #425.1
_L2 = 392 #392.1
_L3 = 111  #126

#consts
_Probe_lenght_pow = _Probe_lenght*_Probe_lenght
_L1_pow = _L1*_L1
_L2_pow = _L2*_L2

#initial pose
x = FIRE_NOT_FOUND_X_VALUE
y = FIRE_NOT_FOUND_Y_VALUE
z = FIRE_NOT_FOUND_Z_VALUE
tilt_x = 0
tilt_y = 0
tilt_z = 0

_last_x = x
_last_y = y
_last_z = z

#current desired arm joint angles
joint_angles = [0,0,0,0,0,0]

#used to check wether the y axis tilt is going to be detect by the imu or by the ur5 camera
get_tilt_y_from_imu = False

camera_tilt = 0.0

def cinematicaInversa(state):
    global x
    global y
    global z

    global tilt_x
    global tilt_y
    global tilt_z

    global _last_x
    global _last_y
    global _last_z

    global joint_angles

    #print("x: " + str(x) + "  y: " + str(y) + "  z: " + str(z))

    if(x < 0):
        print("erro: posicao (" + str([x,y,z]) + ") fora do alcance do braco robotico")

        #return last valid position
        x = _last_x
        y = _last_y
        z = _last_z
        return joint_angles


    #a error is given when the desired position is invalid
    try:
    #main calculations
        x_const = x - _D0
        x_const_pow = x_const*x_const

        y_const = y
        y_const_pow = y_const*y_const

        base_dist = _sqrt(x_const_pow + y_const_pow)
        d = _sqrt(x_const_pow + y_const_pow - _Probe_lenght_pow)

        d_const = d
        z_const = z - _L3

        l_const_pow = d_const*d_const + z_const*z_const
        l_const = _sqrt(l_const_pow)

        t1_const = _acos((_L1_pow - _L2_pow + l_const_pow) / (2*_L1*l_const))
        t2_const = _acos((_L1_pow + _L2_pow - l_const_pow) / (2*_L1*_L2))
        t3_const = _pi - t1_const - t2_const

        s1 = _acos(z_const/l_const)
        s2 = _acos(d_const/l_const)

        #joint angles
        if(state & (1 << defs.ROBOT_CLOCKWISE)):
            joint_angles[0] = -_pi/2 - _acos(_Probe_lenght/base_dist) + _atan2(y_const, x_const) #y = 71, x_const <= 0, x <= D0
            joint_angles[1] = -_pi/2 + (t1_const + s2)
            joint_angles[2] = -_pi + t2_const
            joint_angles[3] = t3_const + s1
            joint_angles[4] = -joint_angles[0] - _pi
        else:
            joint_angles[0] = _acos(_Probe_lenght/base_dist) + _atan2(y_const, x_const)
            joint_angles[1] = _pi/2 - (t1_const + s2)
            joint_angles[2] = _pi - t2_const
            joint_angles[3] = -t3_const - s1
            joint_angles[4] = -joint_angles[0]

        if(state & (1 << defs.IN_STAIR) and not state & (1 << defs.HOKUYO_READING)):
            joint_angles[3] += IN_STAIRS_DISPLACEMENT



        #check if t0 and t4 angles exceeds 2*_pi
        if(joint_angles[0] > 2*_pi):
            joint_angles[0] -= 2*_pi
        elif(joint_angles[0] < -2*_pi):
            joint_angles[0] += 2*_pi
        # if(joint_angles[4] > 2*_pi):
        #     joint_angles[4] -= 2*_pi
        # elif(joint_angles[4] < -2*_pi):
        #     joint_angles[4] += 2*_pi


        joint_angles[0] += tilt_z
        joint_angles[1] += tilt_x
        joint_angles[5] = tilt_y

        #configure tilt
        if((state & (1 << defs.ROBOT_ON_THE_LEFT)) and (state & (1 << defs.ROBOT_ANTICLOCKWISE))):
            if(joint_angles[0] < 0):
                joint_angles[0] += 2*_pi 
            
            joint_angles[0] -= _pi
            # print("################## 1 : joint: " + str(joint_angles[0]) + ",    tilt: " + str(tilt_z))
            
        elif((not (state & (1 << defs.ROBOT_ON_THE_LEFT))) and (state & (1 << defs.ROBOT_CLOCKWISE))):
            joint_angles[0] += _pi
            # print("################## 2 : joint: " + str(joint_angles[0]) + ",    tilt: " + str(tilt_z))


        # print the joint angles and the x,y,z position of the arm (debuging)
        #print(joint_angles)
        #print("x: " + str(x) + ",  y: " + str(y) + ",  z: " + str(z))      
        _last_x = x
        _last_y = y
        _last_z = z
        return joint_angles
    
    except: #Exception as error: #(for debug, uncomment this part of code)
        #print(error)
        print("erro: posicao (" , str([x,y,z]), ") fora do alcance do braco robotico")
        
        #return last valid position
        x = _last_x
        y = _last_y
        z = _last_z
        return joint_angles