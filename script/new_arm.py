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
FIRE_FOUND_X_VALUE =  320
FIRE_FOUND_Y_VALUE =  -70
FIRE_FOUND_Z_VALUE =  800
NARROW_PATH_X_VALUE =  400
NARROW_PATH_Y_VALUE =  80
NARROW_PATH_Z_VALUE =  900
FIRE_NOT_FOUND_X_VALUE =  320
FIRE_NOT_FOUND_Y_VALUE =  -70
FIRE_NOT_FOUND_Z_VALUE =  925

_IMU_TILT_ERROR = 0.1
#-------------------GLOBAL VARIABLES----------------#    

#publish robot state changes to others ROS packages
state_publisher = rospy.Publisher('/pra_vale/def_state', Int32, queue_size=15)

#publishes ROSI speed
rosi_speed_publisher = rospy.Publisher("/pra_vale/rosi_speed", Float32MultiArray, queue_size = 1)

#publish joint values to ur5 arm
arm_publisher = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)

state = defs.NOTHING

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
_tilt_x = 0
_tilt_y = 0
_tilt_z = 0

#current desired arm joint angles
_joint_angles = [0,0,0,0,0,0]

#used to check wether the y axis tilt is going to be detect by the imu or by the ur5 camera
_get_tilt_y_from_imu = False

_camera_tilt = 0.0

def cinematicaInversa():
    global x
    global y
    global z

    global _tilt_x
    global _tilt_y
    global _tilt_z

    global _joint_angles

    #print("x: " + str(x) + "  y: " + str(y) + "  z: " + str(z))

    if(x < 0):
        print("erro: posicao fora do alcance do braco robotico")

        #return last valid position
        return _joint_angles


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
            _joint_angles[0] = -_pi/2 - _acos(_Probe_lenght/base_dist) + atan2(y_const, x_const)
            _joint_angles[1] = -_pi/2 + (t1_const + s2)
            _joint_angles[2] = -_pi + t2_const
            _joint_angles[3] = t3_const + s1
            _joint_angles[4] = -_joint_angles[0] - _pi
        else:
            _joint_angles[0] = _acos(_Probe_lenght/base_dist) + atan2(y_const, x_const)
            _joint_angles[1] = _pi/2 - (t1_const + s2)
            _joint_angles[2] = _pi - t2_const
            _joint_angles[3] = -t3_const - s1
            _joint_angles[4] = -_joint_angles[0]


        #check if t0 and t4 angles exceeds 2*_pi
        if(_joint_angles[0] > 2*_pi):
            _joint_angles[0] -= 2*_pi
        elif(_joint_angles[0] < -2*_pi):
            _joint_angles[0] += 2*_pi
        # if(_joint_angles[4] > 2*_pi):
        #     _joint_angles[4] -= 2*_pi
        # elif(_joint_angles[4] < -2*_pi):
        #     _joint_angles[4] += 2*_pi

        #configure tilt
        _joint_angles[0] += _tilt_z
        _joint_angles[1] += _tilt_x
        _joint_angles[5] = _tilt_y

        # print the joint angles for debuging
        # print(_joint_angles)
        #print("x: " + str(x) + ",  y: " + str(y) + ",  z: " + str(z))            
        return _joint_angles
    
    except:
        print("erro: posicao fora do alcance do braco robotico")
        
        #return last valid position
        return _joint_angles




#get arm joint angles
#used when it's necessary to the robot to wait the arm reachs it's designed position
def arm_current_position(data):
    global state
    global _joint_angles
    global rosi_speed_publisher
    global state_publisher
         
    if(state & (1 << defs.ARM_CHANGING_POSE)):
        #if all joint angles are in the correct places, remove the _ARM_CHANGING_POSE from the states
        for x in range(len(_joint_angles)):
            if(abs(_joint_angles[x] - data.joint_variable[x]) > defs.MAX_JOINT_ANGLE_DIFF):
                return

        if(state & (1 << defs.INITIAL_SETUP)):
            state_publisher.publish(data = -defs.ARM_CHANGING_POSE)
            state_publisher.publish(data = -defs.INITIAL_SETUP)
            state_publisher.publish(data =  defs.ENABLE_VELODYME)
        else:
            state_publisher.publish(data = -defs.ARM_CHANGING_POSE)


#callback from the IMU sensor, make the ur5 arm follow the track automatically
def arm_imu(data):
    global state
    if(state & (1 << defs.ARM_CHANGING_POSE | 1 << defs.CLIMB_STAIR)):
        return
  
    global _tilt_x
    global _tilt_y
    global _tilt_z
    global arm_publisher
    global _get_tilt_y_from_imu

    temp_y = 0.0
    angles = data.data
    if(state & (1 << defs.ROBOT_CLOCKWISE)):
        _tilt_x = angles[0]
        temp_y = -angles[1] #+ 0.065
        _tilt_z = angles[2]
    else:
        _tilt_x = -angles[0]
        temp_y = -angles[1] - 0.065
        _tilt_z = angles[2]
        if(_tilt_z > _pi):
            _tilt_z = _tilt_z - 2*_pi
    
    if _get_tilt_y_from_imu:
        _tilt_y = temp_y
    else:
        _tilt_y = _camera_tilt

        if _tilt_y > temp_y + _IMU_TILT_ERROR:
            _tilt_y = temp_y + _IMU_TILT_ERROR

        elif _tilt_y < temp_y - _IMU_TILT_ERROR:
            _tilt_y = temp_y - _IMU_TILT_ERROR
    
        
    pos = cinematicaInversa()
    #publish joints angles
    arm_publisher.publish(joint_variable = pos)



#callback from the ur5 camera, make the ur5 arm follow the track automatically
def arm_tilt(data):
    global state
    global _camera_tilt 
    global _get_tilt_y_from_imu
    if((state & (1 << defs.ARM_CHANGING_POSE | 1 << defs.CLIMB_STAIR))): #or (state & 1 << defs.ROBOT_ROTATION)):
        return

    if(data.data == -1 or state & (1 << defs.FOUND_FIRE_FRONT) or state & (1 << defs.FOUND_FIRE_TOUCH)):
        _get_tilt_y_from_imu = True
    else:
        if(_get_tilt_y_from_imu == True):
            _get_tilt_y_from_imu = False
            _camera_tilt = 0.0
        
        _camera_tilt += data.data #- 0.1