#!/usr/bin/env python

#sudo pip3 install rospkg catkin_pkg

import rospy
from rosi_defy.msg import ManipulatorJoints
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from math import pi
from math import pow
from math import sqrt
from math import atan2
from math import acos
from math import asin

#-------------------CONST----------------#    
pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)

probe_lenght = 71 #279
d0 = 275 #367
l1 = 425 #425.1
l2 = 392 #392.1
l3 = 111  #126

probe_lenght_pow = probe_lenght*probe_lenght
l1_pow = l1*l1
l2_pow = l2*l2


x = 400
y = 400
z = 700

last_x = x
last_y = y
last_z = z

tilt_x = 0
tilt_y = 0
tilt_z = 0

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

    try:
        x_pow = x*x
        y_pow = y*y
        #z_pow = z*z

        x_const = x - d0
        x_const_pow = x_const*x_const

        y_const = y
        y_const_pow = y_const*y_const

        base_dist = sqrt(x_const_pow + y_const_pow)
        d = sqrt(x_const_pow + y_const_pow - probe_lenght_pow)

        t0 = -pi/2 - acos(probe_lenght/base_dist) + atan2(y_const, x_const)
        if(t0 > 2*pi):
            t0 -= 2*pi
        elif(t0 < -2*pi):
            t0 += 2*pi

        t4 = -t0 - pi
        # if(t4 > 2*pi):
        #     t4 -= 2*pi
        # elif(t4 < -2*pi):
        #     t4 += 2*pi


        d_const = d
        z_const = z - l3

        l_const_pow = d_const*d_const + z_const*z_const
        l_const = sqrt(l_const_pow)

        t1_const = acos((l1_pow - l2_pow + l_const_pow) / (2*l1*l_const))
        t2_const = acos((l1_pow + l2_pow - l_const_pow) / (2*l1*l2))
        t3_const = pi - t1_const - t2_const

        s1 = acos(z_const/l_const)
        s2 = acos(d_const/l_const)
        #s3 = pi/2 - s2

        t1 = pi/2 - (t1_const + s2)
        t2 = pi - t2_const
        t3 = -(t3_const + s1)

        t1 = -t1
        t2 = -t2
        t3 = -t3

        t0 += tilt_z
        t1 += tilt_x
        t5 = tilt_y

        last_x = x
        last_y = y
        last_z = z
        print(str(t0) + ", " + str(t1) + ", " + str(t2) + ", " + str(t3) + ", " + str(t4) + ", " + str(t5))
        return [t0, t1, t2, t3, t4, t5]
    
    except:
        print("erro: posicao fora do alcance do braco robotico")
        x = last_x
        y = last_y
        z = last_z
        return cinematicaInversa()

def arm_pos(data):
    global x
    global y
    global z
    global pub

    print(data)

    x = data.data[0]
    y = data.data[1]
    z = data.data[2]
    
    pos = cinematicaInversa()
    pub.publish(joint_variable = pos)

def arm_move(data):
    global x
    global y
    global z
    global pub

    print(data)

    x += data.data[0]
    y += data.data[1]
    z += data.data[2]
    
    pos = cinematicaInversa()
    pub.publish(joint_variable = pos)

def arm_tilt(data):
    global tilt_x
    global tilt_y
    global tilt_z

    qx = data.orientation.x
    qy = data.orientation.y
    qz = data.orientation.z
    qw = data.orientation.w
    
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    tilt_x = atan2(t0, t1)

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    tilt_y = asin(t2)

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    tilt_z = atan2(t4, t3) + pi/2

    pos = cinematicaInversa()
    pub.publish(joint_variable = pos)


def listener():
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber('/pra_vale/arm_pos', Int32MultiArray, arm_pos)
    rospy.Subscriber('/pra_vale/arm_move', Int32MultiArray, arm_move)
    #rospy.Subscriber('/pra_vale/arm_tilt',Int32MultiArray , arm_tilt)
    rospy.Subscriber("/sensor/imu", Imu, arm_tilt)

    rospy.spin()

    # node_sleep_rate = rospy.Rate(10)
    # while not rospy.is_shutdown():    
    #     #pos = cinematicaInversa()
    #     #pub.publish(joint_variable = pos)
    #     key = readchar.readkey()
    #     key = key.lower()
    #     if key in ('wsadqe'):
    #         print ('Key:' + key)
    #         if key == 'w':
    #             x += step
    #         if key == 's':
    #             x -= step

    #         if key == 'a':
    #             y += step
    #         if key == 'd':
    #             y -= step
                
    #         if key == 'e':
    #             z += step
    #         if key == 'q':
    #             z -= step

    #     if key == '.':
    #         print ("finished")
    #         break
        
    #     pos = cinematicaInversa()
    #     pub.publish(joint_variable = pos)
    #     node_sleep_rate.sleep()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()


