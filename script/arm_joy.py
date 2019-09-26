#!/usr/bin/env python3

#sudo pip3 install rospkg catkin_pkg

import rospy
import readchar
from rosi_defy.msg import ManipulatorJoints
from sensor_msgs.msg import Joy
from math import pi
from math import pow
from math import sqrt
from math import atan2
from math import acos

#-------------------CONST----------------#
probe_lenght = 150 #279
d0 = 275 #367
l1 = 425 #425.1
l2 = 392 #392.1
l3 = 126  #40.9

probe_lenght_pow = probe_lenght*probe_lenght
l1_pow = l1*l1
l2_pow = l2*l2

t4 = -pi/2
t5 = 0

x = 275
y = 152
z = 400

last_x = 275
last_y = 152
last_z = 942

def cinematicaInversa():
    global x
    global y
    global z

    global last_x
    global last_y
    global last_z

    print("x: " + str(x) + "  y: " + str(y) + "  z: " + str(z))

    try:
        x_pow = x*x
        y_pow = y*y
        #z_pow = z*z

        base_dist = sqrt(x_pow + y_pow)
        d = sqrt(x_pow + y_pow - probe_lenght_pow)
        t0 = atan2(y, x) + acos(probe_lenght/base_dist)

        d_const = d - d0
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

        last_x = x
        last_y = y
        last_z = z
        return [t0, t1, t2, t3, t4, t5]
    
    except:
        print("erro: posicao fora do alcance do braco robotico")
        x = last_x
        y = last_y
        z = last_z
        return cinematicaInversa()


def listener():
    rospy.init_node('listener', anonymous=True)

    step = 1
    global x
    global y
    global z

    #rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, callback)
    #rospy.Subscriber('/joy', Joy, callback_Joy)
    
    pub = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)


    #node_sleep_rate = rospy.Rate(10)
    while not rospy.is_shutdown():    
        #pos = cinematicaInversa()
        #pub.publish(joint_variable = pos)
        key = readchar.readkey()
        key = key.lower()
        if key in ('wsadqe'):
            print ('Key:' + key)
            if key == 'w':
                x += step
            if key == 's':
                x -= step

            if key == 'a':
                y += step
            if key == 'd':
                y -= step
                
            if key == 'e':
                z += step
            if key == 'q':
                z -= step

        if key == '.':
            print ("finished")
            break
        
        pos = cinematicaInversa()
        pub.publish(joint_variable = pos)
        #node_sleep_rate.sleep()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")
listener()


