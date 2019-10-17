#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from math import atan2
from math import asin
from math import sin
from math import cos
from math import pi

angles = [0,0,0]

#get ROSI imu orientatio data
#change it from quarternios to euler angles
def imu_callback(data):
    global angles

    #get imu quartenions
    qx = data.orientation.x
    qy = data.orientation.y
    qz = data.orientation.z
    qw = data.orientation.w
    
    #transform quartenions to euler angles
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    x = atan2(t0, t1)  #x euler angle

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    y = asin(t2)  #y euler angle

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    z = atan2(t4, t3) + pi/2  #z euler angle

    #rotates the x and y angles to match the track and not the robot
    x_temp = cos(z)*x - sin(z)*y
    y_temp = sin(z)*x + cos(z)*y

    x = x_temp
    y = y_temp

    angles[0] = x
    angles[1] = y
    angles[2] = z 

    #print(angles) #debug



#main
if __name__ == '__main__':
    rospy.init_node('imu_handler', anonymous=True)
    rospy.Subscriber("/sensor/imu", Imu, imu_callback)
    pub = rospy.Publisher('/pra_vale/imu', Float32MultiArray, queue_size=1)

    print("imu_handler launched")

    node_sleep_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(data = angles)
        node_sleep_rate.sleep()