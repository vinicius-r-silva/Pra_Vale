#!/usr/bin/env python

import rospy
from rosi_defy.msg import RosiMovement
from std_msgs.msg import Float32MultiArray
from rosi_defy.msg import RosiMovementArray

speeds = [0,0,0, 0]
pub = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)


def speed_callback(data):
    global pub
    global speeds
    speeds = data.data
    #print(speeds)

    traction_command_list = RosiMovementArray()

    traction_command = RosiMovement()
    traction_command.nodeID = 1
    traction_command.joint_var = speeds[0]
    traction_command_list.movement_array.append(traction_command)

    traction_command = RosiMovement()
    traction_command.nodeID = 2
    traction_command.joint_var = speeds[1]
    traction_command_list.movement_array.append(traction_command)

    traction_command = RosiMovement()
    traction_command.nodeID = 3
    traction_command.joint_var = speeds[2]
    traction_command_list.movement_array.append(traction_command)

    traction_command = RosiMovement()
    traction_command.nodeID = 4
    traction_command.joint_var = speeds[3]
    traction_command_list.movement_array.append(traction_command)

    pub.publish(traction_command_list)



#main
if __name__ == '__main__':
    rospy.init_node('rosi_speed_handler', anonymous=True)
    rospy.Subscriber("/pra_vale/rosi_speed", Float32MultiArray, speed_callback)

    print("rosi_speed launched")

    rospy.spin()
