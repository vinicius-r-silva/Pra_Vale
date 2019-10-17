#!/usr/bin/env python

import rospy
from rosi_defy.msg import RosiMovement
from std_msgs.msg import Float32MultiArray
from rosi_defy.msg import RosiMovementArray


traction_speeed_pub = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
traction_speeds = [0, 0, 0, 0]
traction_command_list = 0

arm_speeed_pub = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)
arm_speeds = [0, 0, 0, 0]
arm_command_list = 0

#get a list of floats and send it as to VREP as robot wheels speed
def traction_speed_callback(data):
    global traction_command_list
    global traction_speeed_pub
    global traction_speeds
    traction_speeds = data.data
    #print(traction_speeds)

    traction_command_list = RosiMovementArray()

    traction_command = RosiMovement()
    traction_command.nodeID = 1
    traction_command.joint_var = traction_speeds[0]
    traction_command_list.movement_array.append(traction_command)

    traction_command = RosiMovement()
    traction_command.nodeID = 2
    traction_command.joint_var = traction_speeds[1]
    traction_command_list.movement_array.append(traction_command)

    traction_command = RosiMovement()
    traction_command.nodeID = 3
    traction_command.joint_var = traction_speeds[2]
    traction_command_list.movement_array.append(traction_command)

    traction_command = RosiMovement()
    traction_command.nodeID = 4
    traction_command.joint_var = traction_speeds[3]
    traction_command_list.movement_array.append(traction_command)



#get a list of floats and send it as to VREP as robot arms speed
def arm_speed_callback(data):
    global arm_command_list
    global arm_speeed_pub
    global arm_speeds
    arm_speeds = data.data

    arm_command_list = RosiMovementArray()

    arm_command = RosiMovement()
    arm_command.nodeID = 1
    arm_command.joint_var = arm_speeds[0]
    arm_command_list.movement_array.append(arm_command)

    arm_command = RosiMovement()
    arm_command.nodeID = 2
    arm_command.joint_var = arm_speeds[1]
    arm_command_list.movement_array.append(arm_command)

    arm_command = RosiMovement()
    arm_command.nodeID = 3
    arm_command.joint_var = arm_speeds[2]
    arm_command_list.movement_array.append(arm_command)

    arm_command = RosiMovement()
    arm_command.nodeID = 4
    arm_command.joint_var = arm_speeds[3]
    arm_command_list.movement_array.append(arm_command)




#main
if __name__ == '__main__':
    rospy.init_node('rosi_speed_handler', anonymous=True)
    rospy.Subscriber("/pra_vale/rosi_speed", Float32MultiArray, traction_speed_callback)
    rospy.Subscriber("/pra_vale/rosi_arm_speed", Float32MultiArray, arm_speed_callback)

    print("rosi_speed launched")

    
    node_sleep_rate = rospy.Rate(100)

    while not rospy.is_shutdown(): #always keep publishing the state
        #print_state()
        if(traction_command_list != 0):
            traction_speeed_pub.publish(traction_command_list)
        if(arm_command_list != 0):
            arm_speeed_pub.publish(arm_command_list)

        node_sleep_rate.sleep()
    #rospy.spin()
