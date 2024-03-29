#!/usr/bin/env python

import rospy
rospy.init_node("Robot_control", anonymous=True)
import new_arm as arm
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from rosi_defy.msg import RosiMovement
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints  
import defines as defs

#-----------------------CONSTS----------------------#   
#const received form the ur5_Cam when no fire is detected
FIRE_NOT_FOUND = 1000

#defines the max distance from the robot to the fire to check if the arm is capable of reaching it
_TOUCH_FIRE_FRONT_DISTANCE = 70

#the hokuyo it's the center of the arm, but the torque pointer it isn't
#thus, this defines how much the HOKUYO has to deslocate in the x axis to the pointer be in the center
_HOKUYO_DISPLACEMENT = -3




#-------------------GLOBAL VARIABLES----------------#    
#publish joint values to ur5 arm
arm_publisher = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=10)

#publish robot state changes to others ROS packages
state_publisher = rospy.Publisher('/pra_vale/def_state', Int32, queue_size=15)

#publishes ROSI speed
rosi_speed_publisher = rospy.Publisher("/pra_vale/rosi_speed", Float32MultiArray, queue_size = 1)

#it's true when a fire was detect by the ur5 camera
fire_found = False

#the hokuyo distance when the hokuyo node is not processing
hokuyo_distance = -1

#torque value given from
torque_value = 0

robot_last_orientation = defs.ROBOT_CLOCKWISE

#conts wich show wich state the system is currently, see at the states defines
state = defs.NOTHING

leaving_fire_cont = 0

narrow_path_counter = 0

in_stairs = False

arm_changing_pose_counter = 0

arm_move_sucess_counter = 20

#receive an absolute x,y,z position and sets in the simulation
def arm_pos(data):
    global state
    if(state & 1 << defs.ARM_CHANGING_POSE):
        return

    global arm_publisher
    #print(data) #debug

    arm.x = data.data[0]
    arm.y = data.data[1]
    arm.z = data.data[2]
    
    #get the joints angles
    pos, sucess = arm.cinematicaInversa(state)
    #publish joints angles
    if(sucess):
        arm_publisher.publish(joint_variable = pos)


#callback from the ur5 camera when it detects fire
#receive incremental x,y,z position and sets in the simulation
#the ur5 camera publishes the x distance between the arm and the fire
def arm_move(data):
    #globals
    global state
    global fire_found
    global narrow_path_counter
    global arm_publisher
    global state_publisher
    global rosi_speed_publisher
    global leaving_fire_cont
    global arm_move_sucess_counter


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
    
    #if the arm is detecting a fire, but didn't touch it yet
    if(not state & (1 << defs.LEAVING_FIRE)):
        #if the arm just detected the fire, change it's position 
        if(not fire_found):
            fire_found = True
            state_publisher.publish(data = defs.ENABLE_VELODYME)
            state_publisher.publish(data = defs.SETTING_UP_HOKUYO)
            state_publisher.publish(data = defs.ARM_CHANGING_POSE)

            rosi_speed_publisher.publish(data = [0,0,0,0,0,0])
            if(not state & (1 << defs.IN_STAIR)):
                arm.x = arm.FIRE_FOUND_X_VALUE
                arm.y = arm.FIRE_FOUND_Y_VALUE
                arm.z = arm.FIRE_FOUND_Z_VALUE

                if(state & (1 << defs.ROBOT_ANTICLOCKWISE)):
                    arm.z -= 10
                    arm.x += 100
        
        #if the fire was dected on previously frames, try to get closer to it
        else:
            #the camera sends the distance in the x axis from the arm to the fire
            #if the arm isn't already touching it, then move the arm to the position indicated by the camera
            if(not state & (1 << defs.FOUND_FIRE_FRONT | 1 << defs.FOUND_FIRE_TOUCH | 1 << defs.IN_STAIR)):
                arm.x += x_move
                arm.y += y_move
                arm.z += z_move
                #print("move x : " + str(x))
            
            #try to centralize the fire to te arm to find the fire in the hokuyo reading
            if(state & (1 << defs.SETTING_UP_HOKUYO)):
                #if the arm is centralized with the fire, start hokuyo reading
                if(x_move == 0 and y_move == 0):
                    if(state & (1 << defs.IN_STAIR)):
                        arm.z = arm.z - 60
                        arm.x = arm.x + 40

                    state |= (1 << defs.HOKUYO_READING | 1 << defs.ARM_CHANGING_POSE)
                    rosi_speed_publisher.publish(data = ([0,0,0,0]))
                    state_publisher.publish(data = defs.ARM_CHANGING_POSE)
                    state_publisher.publish(data = defs.HOKUYO_READING)

                #if the fire is in the back of the robot, make the robot go reverse
                elif((x_move < 0 and x_move > -10) or (y_move > 0 and y_move < 10)):
                    state_publisher.publish(data = -defs.ENABLE_VELODYME)
                    state_publisher.publish(data =  defs.SETTING_UP_HOKUYO)
                    rosi_speed_publisher.publish(data = ([-0.2,-0.2,-0.2,-0.2]))

                #if the fire is in the front of the robot, make the robot go foward
                elif(abs(x_move) < 10 and abs(y_move) < 10):
                    state_publisher.publish(data = -defs.ENABLE_VELODYME)
                    state_publisher.publish(data =  defs.SETTING_UP_HOKUYO)

                    #if the robot is it in the stair, the give a little extra speed in the wheels
                    if(state & (1 << defs.IN_STAIR)):
                        rosi_speed_publisher.publish(data = ([0.5,0.5,0.5,0.5]))
                    else:
                        rosi_speed_publisher.publish(data = ([0.2,0.2,0.2,0.2]))

                elif(x_move == -10 or y_move == 10):
                    state_publisher.publish(data = -defs.ENABLE_VELODYME)
                    state_publisher.publish(data =  defs.SETTING_UP_HOKUYO)
                    rosi_speed_publisher.publish(data = ([-0.7,-0.7,-0.7,-0.7]))

            elif(arm.x < 300 and (state & (1 << defs.FOUND_FIRE_RIGHT))):
                arm.z = arm.FIRE_FOUND_Z_VALUE
                rosi_speed_publisher.publish(data = [0,0,0,0])
                
                state_publisher.publish(data = -defs.ENABLE_VELODYME)
                state_publisher.publish(data =  defs.ROBOT_ROTATION)

    #get the joints angles
    pos, sucess  = arm.cinematicaInversa(state)
    #publish joints angles
    if(sucess):
        arm_publisher.publish(joint_variable = pos)
        arm_move_sucess_counter = 20




            
#hokuyo reading callback
#tells the fire position in relation to the arm
#makes the arm touch the fire
def hokuyo_distance_callback(data):
    global state
    global fire_found
    global torque_value
    global state_publisher
    global hokuyo_distance
    global leaving_fire_cont
    global arm_move_sucess_counter

    #get the data from the hokuyo reading
    hokuyo_distance = data.data[0]

    #if the arm is moving, is better not process it, because it will change a lot
    if(state & (1 << defs.ARM_CHANGING_POSE)):
        return

    #const used to move the arm
    x_temp = 0
    y_temp = 0
    x_move = 0
    y_move = 0
    if(state & 1 << defs.ROBOT_CLOCKWISE):
        x_temp = arm.x
        y_temp = arm.y
    else:
        x_temp = arm.y
        y_temp = arm.x

    #the hokuyo gives how much distance the arm is in the x an y axis
    #if the arm is getting closer to the fire it's better that the x axis distance is equal to 0
    if(state & (1 << defs.FOUND_FIRE_RIGHT | 1 << defs.FOUND_FIRE_TOUCH | 1 << defs.FOUND_FIRE_FRONT)):
        x_dist = data.data[1]
        if(x_dist > 10):
            x_dist = 10
        elif(x_dist < -10):
            x_dist = -10

        #print("getting distance by hokuyo. x_dist: " + str(x_dist) + "  plus displacement: " + str(x_dist + _HOKUYO_DISPLACEMENT) + "   x: " + str(arm.x))
        x_move += (int)(x_dist) + _HOKUYO_DISPLACEMENT

    #debug
    #print ("\nhokuyo distance: " + str (hokuyo_distance))

    #change from FOUND_FIRE_RIGHT to FOUND_FIRE_FRONT
    if(not state & (1 << defs.ENABLE_VELODYME | 1 << defs.ROBOT_ROTATION) and state & (1 << defs.FOUND_FIRE_RIGHT)):
        state_publisher.publish(data = -defs.FOUND_FIRE_RIGHT)
        state_publisher.publish(data =  defs.FOUND_FIRE_FRONT)

    #if it's the first reading from the hokuyo, the SETTING_UP_HOKUYO is still up
    elif (state & (1 << defs.SETTING_UP_HOKUYO)):
        if(hokuyo_distance > _TOUCH_FIRE_FRONT_DISTANCE):
            state_publisher.publish(data = -defs.SETTING_UP_HOKUYO)
            state_publisher.publish(data =  defs.ENABLE_VELODYME)
            state_publisher.publish(data = -defs.FOUND_FIRE_RIGHT)
            state_publisher.publish(data = -defs.HOKUYO_READING)
        else:
            state_publisher.publish(data = -defs.SETTING_UP_HOKUYO)
            state_publisher.publish(data = -defs.ENABLE_VELODYME)
            state_publisher.publish(data =  defs.FOUND_FIRE_TOUCH)
            state_publisher.publish(data =  defs.HOKUYO_READING)
            arm.camera_tilt = 0
            arm.tilt_y = 0

    #if the robot is facing the fire, make it go foward while is distante from the fire
    elif (state & (1 << defs.FOUND_FIRE_FRONT) and hokuyo_distance > _TOUCH_FIRE_FRONT_DISTANCE):
        rosi_speed_publisher.publish(data = [2,2,2,2])
    
    #if the robot is facing the fire and it's closer to it, rotate the robot to be able to touch the fire
    elif(hokuyo_distance <= _TOUCH_FIRE_FRONT_DISTANCE and state & (1 << defs.FOUND_FIRE_FRONT)):
        rosi_speed_publisher.publish(data = [0,0,0,0])
        state_publisher.publish(data = -defs.FOUND_FIRE_FRONT)
        state_publisher.publish(data =  defs.ROBOT_ROTATION)
        state_publisher.publish(data =  defs.FOUND_FIRE_TOUCH)
    
    #if the robot is close to the fire...
    elif(state & (1 << defs.FOUND_FIRE_TOUCH) and not state & (1 << defs.ROBOT_ROTATION)):
        #if the distance in the x axis is still big, the make it go foward or reverse
        if(state & (1 << defs.ROBOT_CLOCKWISE) and (x_temp > 400 or x_temp < 300)):
            if (x_temp > 400):
                rosi_speed_publisher.publish(data = ([0.5,0.5,0.5,0.5]))
            
            elif (x_temp < 300):
                rosi_speed_publisher.publish(data = ([-0.5,-0.5,-0.5,-0.5]))

        #if the distance in the x axis is still big, the make it go foward or reverse
        elif(not (state & (1 << defs.ROBOT_CLOCKWISE)) and (x_temp < -50 or x_temp > 0)):
            if (x_temp < -50):
                rosi_speed_publisher.publish(data = ([0.5,0.5,0.5,0.5]))
            
            elif (x_temp > 0):
                rosi_speed_publisher.publish(data = ([-0.5,-0.5,-0.5,-0.5]))
        
        #if the distance in the x axis is little, the make the arm go in the direction of the fire
        elif (torque_value < 0.13 and not state & (1 << defs.LEAVING_FIRE)):
            y_move -= 4
            if(state & (1 << defs.IN_STAIR)):
                if(arm.z > arm.FIRE_TOUCH_Z_VALUE_IN_STAIRS):
                    arm.z -= 2
            else:
                if(arm.z > arm.FIRE_TOUCH_Z_VALUE):
                    arm.z -= 2
                    if(state & (1 << defs.BEAM_FIND)):
                        arm.z -= 2
            
            rosi_speed_publisher.publish(data = [0,0,0,0])

        #if the arm alrady touch the fire, but it's still close to it. Make it go backwards, but no really fast
        elif(torque_value >= 0.1 and not state & (1 << defs.LEAVING_FIRE)): 
            leaving_fire_cont = 0
            state_publisher.publish(data = defs.LEAVING_FIRE)
            fire_found = False
        
        #if the arm alrady touch the fire and is distant from it. make it go backward and fast
        elif(state & (1 << defs.LEAVING_FIRE) and hokuyo_distance < 25):
            if hokuyo_distance > 15:
                y_move += 10
            else:
                y_move += 5

        #if the arm alrady touch the fire and is really distant from it. return to the original position
        elif(state & (1 << defs.LEAVING_FIRE) and hokuyo_distance >= 25):
            if(narrow_path_counter):
                arm.x = arm.NARROW_PATH_X_VALUE
                arm.y = arm.NARROW_PATH_Y_VALUE
                arm.z = arm.NARROW_PATH_Z_VALUE
            else:
                arm.x = arm.FIRE_NOT_FOUND_X_VALUE
                arm.y = arm.FIRE_NOT_FOUND_Y_VALUE
                arm.z = arm.FIRE_NOT_FOUND_Z_VALUE

            pos, sucess  = arm.cinematicaInversa(state)
            if(sucess):
                arm_publisher.publish(joint_variable = pos)
            
            state_publisher.publish(data = -defs.FOUND_FIRE_TOUCH)
            state_publisher.publish(data = -defs.HOKUYO_READING)
            state_publisher.publish(data =  defs.ENABLE_VELODYME)
            state_publisher.publish(data =  defs.ARM_CHANGING_POSE)
            return
    
    #move the arm
    if(state & 1 << defs.ROBOT_CLOCKWISE):
        arm.x = x_temp + x_move
        arm.y = y_temp + y_move
    else:
        arm.x = y_temp - y_move
        arm.y = x_temp + x_move

    pos, sucess  = arm.cinematicaInversa(state)
    if(sucess):
        arm_publisher.publish(joint_variable = pos)
        arm_move_sucess_counter = 20
    else:
        arm_move_sucess_counter -= 1
        if(arm_move_sucess_counter == 0):
            state_publisher.publish(data = defs.LEAVING_FIRE)
            state_publisher.publish(data = defs.ENABLE_VELODYME)
            state_publisher.publish(data = -defs.HOKUYO_READING)
            state_publisher.publish(data = -defs.FOUND_FIRE_FRONT)
            state_publisher.publish(data = -defs.FOUND_FIRE_TOUCH)
            state_publisher.publish(data = -defs.FOUND_FIRE_FRONT)
            state_publisher.publish(data = -defs.SETTING_UP_HOKUYO)

            arm.x = arm.FIRE_NOT_FOUND_X_VALUE
            arm.y = arm.FIRE_NOT_FOUND_Y_VALUE
            arm.z = arm.FIRE_NOT_FOUND_Z_VALUE
            pos, sucess  = arm.cinematicaInversa(state)
            arm_publisher.publish(joint_variable = pos)
            arm_move_sucess_counter = 20

    #print("xtemp: " + str(x_temp) + ", ytemp: " + str(y_temp) + ",  xmove: " + str(x_move) + ",  ymove: " + str(y_move))




def state_callback(data):
    global state
    global in_stairs
    global narrow_path_counter
    global arm_publisher
    global state_publisher
    global rosi_speed_publisher
    global robot_last_orientation
    state = data.data

    if (state & (1 << defs.INITIAL_SETUP)):
        global arm_publisher
        global state_publisher
        pos, sucess  = arm.cinematicaInversa(state)
        if(sucess):
            arm_publisher.publish(joint_variable = pos)

        speed = [0,0,0,0]
        rosi_speed_publisher.publish(data = speed)
    

    #if the arm is it in
    if ((state & (1 << defs.NARROW_PATH | 1 << defs.IN_STAIR)) and not narrow_path_counter):
        narrow_path_counter = 40
        arm.x = arm.NARROW_PATH_X_VALUE
        arm.y = arm.NARROW_PATH_Y_VALUE
        arm.z = arm.NARROW_PATH_Z_VALUE
        pos, sucess  = arm.cinematicaInversa(state)
        if(sucess):
            arm_publisher.publish(joint_variable = pos)

        state_publisher.publish(data = defs.ARM_CHANGING_POSE)

    elif(not state & (1 << defs.NARROW_PATH | 1 << defs.IN_STAIR) and narrow_path_counter):
        narrow_path_counter -= 1
        #print("narrow path counter = " + str(narrow_path_counter))
        if(narrow_path_counter == 0):
            arm.x = arm.FIRE_NOT_FOUND_X_VALUE
            arm.y = arm.FIRE_NOT_FOUND_Y_VALUE
            arm.z = arm.FIRE_NOT_FOUND_Z_VALUE
            pos, sucess  = arm.cinematicaInversa(state)
            if(sucess):
                arm_publisher.publish(joint_variable = pos)

    if (state & (1 << defs.CLIMB_STAIR)):
        arm_publisher.publish(joint_variable = [0,0,0,0,0,0])

    if(not state & (1 << robot_last_orientation)):
        if(robot_last_orientation == defs.ROBOT_CLOCKWISE):
            robot_last_orientation = defs.ROBOT_ANTICLOCKWISE
        else:
            robot_last_orientation = defs.ROBOT_CLOCKWISE

        arm.x = arm.FIRE_NOT_FOUND_X_VALUE
        arm.y = arm.FIRE_NOT_FOUND_Y_VALUE
        arm.z = arm.FIRE_NOT_FOUND_Z_VALUE
        pos, sucess  = arm.cinematicaInversa(state)
        if(sucess):
            arm_publisher.publish(joint_variable = pos)
        state_publisher.publish(data = defs.ARM_CHANGING_POSE)
        rosi_speed_publisher.publish(data = [0,0,0,0,0,0])

    if (state & (1 << defs.IN_STAIR) and not in_stairs):
        in_stairs = True
        arm.x = arm.IN_STAIRS_X_VALUE
        arm.y = arm.IN_STAIRS_Y_VALUE
        arm.z = arm.IN_STAIRS_Z_VALUE
        pos, sucess  = arm.cinematicaInversa(state)
        if(sucess):
            arm_publisher.publish(joint_variable = pos)
        state_publisher.publish(data = defs.ARM_CHANGING_POSE)
        state_publisher.publish(data = defs.FORCE_VELODYME)

    if(((state & (1 << defs.ARM_CHANGING_POSE)) and (not (state & (1 << defs.FORCE_VELODYME))))):
        rosi_speed_publisher.publish(data = [0,0,0,0,0,0])
    elif(((state & (1 << defs.FORCE_VELODYME)) and (not (state & (1 << defs.ARM_CHANGING_POSE))))):
        state_publisher.publish(data = -defs.FORCE_VELODYME)


def torque_callback(data):
    global torque_value
    torque_value = data.twist.linear.z




#get arm joint angles
#used when it's necessary to the robot to wait the arm reachs it's designed position
def arm_current_position(data):
    global state
    global state_publisher
    global rosi_speed_publisher
    global arm_changing_pose_counter
         
    if(state & (1 << defs.ARM_CHANGING_POSE)):

        #if all joint angles are in the correct places, remove the _ARM_CHANGING_POSE from the states
        for x in range(len(arm.joint_angles)):
            if(abs(arm.joint_angles[x] - data.joint_variable[x]) > defs.MAX_JOINT_ANGLE_DIFF):
                return

        if(state & (1 << defs.INITIAL_SETUP)):
    
            state_publisher.publish(data = -defs.ARM_CHANGING_POSE)
            state_publisher.publish(data = -defs.INITIAL_SETUP)
            state_publisher.publish(data =  defs.ENABLE_VELODYME)
        else:
            if(state & (1 << defs.IN_STAIR) and state & (1 << defs.HOKUYO_READING)):
                if(arm_changing_pose_counter == 0 or abs(torque_value) > 0.1):
                    arm_changing_pose_counter = 12
                
                arm_changing_pose_counter -= 1
                if(arm_changing_pose_counter == 0):
                    state_publisher.publish(data = -defs.ARM_CHANGING_POSE)
                    state_publisher.publish(data = -defs.FORCE_VELODYME)

            else:
                state_publisher.publish(data = -defs.ARM_CHANGING_POSE)
                state_publisher.publish(data = -defs.FORCE_VELODYME)


#callback from the IMU sensor, make the ur5 arm follow the track automatically
def arm_imu(data):
    global state
    global arm_publisher
    if(state & (1 << defs.ARM_CHANGING_POSE | 1 << defs.CLIMB_STAIR)):
        return

    temp_y = 0.0
    angles = data.data
    if(state & (1 << defs.ROBOT_CLOCKWISE)):
        arm.tilt_x = angles[0]
        temp_y = -angles[1] #+ 0.065
        arm.tilt_z = angles[2]
    else:
        arm.tilt_x = -angles[0]
        temp_y = -angles[1] - 0.065
        arm.tilt_z = angles[2]
        if(arm.tilt_z > arm._pi):
            arm.tilt_z = arm.tilt_z - 2*arm._pi
    
    if arm.get_tilt_y_from_imu:
        arm.tilt_y = temp_y
    else:
        arm.tilt_y = arm.camera_tilt

        if(not (state & (1 << defs.IN_STAIR))):
            if arm.tilt_y > temp_y + arm.IMU_TILT_ERROR:
                arm.tilt_y = temp_y + arm.IMU_TILT_ERROR

            elif arm.tilt_y < temp_y - arm.IMU_TILT_ERROR:
                arm.tilt_y = temp_y - arm.IMU_TILT_ERROR
    
        
    pos, sucess  = arm.cinematicaInversa(state)
    #publish joints angles
    if(sucess):
        arm_publisher.publish(joint_variable = pos)



#callback from the ur5 camera, make the ur5 arm follow the track automatically
def arm_tilt(data):
    global state
    #if the arm is changing positon or the robot is climbing the stair, it's better to not get the robot orientation
    if((state & (1 << defs.ARM_CHANGING_POSE | 1 << defs.CLIMB_STAIR))): 
        return

    #if the camera isn't able to get the camera tilt, the get it from the imu
    if((not state & (1 << defs.IN_STAIR)) and (data.data == -1 or state & (1 << defs.FOUND_FIRE_FRONT | 1 << defs.FOUND_FIRE_TOUCH))):
        arm.get_tilt_y_from_imu = True
    else:
        if(arm.get_tilt_y_from_imu == True):
            arm.get_tilt_y_from_imu = False
            arm.camera_tilt = 0.0
        
        if(data.data != -1):
            arm.camera_tilt += data.data
            #print(arm.camera_tilt) #degub


#main
def listener():
    global state
    arm.cinematicaInversa(state)

    rospy.Subscriber('/pra_vale/arm_tilt', Float32, arm_tilt)  
    rospy.Subscriber("/pra_vale/estados", Int32, state_callback)
    rospy.Subscriber("/pra_vale/imu", Float32MultiArray, arm_imu)
    rospy.Subscriber('/pra_vale/arm_move', Int32MultiArray, arm_move)
    rospy.Subscriber("/ur5/forceTorqueSensorOutput", TwistStamped, torque_callback)
    rospy.Subscriber("/pra_vale/hokuyo_distance", Int32MultiArray, hokuyo_distance_callback)
    rospy.Subscriber("/ur5/jointsPositionCurrentState", ManipulatorJoints, arm_current_position)

    rospy.spin()



#main
print("robot launched")
listener()