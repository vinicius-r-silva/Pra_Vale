#!/usr/bin/env python

import cv2
import time
import rospy
import numpy as np
import defines as defs
from math import atan2
from cv_bridge import CvBridge
from scipy.optimize import leastsq

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

#-----------------------CONSTS----------------------#   
#const send to the arm node when no fire is detected
_FIRE_NOT_FOUND = 1000

#consts used on the detection if thre is a track at the cam
#define the row use on the detection
_TRACK_DETECTION_ROW = 150
#define the max qtd of balck pixels
_TRACK_DETECTION_MAX_PIXELS = 25

#defines how close the fire has to be to center of the image to be considered fire
_ERROR_UPPER_LIMIT = 150

#-------------------GLOBAL VARIABLES----------------# 
#enabled = True

#publish joint values to arm.py
arm_move = rospy.Publisher('/pra_vale/arm_move', Int32MultiArray, queue_size=10)
arm_tilt = rospy.Publisher('/pra_vale/arm_tilt', Float32, queue_size=10)

#publish robot state changes to others ROS packages
state_publisher = rospy.Publisher('/pra_vale/def_state', Int32, queue_size=15)


#desired z (height) position of the arm
desired_z = 20

#state of the robot
state = 1 << defs.NOTHING

leaving_fire_counter = 0

cancel_setting_up_hokuyo_conter = 0

found_fire_counter = 0


#----------------------FUNCTIONS----------------------# 

def state_callback(data):
    global state
    state = data.data



#given two array of values, find the best line that best fits the arrays
def leastSquare (x_list, y_list):
    x_array = np.array(y_list)
    y_array = np.array(x_list)

    # here, create lambda functions for Line fit
    # tpl is a tuple that contains the parameters of the fit
    func=lambda tpl,x_array : tpl[0]*x_array+tpl[1]

    # ErrorFunc is the diference between the func and the y "experimental" data
    ErrorFunc=lambda tpl,x_array,y_array: func(tpl,x_array)-y_array

    #tplInitial contains the "first guess" of the parameters 
    tplInitial1=(1.0,2.0)

    # leastsq finds the set of parameters in the tuple tpl that minimizes
    tplFinal1,success=leastsq(ErrorFunc,tplInitial1[:],args=(x_array,y_array))
    return tplFinal1


#detect if the track is in front of the camera
#used when necessary to get the camera tilt angle
def there_is_track(frame):
    row = _TRACK_DETECTION_ROW
    black_pixels_qtd = 0
    col = 0

    #check if determined row has a suficient pixels that differs from black
    while col < frame.shape[1]:
        if np.all(frame[row, col] == [0,0,0]):
            black_pixels_qtd += 1
            if(black_pixels_qtd > _TRACK_DETECTION_MAX_PIXELS):
                return False

        col += 1
    
    #for debug draw a blue line where the pixels were verified
    if(defs.DEBUGGING):
        cv2.line(frame, (0, _TRACK_DETECTION_ROW), (frame.shape[1], _TRACK_DETECTION_ROW), (255,0,0))

    return True



#given a frame from the track, get the tilt angle of the camera
#find the firsts black pixels from the botton
def get_tilt_angle(frame):
    #consts
    qtd = 200               #number of columns to analyse
    rows = frame.shape[0]   #number of rows of the frame
    cols = frame.shape[1]   #number of columns of the frame
    step = (int)(cols/qtd)  #step wich the the column number is incresed on the main loop
    
    #the collumns are analysed from the botton to the top, checking where the first black pixel appear
    #the botton limit and the top limit are defined bellow
    upper_limit = (int)((2*rows)/3) 
    botton_limit = (int)(rows - 1) - 20
    
    #variables used on the main loop
    current_col = 0      
    pixels_found = 0

    #main loop
    #check the first black pixels from the botton of the image
    #then call the leastSquare funtion the find the best line
    cont = 0
    x_list = []
    y_list = []
    while cont < qtd:
        x = botton_limit   #start from the botton
        if np.all(frame[x, current_col] == 0):
            cont += 1
            current_col += step
            continue
        x -= 1
        while(x > upper_limit): #to the upperlimit
            B = frame[x, current_col][0]
            G = frame[x, current_col][1]
            R = frame[x, current_col][2]
            if R == G == B == 0 or (abs(((int)(R) - (int)(G)) + (int)((R) - (int)(B))) > 20): #check if pixel is black
                x_list.append(x)                   #if it is, count it
                y_list.append(current_col)
                pixels_found += 1
                # frame[x, current_col][0] = 0  #paint red the found pixel
                # frame[x, current_col][1] = 0
                # frame[x, current_col][2] = 255                
                break
            x -= 1

        cont += 1
        current_col += step

    #if not enough pixels were found, cancel the search
    if(pixels_found < (qtd / 5)):
        return [-1,-1]
        
    #otherwise, calculates the leastSquare
    coef = leastSquare(x_list, y_list)

    #draw the line found
    if(defs.DEBUGGING):
        cv2.line(frame, (0, (int)(coef[1])), (cols, (int)(coef[1] + coef[0]*cols)), (0,0,255), 1)

    #calculates the angle of the line made from the leastSquare function
    angle = atan2(coef[0]*cols, cols)
    return [angle, coef[1]]



#given a frame from the track, get the tilt angle of the camera
def get_tilt_angle_while_in_stairs(frame):
    #consts
    qtd = 200               #number of columns to analyse
    rows = frame.shape[0]   #number of rows of the frame
    cols = frame.shape[1]   #number of columns of the frame
    step = (int)(cols/qtd)  #step wich the the column number is incresed on the main loop
    
    #the collumns are analysed from the top to the botton, checking where the first mettalic pixel appear
    #the botton limit and the top limit are defined bellow
    upper_limit  = (int)(10) 
    botton_limit = (int)(rows/2)
    
    #variables used on the main loop
    current_col = 0      
    pixels_found = 0

    #main loop
    #check the first black pixels from the botton of the image
    #then call the leastSquare funtion the find the best line
    cont = 0
    x_list = []
    y_list = []
    while cont < qtd:
        x = upper_limit   #start from the top
        while(x < botton_limit): #to the botton
            B = frame[x, current_col][0]
            G = frame[x, current_col][1]
            R = frame[x, current_col][2]
            if (R != 0 and (R == G == B)): #check if pixel is metallic
                x_list.append(x)                   #if it is, count it
                y_list.append(current_col)
                pixels_found += 1
                # frame[x, current_col][0] = 0   #paint red the found pixel
                # frame[x, current_col][1] = 0
                # frame[x, current_col][2] = 255                
                break
            x += 1

        cont += 1
        current_col += step

    #if not enough pixels were found, cancel the search
    if(pixels_found < (qtd / 5)):
        return [-1,-1]
        
    #otherwise, calculates the leastSquare
    coef = leastSquare(x_list, y_list)

    #draw the line found
    if(defs.DEBUGGING):
        cv2.line(frame, (0, (int)(coef[1])), (cols, (int)(coef[1] + coef[0]*cols)), (0,0,255), 1)

    #calculates the angle of the line made from the leastSquare function
    angle = atan2(coef[0]*cols, cols)
    return [angle, coef[1]]


#ur5Cam Callback
#finds fire in the given image and calculates the x,y coordinates of the fire
def ur5_callback(data):
    global state
    global arm_move
    global arm_tilt
    global found_fire_counter
    global leaving_fire_counter
    global cancel_setting_up_hokuyo_conter

    #get the image
    bridge=CvBridge()
    frame = cv2.flip(cv2.cvtColor(bridge.imgmsg_to_cv2(data),cv2.COLOR_BGR2RGB),1)

    #check if it's necessary process the image
    if(state & ((1 << defs.ARM_CHANGING_POSE) | (1 << defs.FOUND_FIRE_FRONT) | (1 << defs.FOUND_FIRE_TOUCH) | (1 << defs.NOTHING))):
        cv2.imshow("Camera",frame) #if it isn't necessary, show the image and exit
        cv2.waitKey(1)
        return


    #Calculates RGB threshold to find fire
    mask=cv2.inRange(frame,(0,175,210),(100,255,255))
    
    #Find contours of mask
    contours, im = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    #Check size of object
    error = _FIRE_NOT_FOUND
    if(len(contours) > 0 and cv2.contourArea(max(contours,key=cv2.contourArea))>100):
        #Get biggest object found
        fire = max(contours,key=cv2.contourArea)
        
        # Draw contour:
        cv2.drawContours(frame, [fire], 0, (0, 0, 255), 3)

		# Calculate image moments of the detected contour
        M = cv2.moments(fire)

		# Draw center:
        x = M['m10'] / M['m00']
        cv2.line(frame, (frame.shape[1]/2,0), (frame.shape[1]/2, frame.shape[0]), (255,0,0), 1)
        cv2.line(frame, ((int)(x),0), ((int)(x), frame.shape[0]), (0,255,0), 1)

        #calculates how far the fire center is from the center of the frame
        error = frame.shape[1]/2 - x
        if(abs(error) > _ERROR_UPPER_LIMIT and not state & (1 << defs.SETTING_UP_HOKUYO | 1 << defs.ROBOT_CLOCKWISE)): #if the fire it's to far, ignores
            error = _FIRE_NOT_FOUND
        else:
            if(error > 10):
                error = 10
            elif(error < -10):
                error = -10
            

    #check if there is a track on the camera sight
    if(state & (1 << defs.IN_STAIR) and not (state & (1 << defs.SETTING_UP_HOKUYO | 1 << defs.HOKUYO_READING | 1 << defs.LEAVING_FIRE))):
        angle, b = get_tilt_angle_while_in_stairs(frame)
        
    elif ((not state & (1 << defs.IN_STAIR)) and there_is_track(frame)): #if there is a track, get the tilt angle from it
        angle, b = get_tilt_angle(frame)
    else:                     #ortherwise, just return -1
        angle = -1
        b = -1
    
    #publish it to the arm node   
    arm_tilt.publish(data = angle/2)

    #calculates how much the arm has to move in the z axis
    #keeps the camera in the same height as the the track rolls
    z = 0
    if((state & (1 << defs.FOUND_FIRE_FRONT | 1 << defs.FOUND_FIRE_TOUCH)) and b != -1):
        z = (b - desired_z)/5

    

    if (state & 1 << defs.FIRE_FOUND_BY_CAM and error == _FIRE_NOT_FOUND):
        if(found_fire_counter == 0):
            found_fire_counter = 20
        
        found_fire_counter -= 1
        if(found_fire_counter == 0):
            state_publisher.publish(data = -defs.FIRE_FOUND_BY_CAM)

    if (state & 1 << defs.LEAVING_FIRE and error == _FIRE_NOT_FOUND):
        if(leaving_fire_counter == 0):
            leaving_fire_counter = 20
        
        leaving_fire_counter -= 1
        if(leaving_fire_counter == 0):
            state_publisher.publish(data = -defs.LEAVING_FIRE)

    if(not (state & (1 << defs.BEAM_FIND))):
        #publishes to the arm node
        if(error != _FIRE_NOT_FOUND):
            cancel_setting_up_hokuyo_conter = 0
            arm_move.publish(data = [error, 0, z])

            if(not (state & (1 << defs.FIRE_FOUND_BY_CAM))):
                state_publisher.publish(data = defs.FIRE_FOUND_BY_CAM)
        
        elif(state & (1 << defs.SETTING_UP_HOKUYO)):
            if (cancel_setting_up_hokuyo_conter == 0):
                cancel_setting_up_hokuyo_conter = 20
            
            cancel_setting_up_hokuyo_conter -= 1
            if(cancel_setting_up_hokuyo_conter == 0):
                state_publisher.publish(data = -defs.SETTING_UP_HOKUYO)


    cv2.imshow("Camera",frame)
    cv2.waitKey(1)



#get robot state
def state_set(data):
    global state
    state = data.data



def listener():
    rospy.init_node('findFire', anonymous=True)

    rospy.Subscriber("/sensor/ur5toolCam", Image, ur5_callback)
    rospy.Subscriber("/pra_vale/estados", Int32, state_set)
    
    rospy.spin()



#main
print("my_5cam launched")
listener()