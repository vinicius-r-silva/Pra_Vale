#!/usr/bin/env python
import rospy
import defines as defs
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from rosi_defy.msg import HokuyoReading
import numpy as np
import cv2

#consts
_IMG_SIZE = 750  #the size of the frame generated from the HOKUYO reading
_RADIOS_FIRE = 9

_HOKUYO_READING_MAX = 10

last_fire_coord = [-1,-1]
state = defs.NOTHING
is_following_fire = False

distance_publisher = rospy.Publisher('/pra_vale/hokuyo_distance', Int32MultiArray, queue_size = 1)


#make an grayscale image from the hokuyo data.reading
def getFrame(data):
    size = len(data.reading)
    half_max = _HOKUYO_READING_MAX/2
    Kp = _IMG_SIZE/_HOKUYO_READING_MAX

    i = 0
    frame = np.zeros((_IMG_SIZE,_IMG_SIZE), np.uint8) #create a black image

    #interate every three elements, saving the x and y values
    while i < size:                
        y = data.reading[i]     
        x = data.reading[i + 1]

        #transform the hokuyo value to pixels position
        y = _IMG_SIZE - (y+half_max)*Kp
        x = _IMG_SIZE - (x+half_max)*Kp

        #paint white the x,y pixel 
        frame[(int)(y) , (int)(x)] = 255 
        i += 3
    return frame


#given a frame, find the nearest pixel from the robot and from the center of the y axis
def getFireInitialPos(frame):
    botton_limit = _IMG_SIZE/2
    upper_limit = _IMG_SIZE/5

    #given that the robot is in the center of the image
  
    #search the center line of the until found a white 
    i = botton_limit
    half_frame = _IMG_SIZE/2
    while(i > upper_limit):
        if(frame[i][half_frame] == 255):
            return i, half_frame
        i -= 1
    
    #if no white pixel was found in the previously loop
    #search in the adjacents columns of the image
    dist = 1
    while (dist < half_frame):
        i = botton_limit
        j_r = half_frame + dist
        j_l = half_frame - dist
        while(i > upper_limit):
            if(frame[i][j_r] == 255):
                return i, j_r

            if(frame[i][j_l] == 255):
                return i, j_l

            i -= 1        
        dist += 1

    return -1,-1


#given a previously position of the fire, find where is the given frame
#search in a small area arround the fire position given 
#and return the center point of the white pixels
def getFireCenter(frame, fogoX, fogoY):

    #search limits form a small square arround the previously fire position
    botton = fogoY + _RADIOS_FIRE
    top = fogoY - _RADIOS_FIRE
    left = fogoX - _RADIOS_FIRE
    right = fogoX + _RADIOS_FIRE

    i = top
    j = left
    sumY = 0
    sumX = 0
    points_found = 0

    #find all white pixels inside the small square
    #make weighted average of the pixels location to find the center of all white pixels
    while i < botton:
        j = left
        while(j < right):
            if(frame[i][j] == 255):
                points_found += 1
                sumY += i - top + 1
                sumX += j - left + 1
            j += 1
        i += 1
    
    #calculates the center pixel
    if (points_found == 0):
        fogoX = 0
        fogoY = 0
    else:
        fogoX = fogoX + (sumX/points_found - _RADIOS_FIRE)
        fogoY = fogoY + (sumY/points_found - _RADIOS_FIRE)

    #print for debuging
    #print((sumX/points_found - _RADIOS_FIRE), (sumY/points_found - _RADIOS_FIRE))
    #print(fogoX, fogoY)
    
    return fogoX, fogoY



def hokuyo_callback(data):
    global state
    global last_fire_coord
    global distance_publisher   
    if(not (state & 1 << defs.HOKUYO_READING)):
        if(last_fire_coord[0] != -1):
            last_fire_coord[0] = -1
            last_fire_coord[1] = -1
        return

    if(state & (1 << defs.ARM_CHANGING_POSE)):
        return

    frame = getFrame(data)

    if(last_fire_coord[0] == -1):
        fogoY, fogoX = getFireInitialPos(frame)
        cv2.circle(frame    , (fogoX,fogoY), _RADIOS_FIRE, color = 255, thickness = 1, lineType = 8, shift = 0)
        last_fire_coord[0] = fogoX
        last_fire_coord[1] = fogoY
    else:
        fogoX, fogoY = getFireCenter(frame, last_fire_coord[0], last_fire_coord[1])
        last_fire_coord[0] = fogoX
        last_fire_coord[1] = fogoY

    dist_Y = _IMG_SIZE/2 - fogoY
    dist_X = _IMG_SIZE/2 - fogoX

    cv2.circle(frame, (fogoX,fogoY), _RADIOS_FIRE, color = 255, thickness = 1, lineType = 8, shift = 0)
    cv2.imshow('hokuyo', frame)
    cv2.waitKey(1)

    distance_publisher.publish(data = [dist_Y, dist_X])

    # cv2.imshow('image', frame)
    # cv2.waitKey(1)

            
#enables or disables the hokuyo processing
def state_callback(data):
    global state
    state = data.data



def listener():
    rospy.init_node('hokuyo', anonymous=True)

    rospy.Subscriber("/sensor/hokuyo", HokuyoReading, hokuyo_callback)
    rospy.Subscriber("/pra_vale/estados", Int32, state_callback)

    rospy.spin()


#main
print("hokuyo launched")
listener()