#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from rosi_defy.msg import HokuyoReading
import numpy as np
import cv2

_IMG_SIZE = 1000
_MAX_RANGE = 50
_RADIOS_FIRE = 9

_STATE_DISABLE = 0
_STATE_READING = 1
_STATE_FOLLOWING = 2

_HOKUYO_READING_MAX = 10

last_distance = 0
state = _STATE_DISABLE
is_following_fire = False

distance_publisher = rospy.Publisher('/pra_vale/hokuyo_distance', Int32, queue_size = 1)


def getHalfDistance(data):

    blank_image = np.zeros((_IMG_SIZE,_IMG_SIZE), np.uint8)
    size = len(data.reading)

    i = 0
    distancia = 0
    menor_distancia   = 0
    melhor_x_robo   = 0
    melhor_x_centro = _IMG_SIZE

    half_img_size = _IMG_SIZE/2
    half_max = _HOKUYO_READING_MAX/2
    Kp = _IMG_SIZE/_HOKUYO_READING_MAX

    x_fogo = 0
    y_fogo = 0

    while i < size:
        y = data.reading[i]
        x = data.reading[i + 1]

        y = _IMG_SIZE - (y+half_max)*Kp
        x = _IMG_SIZE - (x+half_max)*Kp

        if(y < half_img_size and y > menor_distancia):
            menor_distancia = y
        
        if(menor_distancia - y  < _MAX_RANGE):
            if abs(x - half_img_size) < melhor_x_centro:
                melhor_x_centro = abs(x - half_img_size)
                melhor_x_robo = x
                y_fogo = y
                distancia = half_img_size - y

            blank_image[(int)(y) , (int)(x)] = 255
            
        i += 3

    for x in range(499):
        blank_image[x,half_img_size] = 120

    #distance_publisher.publish(data = distancia)
    np.transpose(blank_image)
    print("dist: " + str(distancia))
    #cv2.imshow('image',blank_image)
    cv2.waitKey(1)

    print(menor_distancia, melhor_x_centro)
    return (int) (y_fogo), (int) (melhor_x_robo), blank_image



def find_nearest_white(img, target):
    nonzero = np.argwhere(img == 255)
    distances = (nonzero[:,0] - target[0]) ** 2 + (nonzero[:,1] - target[1]) ** 2
    nearest_index = np.argmin(distances)
    return nonzero[nearest_index]



def callback(data):
    global state
    global last_distance
    global distance_publisher   
    if(state == _STATE_DISABLE):
        if(last_distance != -1):
            distance_publisher.publish(data = -1)
            last_distance = -1

        return

    elif(state == _STATE_FOLLOWING or state == _STATE_READING):
        fogoY, fogoX, blank_image = getHalfDistance(data)
        cv2.circle(blank_image, (fogoX,fogoY), _RADIOS_FIRE, 255, 1,8,0)
        
        try:
            fogoCoord = find_nearest_white(blank_image,(fogoX,fogoY))
            print ("fogo: " + str(fogoCoord))
            cv2.circle(blank_image, tuple(fogoCoord) , _RADIOS_FIRE, 100, 1,8,0)
            
        except Exception as e:
            print(e)
        
        

        cv2.imshow('image', blank_image)

        

def hokuyo_state(data):
    global state
    state = data.data
    print("hokuyo is: " + str(state))

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensor/hokuyo", HokuyoReading, callback)
    rospy.Subscriber("/pra_vale/hokuyo_state", Int32, hokuyo_state)

    rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")

listener()