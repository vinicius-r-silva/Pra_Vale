#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from rosi_defy.msg import HokuyoReading
import numpy as np
import cv2

max_range = 25

enabled = 0

def callback(data):
    global enabled
    #if(enabled == 0):
    #    return

    blank_image = np.zeros((500,500), np.uint8)

    for x in range(499):
        blank_image[x,250] = 120
    
    size = len(data.reading)
    i = 0
    #item = [0, 0, 0]
    #indeces =[(int)(0)]
    # while (i < len(data.reading)):
    #     print(data.reading[i], end = '')
    #     print("  ", end = '')
    #     if(i % 3 == 2):
    #         print(" ")
    #     i = i + 1

    melhor_x_centro = 1000
    melhor_y_robo   = 0
    distancia = 0
    i = 0
    while i < size:
        y = data.reading[i]
        x = data.reading[i + 1]
        #c = data.reading[i + 2]
        #item = [a,b,c]

        y = 500 - (y+5)*50
        x = 500 - (x+5)*50

        if(y < 250 and y > melhor_y_robo):
            melhor_y_robo = y
        
        if(melhor_y_robo - y  < max_range):
            if abs(x - 250) < melhor_x_centro:
                melhor_x_centro = abs(x - 250)
                distancia = 250 - y

            blank_image[(int)(y) , (int)(x)] = 255
        #indeces.append((int)((a+5)*50*50 + (b+5)*5))
        #print(item)
        i += 3

    #print("\n\n\n")
    #print(indeces)
    #print(type(indeces))
    #teste = [2,3,4]
    #blank_image[indeces] = 255
    np.transpose(blank_image)
    print("dist: " + str(distancia) + ",  best_y: " + str(melhor_y_robo))
    #print("\n\n\n")
    cv2.imshow('image',blank_image)
    cv2.waitKey(1)

def enable_hokuyo(data):
    global enabled
    enabled = data.data
    print("hokuyo is: " + str(enabled))

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensor/hokuyo", HokuyoReading, callback)
    rospy.Subscriber("/pra_vale/enable_hokuyo", Int8, enable_hokuyo)

    rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")

listener()