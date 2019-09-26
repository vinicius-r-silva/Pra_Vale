#!/usr/bin/env python3

import rospy
from rosi_defy.msg import HokuyoReading
import numpy as np
import cv2

def callback(data):
    blank_image = np.zeros((500,500), np.uint8)
    x = len(data.reading)
    i = 0
    #item = [0, 0, 0]
    #indeces =[(int)(0)]
    # while (i < len(data.reading)):
    #     print(data.reading[i], end = '')
    #     print("  ", end = '')
    #     if(i % 3 == 2):
    #         print(" ")
    #     i = i + 1

    i = 0
    while i < x:
        a = data.reading[i]
        b = data.reading[i + 1]
        c = data.reading[i + 2]
        item = [a,b,c]
        blank_image[(int)(500 - (a+5)*50) , (int)(500 - (b+5)*50)] = 255
        #indeces.append((int)((a+5)*50*50 + (b+5)*5))
        print(item)
        i += 3

    #print("\n\n\n")
    #print(indeces)
    #print(type(indeces))
    #teste = [2,3,4]
    #blank_image[indeces] = 255
    np.transpose(blank_image)
    print("\n\n\n")
    cv2.imshow('image',blank_image)
    cv2.waitKey(1)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/sensor/hokuyo", HokuyoReading, callback)

    rospy.spin()

print("\nL\nA\nU\nN\nC\nH\nE\nD\n")

listener()