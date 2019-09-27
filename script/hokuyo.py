#!/usr/bin/env python3

import rospy
from rosi_defy.msg import HokuyoReading
import numpy as np
import cv2

def naive_line(r0, c0, r1, c1):
    # The algorithm below works fine if c1 >= c0 and c1-c0 >= abs(r1-r0).
    # If either of these cases are violated, do some switches.
    if abs(c1-c0) < abs(r1-r0):
        # Switch x and y, and switch again when returning.
        xx, yy, val = naive_line(c0, r0, c1, r1)
        return (yy, xx, val)

    # At this point we know that the distance in columns (x) is greater
    # than that in rows (y). Possibly one more switch if c0 > c1.
    if c0 > c1:
        return naive_line(r1, c1, r0, c0)

    # We write y as a function of x, because the slope is always <= 1
    # (in absolute value)
    x = np.arange(c0, c1+1, dtype=float)
    y = x * (r1-r0) / (c1-c0) + (c1*r0-c0*r1) / (c1-c0)

    valbot = np.floor(y)-y+1
    valtop = y-np.floor(y)

    return (np.concatenate((np.floor(y), np.floor(y)+1)).astype(int), np.concatenate((x,x)).astype(int),
            np.concatenate((valbot, valtop)))



def callback(data):
    blank_image = np.zeros((500,500), np.uint8)
    
    #np.concatenate((blank_image, naive_line(0,250,499,250)))
    dx = 499
    dy = 0
    for x in range(499):
        blank_image[x,250] = 120
    
    
    
    
    
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