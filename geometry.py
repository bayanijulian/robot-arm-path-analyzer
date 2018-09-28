# geometry.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains geometry functions that relate with Part1 in MP2.
"""

import math
import numpy as np
from const import *

def computeCoordinate(start, length, angle):
    """Compute the end cooridinate based on the given start position, length and angle.

        Args:
            start (tuple): base of the arm link. (x-coordinate, y-coordinate)
            length (int): length of the arm link
            angle (int): degree of the arm link from x-axis to couter-clockwise

        Return:
            End position of the arm link, (x-coordinate, y-coordinate)
    """
    startX, startY = start
    endX = startX + (math.cos(math.radians(angle)) * length)
    endY = startY - (math.sin(math.radians(angle)) * length)
    #print("angle, length are {}, {}".format(angle, length))
    #print("cos is {} and sine is {}".format(math.cos(angle), math.sin(angle)))
    #print("startX is {} and endX is {} .... startY is {} and endY is {}".format(startX, endX, startY, endY))
    return endX, endY

def doesArmTouchObstacles(armPos, obstacles):
    #if armPos[0][0] > 100:
    #    return True
    #print(armPos[0][0], armPos[0][1])
    #print(armPos[0][0][0] - 4)
    #shoulder_to_elbow_length = math.sqrt(math.pow((armPos[0][1][1] - armPos[0][0][1]),2) + math.pow((armPos[0][1][0] - armPos[0][0][0]),2))
    #print(shoulder_to_elbow_length)
    #elbow_to_hand_length = math.sqrt(math.pow((armPos[1][1][1] - armPos[1][0][1]),2) + math.pow((armPos[1][1][0] - armPos[1][0][0]),2))
    #print(elbow_to_hand_length)
    #print(armPos[1][0], armPos[1][1])

    """Determine whether the given arm links touch obstacles

        Args:
            armPos (list): start and end position of all arm links [(start, end)]
            obstacles (list): x-, y- coordinate and radius of obstacles [(x, y, r)]

        Return:
            True if touched. False it not.
    """
<<<<<<< HEAD
    print(len(obstacles))
    for arm in armPos:
        start, end = arm
        startX, startY = start
        endX, endY = end
        #print("start is {} and end is {}".format(start, end))
        for obstacle in obstacles:
            x, y, r = obstacle
            distance = distanceFromLineToPoint(startX, startY, endX, endY, x, y)
            if (distance <= r):
                #print("distance is {} and circle {},{} and r is {}, startX:{}, startY:{}, endX:{}, endY:{}".format(distance, x, y, r, startX, startY, endX, endY))
                #print("hit obstacle")
                return True
    #print("not hitting anything")
=======
>>>>>>> 13b2a9751870be0eaaf3ee38c3e118874036397e
    return False

# https://brilliant.org/wiki/dot-product-distance-between-point-and-a-line/
# formula for shortest distance from a point to a line
def distanceFromLineToPoint(startX, startY, endX, endY, pointX, pointY):
    # vertical line, so tallest point is endX and endY
    if (endX - startX == 0):
        deltaX = pointX - endX
        deltaY = pointY - endY
        distance = math.sqrt(math.pow(deltaX, 2) + math.pow(deltaY, 2))
        #print("distance from function is {} and circle x, y is {}, {}".format(distance, pointX, pointY))
        return distance
    # istance is 5.234924505375147 and circle 150,50 and r is 10, startX:150, startY:200, endX:153.4899496702501, endY:100.06091729809043
    # a = slope, switched y2 and y1 for different coordinate
    m = (startY - endY) / (endX - startX)
    a = m
    b = -1
    # c = -mx1 + y1
    c = ((m * -1) * startX ) + startY
    numerator = (a * pointX) + (b * pointY) + c
    denominator = math.sqrt(math.pow(a, 2) + math.pow(b, 2))
    distance = math.fabs(numerator) / denominator
    return distance

def doesArmTouchGoals(armEnd, goals):

    for i in goals:
        if i[2] >= (math.sqrt(((armEnd[1] - i[1]) ** 2) + (armEnd[0] - i[0]) **2)):
            return True
    #if math.sqrt(math.pow((armEnd[1][1] - armEnd[0][1]),2) + math.pow((armEnd[1][0] - armEnd[0][0]),2))
    #circle_equation = math.pow(x,2) + math.pow(y,2)
    #if armEnd[0] > 100.0:
    #    return True
    #center = np.array()
    """Determine whether the given arm links touch goals

        Args:
            armEnd (tuple): the arm tick position, (x-coordinate, y-coordinate)
            goals (list): x-, y- coordinate and radius of goals [(x, y, r)]

        Return:
            True if touched. False it not.
    """

    return False


def isArmWithinWindow(armPos, window):
    """Determine whether the given arm stays in the window

        Args:
            armPos (list): start and end position of all arm links [(start, end)]
            window (tuple): (width, height) of the window

        Return:
            True if all parts are in the window. False it not.
    """
    return True
