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
    #print("startX:{}, startY:{}, endX:{}, endY:{}, length is {}".format(startX, startY, endX, endY, length))
    #print("angle, length are {}, {}".format(angle, length))
    #print("cos is {} and sine is {}".format(math.cos(angle), math.sin(angle)))
    #print("startX is {} and endX is {} .... startY is {} and endY is {}".format(startX, endX, startY, endY))
    
    return endX, endY
# startX:150, startY:200, endX:160.45284632676535, endY:100.54781046317267, length is 100
def olddoesArmTouchObstacles(armPos, obstacles):
    #if armPos[0][0] > 100:
    #    return True
    #print(armPos[0][0], armPos[0][1])
    #print(armPos[0][0][0] - 4)
    #shoulder_to_elbow_length = math.sqrt(math.pow((armPos[0][1][1] - armPos[0][0][1]),2) + math.pow((armPos[0][1][0] - armPos[0][0][0]),2))
    #print(shoulder_to_elbow_length)
    #elbow_to_hand_length = math.sqrt(math.pow((armPos[1][1][1] - armPos[1][0][1]),2) + math.pow((armPos[1][1][0] - armPos[1][0][0]),2))
    #print(elbow_to_hand_length)
    #print(armPos[1][0], armPos[1][1])
    #return False
    """Determine whether the given arm links touch obstacles

        Args:
            armPos (list): start and end position of all arm links [(start, end)]
            obstacles (list): x-, y- coordinate and radius of obstacles [(x, y, r)]

        Return:
            True if touched. False it not.
    """
    
    for arm in armPos:
        start, end = arm
        startX, startY = start
        endX, endY = end
        outOfBounds = True
        #print("start is {} and end is {}".format(start, end))
        for obstacle in obstacles:
            
            x, y, r = obstacle
            
            #y - y1 = m (x - x1)
            startY = 200 - startY
            endY = 200 - endY
            y = 200 - y
            distance = distanceFromLineToPoint(startX, startY, endX, endY, x, y)
            intersectY = 0
            intersectX = 0
            if(startY == 0):
                continue
            
            if (endY - startY == 0):
                angle = math.radians(90)
                
                
            else:
                perpenidicular_slope = -1 * (endX - startX) / (endY - startY)
                angle = math.atan(perpenidicular_slope)
                print("slope is {}".format(perpenidicular_slope))
            intersectX = x - (math.cos(angle)*distance)

            arm_angle = math.atan(slope)

            print("start is {},{} and end is {},{}".format(startX, startY, endX, endY))
            print("angle is {}".format(math.degrees(angle)))
            
            
            intersectY = y - (math.sin(angle)*distance)
            
            if(endX - startX < 0):
                temp = startX
                startX = endX
                endX = temp
            if(endY- startY < 0):
                temp = startY
                startY = endY
                endY = temp
            
            #(intersectX >= startX) and (intersectX <= endX) and 
            if (intersectX >= startX) and (intersectX <= endX) and (intersectY <= endY) and (intersectY >= startY):
                outOfBounds = False
            else:
                print("miss X- {}, {} intersects at x = {} with arm range x {},{}".format(x, y, intersectX, startX, endX))
                print("miss Y- {}, {} intersects at y = {} with arm range y {},{}".format(x, y, intersectY, startY, endY))
            #if (endX - startX == 0):
            #outOfBounds = False
            if (distance <= r) and not outOfBounds:
                print("hit X- {}, {} intersects at x = {} with arm range x {},{}".format(x, y, intersectX, startX, endX))
                print("hit Y- {}, {} intersects at y = {} with arm range y {},{}".format(x, y, intersectY, startY, endY))
                #print("distance is {} and circle {},{} and r is {}, startX:{}, startY:{}, endX:{}, endY:{}".format(distance, x, y, r, startX, startY, endX, endY))
                #print("hit obstacle at {}, {}".format(x, y))
                return True
            else:
                print("missed distance for {}, {} intersects at x = {} with arm range x {},{} with dist {}".format(x, y, intersectX, startX, endX, distance))
    #print("not hitting anything")
    return False

def doesArmTouchObstacles(armPos, obstacles):
    for arm in armPos:
        start, end = arm
        #print("start: {} end: {}".format(start, end))
        startX, startY = start
        endX, endY = end

        startY = 200 - startY
        endY = 200 - endY

        start = (startX, startY)
        end = (endX, endY)
        for obstacle in obstacles:
            
            x, y, r = obstacle
            
            #y - y1 = m (x - x1)
            
            y = 200 - y
            
            center = (x, y)
            #print("end: {} does not collide with {}".format(end, center))
            flag = doesCircleLineCollide(start, end, center, r)
            if(flag):
                #print("start: {} collides with {}".format(start, center))
                return True
            
                
    return False

# https://brilliant.org/wiki/dot-product-distance-between-point-and-a-line/
# formula for shortest distance from a point to a line
def distanceFromLineToPoint(startX, startY, endX, endY, pointX, pointY):
    
    # vertical line, so tallest point is endX and endY
    if (endX - startX == 0):
        deltaX = pointX - endX
        deltaY = pointY - endY
        distance = math.sqrt(math.pow(deltaX, 2) + math.pow(deltaY, 2))
        #print("distance from function is {} and  y1, y2 is {}, {}".format(distance, startY, endY))
        return distance
    # istance is 5.234924505375147 and circle 150,50 and r is 10, startX:150, startY:200, endX:153.4899496702501, endY:100.06091729809043
    # a = slope, switched y2 and y1 for different coordinate
    m =  (endY - startY) / (endX - startX)
    a = m
    b = -1
    # c = -mx1 + y1
    c = ((m * -1) * startX) + startY
    numerator = (a * pointX) + (b * pointY) + c
    denominator = math.sqrt(math.pow(a, 2) + math.pow(b, 2))
    distance = math.fabs(numerator) / denominator
    return distance

def doesCircleLineCollide(start, end, center, radius):
    #print("start:{}, end:{}, center:{}, radius:{}".format(start, end, center, radius))
    line = tuple(np.subtract(end, start))
    #print("line is {}".format(line))
    lineToCircle = tuple(np.subtract(start, center))
    #print("lineCircle is {}".format(lineToCircle))
    a = np.dot(line,line)
    #print("a is {}".format(a))
    b = 2 * np.dot(lineToCircle,line)
    #print("b is {}".format(b))
    c = np.dot(lineToCircle,lineToCircle) - (radius * radius)
    #print("c is {}".format(c))
    discrim = (b*b) - (4 * a * c)
    if (discrim > 0):
        discrim = math.sqrt(discrim)
        t1 = ((b*-1) - discrim) / (2*a)
        t2 = ((b*-1) + discrim) / (2*a)
        if (t1 >= 0) and (t1 <= 1):
            return True
        if (t2 >= 0) and (t2 <= 0):
            return True
    return False
    

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
    for arm in armPos:
        start, end = arm
        startX, startY = start
        endX, endY = end
        width, height = window
        if startX < 0 or startY < 0 or endX > width or endY > height:
            return False
        if endX < 0 or endY < 0 or startX > width or startY > height:
            return False
    return True
