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
    # to add on to y, you need to subtract since y+ goes down from top left
    endY = startY - (math.sin(math.radians(angle)) * length)
    
    return endX, endY

def doesArmTouchObstacles(armPos, obstacles):
    for arm in armPos:
        start, end = arm       
        startX, startY = start
        endX, endY = end

        start = (startX, startY)
        end = (endX, endY)

        for obstacle in obstacles:
            x, y, r = obstacle
            center = (x, y)

            collision = doesCircleLineCollide(start, end, center, r)
            if(collision):
                return True

    return False

def doesCircleLineCollide(start, end, center, radius):
    # creates vectors
    line = tuple(np.subtract(end, start))
    lineToCircle = tuple(np.subtract(start, center))
    
    # quadractic equation using circle equation with parameterized equations for lines
    a = np.dot(line,line)
    b = 2 * np.dot(lineToCircle,line)
    c = np.dot(lineToCircle,lineToCircle) - (radius * radius)
    
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
    """Determine whether the given arm links touch goals

        Args:
            armEnd (tuple): the arm tick position, (x-coordinate, y-coordinate)
            goals (list): x-, y- coordinate and radius of goals [(x, y, r)]

        Return:
            True if touched. False it not.
    """
    for i in goals:
        if i[2] >= (math.sqrt(((armEnd[1] - i[1]) ** 2) + (armEnd[0] - i[0]) **2)):
            return True

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
        # checks to make sure each point of the arm is within bounds
        if startX < 0 or startY < 0 or endX > width or endY > height:
            return False
        if endX < 0 or endY < 0 or startX > width or startY > height:
            return False
    return True
