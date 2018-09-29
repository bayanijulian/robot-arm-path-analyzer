
# transform.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
# 
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains the transform function that converts the robot arm map
to the maze.
"""
import copy
from arm import Arm
from maze import Maze
from search import *
from geometry import *
from const import *
from util import *

def transformToMaze(arm, goals, obstacles, window, granularity):
    """This function transforms the given 2D map to the maze in MP1.
    
        Args:
            arm (Arm): arm instance
            goals (list): [(x, y, r)] of goals
            obstacles (list): [(x, y, r)] of obstacles
            window (tuple): (width, height) of the window
            granularity (int): unit of increasing/decreasing degree for angles

        Return:
            Maze: the maze instance generated based on input arguments.

    """
    alphaLimits, betaLimits = arm.getArmLimit()
    alphaMin, alphaMax = alphaLimits
    betaMin, betaMax = betaLimits

    rows = int(((alphaMax - alphaMin) / granularity) + 1)
    cols = int(((betaMax - betaMin) / granularity) + 1)
    print("rows:{}, cols:{}".format(rows, cols))

    alpha = alphaMin - granularity
    
    startAngles = arm.getArmAngle()
    
    map = []

    for x in range(rows):
        row = []
        alpha += granularity
        beta = betaMin - granularity
        for y in range(cols):
            beta += granularity
            arm.setArmAngle((alpha, beta))
            armPos = arm.getArmPos()

            isWall = doesArmTouchObstacles(armPos, obstacles) or not isArmWithinWindow(armPos, window)
            if isWall:
                row.append(WALL_CHAR)
                continue

            isObjective = doesArmTouchObstacles(armPos, goals)
            if isObjective:
                row.append(OBJECTIVE_CHAR)
                continue
                
            row.append(SPACE_CHAR)
        map.append(row)
    
    
    startIndexes = angleToIdx(startAngles, (alphaMin, betaMin), granularity)
    startAlpha, startBeta = startIndexes
    map[startAlpha][startBeta] = START_CHAR
    maze = Maze(map, (alphaMin, betaMin), granularity)
    
    return maze