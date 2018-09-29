
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

    startAngles = arm.getArmAngle()
    
    map = []
    # subtracts one because it will just add one in first loop
    alpha = alphaMin - granularity
    for x in range(rows): # all alpha values
        row = []
        alpha += granularity
        # subtracts one because it will just add one in first loop
        beta = betaMin - granularity
        for y in range(cols): # all beta values at the current alpha
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

            # not objective or wall then free space    
            row.append(SPACE_CHAR)
        map.append(row)
    
    # transforms start angles to index in maze
    startIndexes = angleToIdx(startAngles, (alphaMin, betaMin), granularity)
    startAlpha, startBeta = startIndexes
    # adds start to maze
    map[startAlpha][startBeta] = START_CHAR

    maze = Maze(map, (alphaMin, betaMin), granularity)
    return maze