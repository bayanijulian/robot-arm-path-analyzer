
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
    if(arm.getNumArmLinks() == 3): 
        return transformToMazeFor3Arms(arm, goals, obstacles, window, granularity)
    if(arm.getNumArmLinks() == 1):
        return transformToMazeFor1Arm(arm, goals, obstacles, window, granularity)

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
            armEnd = arm.getEnd()

            isWallFirstArm = doesArmTouchObstacles([armPos[0]], obstacles) or not isArmWithinWindow([armPos[0]], window)
            if(isWallFirstArm):
                for y in range(cols):
                    row.append(WALL_CHAR)
                map.append(row)
                break
            
            isWall = doesArmTouchObstacles(armPos, obstacles) or not isArmWithinWindow(armPos, window)
            if isWall:
                row.append(WALL_CHAR)
                continue

            doesGoThrough = not doesArmTouchGoals(armEnd, goals) and doesArmTouchObstacles(armPos, goals)
            if doesGoThrough:
                row.append(WALL_CHAR)
                continue

            isObjective = doesArmTouchGoals(armEnd, goals)
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

def transformToMazeFor3Arms(arm, goals, obstacles, window, granularity):
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
    alphaLimits, betaLimits, gammaLimits = arm.getArmLimit()
    alphaMin, alphaMax = alphaLimits
    betaMin, betaMax = betaLimits
    gammaMin, gammaMax = gammaLimits

    rows = int(((alphaMax - alphaMin) / granularity) + 1)
    cols = int(((betaMax - betaMin) / granularity) + 1)
    depths = int(((gammaMax - gammaMin) / granularity) + 1)

    startAngles = arm.getArmAngle()
    
    map = []
    # subtracts one because it will just add one in first loop
    alpha = alphaMin - granularity
    for x in range(rows): # all alpha values
        row = []
        alpha += granularity
        # subtracts one because it will just add one in first loop
        arm.setArmAngle((alpha, betaMin, gammaMin))
        armPos = arm.getArmPos()
        isWallFirstArm = doesArmTouchObstacles([armPos[0]], obstacles) or not isArmWithinWindow([armPos[0]], window)
        if(isWallFirstArm):
            depth = []
            for y in range(cols):
                for z in range(depths):
                    depth.append(WALL_CHAR)
                row.append(depth)
            map.append(row)
            continue

        beta = betaMin - granularity
        for y in range(cols): # all beta values at the current alpha
            depth = []
            beta += granularity
            gamma = gammaMin - granularity

            arm.setArmAngle((alpha, beta, gammaMin))
            armPos = arm.getArmPos()
            
            isWallSecondArm = doesArmTouchObstacles([armPos[0]], obstacles) or not isArmWithinWindow([armPos[0]], window)
            if(isWallSecondArm):
                depth = []
                for z in range(depths):
                    depth.append(WALL_CHAR)
                row.append(depth)    
                continue
            for z in range(depths):
                gamma += granularity

                arm.setArmAngle((alpha, beta, gamma))
                armPos = arm.getArmPos()
                armEnd = arm.getEnd()

                
            
                isWall = doesArmTouchObstacles(armPos, obstacles) or not isArmWithinWindow(armPos, window)
                if isWall:
                    depth.append(WALL_CHAR)
                    continue

                doesGoThrough = not doesArmTouchGoals(armEnd, goals) and doesArmTouchObstacles(armPos, goals)
                if doesGoThrough:
                    depth.append(WALL_CHAR)
                    continue

                isObjective = doesArmTouchGoals(armEnd, goals)
                if isObjective:
                    depth.append(OBJECTIVE_CHAR)
                    continue

                # not objective or wall then free space    
                depth.append(SPACE_CHAR)
            row.append(depth)
        map.append(row)
    
    # transforms start angles to index in maze
    startIndexes = angleToIdx(startAngles, (alphaMin, betaMin, gammaMin), granularity)
    startAlpha, startBeta, startGamma = startIndexes
    # adds start to maze
    map[startAlpha][startBeta][startGamma] = START_CHAR

    maze = Maze(map, (alphaMin, betaMin, gammaMin), granularity)
    return maze

def transformToMazeFor1Arm(arm, goals, obstacles, window, granularity):
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
   
    alphaLimits = arm.getArmLimit()[0]
    alphaMin = alphaLimits[0]
    alphaMax = alphaLimits[1]
    
    rows = int(((alphaMax - alphaMin) / granularity) + 1)

    startAngles = arm.getArmAngle()
    
    map = []
    # subtracts one because it will just add one in first loop
    alpha = alphaMin - granularity
    for x in range(rows): # all alpha values
       
        alpha += granularity
        # subtracts one because it will just add one in first loop
       
        arm.setArmAngle((alpha,))
        armPos = arm.getArmPos()
        armEnd = arm.getEnd()

        isWall = doesArmTouchObstacles(armPos, obstacles) or not isArmWithinWindow(armPos, window)
        if isWall:
            map.append(WALL_CHAR)
            continue

        doesGoThrough = not doesArmTouchGoals(armEnd, goals) and doesArmTouchObstacles(armPos, goals)
        if doesGoThrough:
            map.append(WALL_CHAR)
            continue

        isObjective = doesArmTouchGoals(armEnd, goals)
        if isObjective:
            map.append(OBJECTIVE_CHAR)
            continue

        # not objective or wall then free space    
        map.append(SPACE_CHAR)
        
    
    # transforms start angles to index in maze
    startIndexes = angleToIdx(startAngles, (alphaMin,), granularity)
    startAlpha = startIndexes[0]
    # adds start to maze
    map[startAlpha] = START_CHAR

    maze = Maze(map, (alphaMin,), granularity)
    return maze