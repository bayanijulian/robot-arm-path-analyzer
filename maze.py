# maze.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Jongdeog Lee (jlee700@illinois.edu) and
#            Michael Abir (abir2@illinois.edu) on 09/12/2018
"""
This file contains the Maze class, which reads in a maze file and creates
a representation of the maze that is exposed through a simple interface.
"""

import copy
from const import *
from util import *

class Maze:
    def __init__(self, input_map, offsets, granularity):
        """Initializes the Maze object by reading the maze from a file

            Args:
                input_map (list): 2D array. Alpha is row and beta is column
                offsets (list): min value of alpha and beta
                granularity (int): unit of increasing and decreasing the joint angle
        """
        self.__start = None
        self.__objective = []

        self.offsets = offsets
        self.granularity = granularity

        if len(self.offsets) == 3:
            self.__dimensions = [len(input_map), len(input_map[0]), len(input_map[0][0])]
            self.__map = input_map

            for x in range(self.__dimensions[ALPHA]):
                for y in range(self.__dimensions[BETA]):
                    for z in range(self.__dimensions[GAMMA]):
                        if self.__map[x][y][z] == START_CHAR:
                            self.__start = idxToAngle((x, y, z), self.offsets, granularity)
                        elif self.__map[x][y][z] == OBJECTIVE_CHAR:
                            self.__objective.append(idxToAngle((x, y, z), self.offsets, granularity))
        elif len(self.offsets) == 2:
            self.__dimensions = [len(input_map), len(input_map[0])]        
            self.__map = input_map

            for x in range(self.__dimensions[ALPHA]):
                for y in range(self.__dimensions[BETA]):                
                    if self.__map[x][y] == START_CHAR:                    
                        self.__start = idxToAngle((x, y), self.offsets, granularity)
                    elif self.__map[x][y] == OBJECTIVE_CHAR:
                        self.__objective.append(idxToAngle((x, y), self.offsets, granularity))
        else:
            self.__dimensions = [len(input_map)]        
            self.__map = input_map

            for x in range(self.__dimensions[ALPHA]):
                if self.__map[x] == START_CHAR:                    
                        self.__start = idxToAngle((x,), self.offsets, granularity)
                elif self.__map[x] == OBJECTIVE_CHAR:
                        self.__objective.append(idxToAngle((x,), self.offsets, granularity))

        if not self.__start:
            print("Maze has no start")
            raise SystemExit

        if not self.__objective:
            print("Maze has no objectives")
            raise SystemExit

    def getChar(self, alpha, beta = 0, gamma = 0):
        # Get character for the given alpha and beta position

        if len(self.offsets) == 3:
            x, y, z = angleToIdx((alpha, beta, gamma), self.offsets, self.granularity)
            return self.__map[x][y][z]
        elif len(self.offsets) == 2:
            x, y = angleToIdx((alpha, beta), self.offsets, self.granularity)
            return self.__map[x][y]
        else:
            x = angleToIdx((alpha,), self.offsets, self.granularity)[0]
        return self.__map[x]

    def isWall(self, alpha, beta = 0, gamma = 0):
        # Returns True if the given position is the location of a wall
        return self.getChar(alpha, beta, gamma) == WALL_CHAR

    def isObjective(self, alpha, beta = 0, gamma = 0):
        # Rturns True if the given position is the location of an objective
        return self.getChar(alpha, beta, gamma) == OBJECTIVE_CHAR

    def getStart(self):
        # Returns the start position as a tuple of (beta, column)
        return self.__start

    def setStart(self, start):
        # Set the start position as a tuple of (beta, column)
        self.__start = start

    def getDimensions(self):
        # Returns the dimensions of the maze as a (row, column) tuple
        return self.__dimensions

    def getObjectives(self):
        # Returns the list of objective positions of the maze
        return copy.deepcopy(self.__objective)

    def setObjectives(self, objectives):
        # Set the list of objective positions of the maze
        self.__objective = objectives

    def isValidMove(self, alpha, beta = 0, gamma = 0):
        # Check if the agent can move into a specific beta and column
        if len(self.offsets) == 3:
            x, y, z = angleToIdx((alpha, beta, gamma), self.offsets, self.granularity)
            return x >= 0 and x < self.getDimensions()[ALPHA] and \
                   y >= 0 and y < self.getDimensions()[BETA] and \
                   z >= 0 and z < self.getDimensions()[GAMMA] and \
                   not self.isWall(alpha, beta, gamma)
        elif len(self.offsets) == 2:
            x, y = angleToIdx((alpha, beta), self.offsets, self.granularity)
            return x >= 0 and x < self.getDimensions()[ALPHA] and \
                   y >= 0 and y < self.getDimensions()[BETA] and \
                   not self.isWall(alpha, beta)
        else:
            x, = angleToIdx((alpha,), self.offsets, self.granularity)
        return x >= 0 and x < self.getDimensions()[ALPHA] and \
               not self.isWall(alpha)


    def getNeighbors(self, alpha, beta = 0, gamma = 0):
        # Returns list of neighboing squares that can be moved to from the given beta,gamma
        if len(self.offsets) == 3:
            possibleNeighbors = [
                (alpha + self.granularity, beta, gamma),
                (alpha - self.granularity, beta, gamma),
                (alpha, beta + self.granularity, gamma),
                (alpha, beta - self.granularity, gamma),
                (alpha, beta, gamma + self.granularity),
                (alpha, beta, gamma - self.granularity)
            ]
            neighbors = []
            for a, b, c in possibleNeighbors:
                if self.isValidMove(a,b,c):
                    neighbors.append((a,b,c))
            return neighbors
        elif len(self.offsets) == 2:
            possibleNeighbors = [
                (alpha + self.granularity, beta),
                (alpha - self.granularity, beta),
                (alpha, beta + self.granularity),
                (alpha, beta - self.granularity)
            ]
            neighbors = []
            for a, b in possibleNeighbors:
                if self.isValidMove(a,b):
                    neighbors.append((a,b))
            return neighbors
        else:
            possibleNeighbors = [
                (alpha[0] + self.granularity),
                (alpha[0] - self.granularity)
            ]
            neighbors = []
            for a in possibleNeighbors:
                if self.isValidMove(a):
                    neighbors.append((a,))
        return neighbors

    def saveToFile(self, filename):
        # Export the maze to the text file
        outputMap = ""
        print("saving file")
        if len(self.offsets) == 3:
            for gamma in range(self.__dimensions[2]):
                for beta in range(self.__dimensions[1]):
                    for alpha in range(self.__dimensions[0]):
                        outputMap += self.__map[alpha][beta][gamma]
                    outputMap += "\n"
            with open(filename, 'w') as f:
                f.write(outputMap)

        elif len(self.offsets) == 2:
            print("saving file for 2")
            for beta in range(self.__dimensions[1]):
                for alpha in range(self.__dimensions[0]):
                    outputMap += self.__map[alpha][beta]
                outputMap += "\n"
            with open(filename, 'w') as f:
                f.write(outputMap)

        else:
            for alpha in range(self.__dimensions[0]):
                outputMap += self.__map[alpha]
            outputMap += "\n"

            with open(filename, 'w') as f:
                f.write(outputMap)

        return True
