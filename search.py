# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
# 
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains search functions.
"""
import heapq

# Search should return the path and the number of states explored.
# The path should be a list of tuples in the form (alpha, beta, gamma) that correspond
# to the positions of the path taken by your search algorithm.
# Number of states explored should be a number.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,greedy,astar)
# You may need to slight change your previous search functions in MP1 since this is 3-d maze

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "dfs": dfs,
        "greedy": greedy,
        "astar": astar,
    }.get(searchMethod, [])(maze)

def bfs(maze):
    # TODO: Write your code here    
    return [], 0

def dfs(maze):
    # TODO: Write your code here    
    return [], 0

def greedy(maze):
    # TODO: Write your code here    
    return [], 0

def astar(maze):
    start = maze.getStart()
    goals = set(maze.getObjectives())
    numArms = len(start)
    frontier = []
    explored = set([])
    costLookUp = {}

    startNode = NodeAstar(None, start, 0, 0, goals)
    
    costLookUp[startNode.state] = 0
    heapq.heappush(frontier, startNode)
    
    currentNode = None
    
    while (len(frontier) > 0):
        currentNode = heapq.heappop(frontier)
        explored.add(currentNode)

        if(currentNode.is_goal()):
            break

        neighbors = None
        if(numArms == 1):
            alpha = currentNode.state
            neighbors = maze.getNeighbors(alpha)
        elif(numArms == 2):
            alpha, beta = (currentNode.state)
            neighbors = maze.getNeighbors(alpha, beta)
        elif(numArms == 3):
            alpha, beta, gamma = (currentNode.state)
            neighbors = maze.getNeighbors(alpha, beta, gamma)
        
        for neighbor in neighbors:
            node = currentNode.add_child(neighbor)
            if node not in explored and node not in frontier:
                heapq.heappush(frontier, node)
                costLookUp[node.state] = node.total_cost
            else:
               # adds state back in again this new path was cheaper
                currentCost = node.total_cost
                oldCost = costLookUp[node.state]
                if(currentCost < oldCost):
                    costLookUp[node.state] = currentCost
                    if node in explored:
                        explored.remove(node)
                        frontier.append(node)
                    else:
                        frontier.append(node)
    # only returns the path cost, which is needed for precomputations
    path = currentNode.get_solution()
    return path, len(explored)

class NodeAstar:
    ### only works for start position to goal position
    ### state is the current position as tuple (row, col)
    ### path cost is path cost from start to this state
    def __init__(self, parent, state, path_cost, estimated_cost, goals):
        self.parent = parent
        self.state = state
        self.goals = goals
        self.path_cost = path_cost
        self.estimated_cost = estimated_cost
        self.total_cost = self.path_cost + self.estimated_cost
    
    def is_goal(self):
        return self.state in self.goals

    def add_child(self, postion):
        state = postion
        path_cost = self.path_cost + 1
        estimated_cost = self.get_estimated_cost()
        node = NodeAstar(self, state, path_cost, estimated_cost, self.goals)
        return node
    
    def get_estimated_cost(self):
        distances = []
        start = self.state
        for goal in self.goals:
            distance = self.get_manhattan_distance(start, goal)
            distances.append(distance)
        distances.sort(reverse=True)

        min_dist = distances.pop()
        return min_dist

    def get_manhattan_distance(self, start, end):
        if(len(start) == 1):
            return start[0] - end[0]
        if(len(start) == 2):
            delta_row = start[0] - end[0]
            delta_col = start[1] - end[1]
            distance = abs(delta_row) + abs(delta_col)
            return distance
        else:
            delta_row = start[0] - end[0]
            delta_col = start[1] - end[1]
            delta_depth = start[2] - end[2]
            distance = abs(delta_row) + abs(delta_col) + abs(delta_depth)
            return distance

    def get_solution(self):
        current_node = self
        path = []
        current_position = current_node.state
        
        # stops at start node which has parent set to None
        while not (current_node.parent is None):
            path.append(current_position)
            current_node = current_node.parent
            current_position = current_node.state
        # appends the starts node position
        path.append(current_position)
        
        path.reverse()
        return path

    def __lt__(self, other):
        if(self.total_cost == other.total_cost):
            return self.path_cost > other.path_cost
        return self.total_cost < other.total_cost

    def __eq__(self, other):
        return self.state == other.state

    def __hash__(self):
        return hash(self.state)