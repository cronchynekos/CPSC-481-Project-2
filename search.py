# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import time
import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    states = util.Stack()
    cost = 0
    visited = []
    actions = []
    startingPosition = problem.getStartState()
    states.push((startingPosition, actions, cost))

    while not states.isEmpty():

        position, actions, cost = states.pop()

        if position not in visited:
            visited.append(position)
            
            if problem.isGoalState(position):
                return actions
            
            
            for item in problem.getSuccessors(position):
                states.push((item[0], actions + [item[1]], item[2]))
        


    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    states = util.Queue()
    cost = 0
    visited = []
    actions = []
    startingPosition = problem.getStartState()
    states.push((startingPosition, actions, cost))
    while not states.isEmpty():
        position, actions, cost = states.pop()
        if position not in visited:
            visited.append(position)
            if problem.isGoalState(position):
                return actions
            for item in problem.getSuccessors(position):
                states.push((item[0], actions + [item[1]], item[2]))
    return []

    # states = util.Queue()
    # cost = 0
    # visited = []
    # actions = []
    # startingPosition = problem.getStartState()
    # states.push((startingPosition, actions, cost))

    # while not states.isEmpty():
    #     print(states)
    #     position, actions, cost = states.pop()

    #     if position not in visited:
    #         visited.append(position)
            
    #         if problem.isGoalState(position):
    #             return actions
            
            
    #         for item in problem.getSuccessors(position):
    #             states.push((item[0], actions + [item[1]], item[2]))
    # util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    open = util.PriorityQueue()

    # starting position or root
    startingPosition = problem.getStartState() 
    
    startHeuristic = heuristic(startingPosition, problem)

    open.push((startingPosition, [], 0), startHeuristic)

    # a list of node we visited
    closed = []

    # what we need to return
    path = []

    # While open.isEmpty is not true
    while not open.isEmpty():
        coordinate, path, cost = open.pop()

        if problem.isGoalState(coordinate):
            return path

        if not coordinate in closed:
            closed.append(coordinate)

            for c_coordinate, direction, cost in problem.getSuccessors(coordinate):
                if not c_coordinate in closed:

                        actions = list(path)
                        actions += [direction]
                        costOfActions = problem.getCostOfActions(actions)
                        
                        get_heuristic = heuristic(c_coordinate, problem)
                        open.push((c_coordinate, actions, 1), costOfActions + get_heuristic)

    return []
    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
