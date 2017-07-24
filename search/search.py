# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
import logging

logging.basicConfig(level = logging.DEBUG)

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  def isGoal(state):
    '''
    Returns the current state if it has reached the goal
    '''
    return problem.isGoalState(state)

  #Keep track of the frontier and of previously explored nodes
  initial_state = problem.getStartState()
  explored = set()

  stack = util.Stack()
  stack.push((initial_state, []))
  

  while not stack.isEmpty():
    frontier, path = stack.pop()
    explored.add(frontier)
    if isGoal(frontier):
      return path #Return path if the goal is reached

    successors = problem.getSuccessors(frontier)
    for successor, action, step_cost in successors:
      if successor not in explored:
        stack.push((successor, path + [action]))

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  def isGoal(state):
    '''
    Returns the current state if it has reached the goal
    '''
    return problem.isGoalState(state)

  #Keep track of the frontier and of previously explored nodes
  initial_state = problem.getStartState()
  explored = []

  queue = util.Queue()
  queue.push((initial_state, []))
  

  while not queue.isEmpty():
    frontier, path = queue.pop()
    logging.debug(frontier)
    if isGoal(frontier):
      return path # Return path if the goal is reached

    successors = problem.getSuccessors(frontier)
    for successor, action, step_cost in successors:
      if successor not in explored:
        explored.append(successor)
        queue.push((successor, path + [action]))

  util.raiseNotDefined()
      
def uniformCostSearch(problem):
  '''
  Search the node of least total cost first. 
  '''
  def isGoal(state):
    '''
    Returns the current state if it has reached the goal
    '''
    return problem.isGoalState(state)

  #Keep track of the frontier and of previously explored nodes
  initial_state = problem.getStartState()
  explored = []

  priority_queue = util.PriorityQueue()
  priority_queue.push((initial_state, []), 0)
  
  while not priority_queue.isEmpty():
    frontier, path = priority_queue.pop()
    if isGoal(frontier):
      return path #Return path if the goal is reached

    successors = problem.getSuccessors(frontier)
    for successor, action, step_cost in successors:
      if successor not in explored:
        explored.append(frontier)
        priority_queue.push((successor, path + [action]), step_cost)

  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic = nullHeuristic):
  """
  Search the node that has the lowest combined cost and heuristic first.
  """

  def isGoal(state):
    '''
    Returns the current state if it has reached the goal
    '''
    return problem.isGoalState(state)

  #Keep track of the frontier and of previously explored nodes
  initial_state = problem.getStartState()
  explored = []

  priority_queue = util.PriorityQueue()
  priority_queue.push((initial_state, []), 0)
  
  while not priority_queue.isEmpty():
    frontier, path = priority_queue.pop()
    if isGoal(frontier):
      return path #Return path if the goal is reached

    successors = problem.getSuccessors(frontier)
    for successor, action, step_cost in successors:
      if successor not in explored:
        explored.append(frontier)
        priority_queue.push((successor, path + [action]), step_cost + heuristic(successor, problem))
  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
