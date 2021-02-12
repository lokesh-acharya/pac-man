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

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """
    REVERSE_PUSH = False

    @staticmethod
    def reverse_push():
        SearchProblem.REVERSE_PUSH = not SearchProblem.REVERSE_PUSH

    @staticmethod
    def print_push():
        print(SearchProblem.REVERSE_PUSH)

    @staticmethod
    def get_push():
        return SearchProblem.REVERSE_PUSH

    def get_expanded(self):
        return self.__expanded

    def inc_expanded(self):
        self.__expanded += 1

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
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):

    start = problem.getStartState()
    frontier = util.Stack()
    explored = []
    parent = {start: None}
    frontier.push((start, 'null', 0))
    goal = None
    actions = []
    while not (frontier.isEmpty()):
        state = frontier.pop()
        # print("popped state: ", state)
        explored.append(state[0])
        # print("explored: ", explored)
        if problem.isGoalState(state[0]):
            goal = state[0]
            actions.insert(0, state[1])
            break
        children = problem.getSuccessors(state[0])
        # print("children: ", children)
        for child in children:
            # print("child: ", child)
            if child[0] not in explored:
                parent[child[0]] = state
                frontier.push(child)
    # print(goal)
    curr = goal
    # actions.insert(0, parent[goal][1])
    while parent[curr] is not None:
        # print("aaya!")
        actions.insert(0, parent[curr][1])
        # print(actions)
        curr = parent[curr][0]
    actions.pop(0)
    # print(actions)
    return actions

def breadthFirstSearch(problem):
    start = problem.getStartState()
    frontier = util.Queue()
    explored = []
    parent = {start: None}
    st = (start, 'null', 0)
    frontier.push(st)
    goal = None
    actions = []
    visited = set()
    visited.add(start)
    while not frontier.isEmpty():
        state = frontier.pop()
        if state[0] not in explored:
            explored.append(state[0])
            if problem.isGoalState(state[0]):
                goal = state[0]
                actions.insert(0, state[1])
                break
            children = problem.getSuccessors(state[0])
            for child in children:
                if child[0] not in explored and child[0] not in visited:
                    parent[child[0]] = state
                    frontier.push(child)
                    visited.add(child[0])
    curr = goal
    while parent[curr] is not None:
        actions.insert(0, parent[curr][1])
        curr = parent[curr][0]
    actions.pop(0)
    return actions

def uniformCostSearch(problem):
    start = problem.getStartState()
    frontier = util.PriorityQueue()
    explored = set()
    frontier.push((start, [], 0), 0)
    while not frontier.isEmpty():
        state = frontier.pop()
        csf = state[2]
        path = state[1]
        if problem.isGoalState(state[0]):
            return path
        if state[0] not in explored:
            explored.add(state[0])
            children = problem.getSuccessors(state[0])
            for child in children:
                if child[0] not in explored:
                    frontier.push((child[0], path + [child[1]], csf+child[2]), csf + child[2])
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    start = problem.getStartState()
    frontier = util.PriorityQueue()
    explored = set()
    frontier.push((start, [], 0), 0)
    while not frontier.isEmpty():
        state = frontier.pop()
        csf = state[2]
        path = state[1]
        if problem.isGoalState(state[0]):
            return path
        if state[0] not in explored:
            explored.add(state[0])
            children = problem.getSuccessors(state[0])
            for child in children:
                if child[0] not in explored:
                    frontier.push((child[0], path + [child[1]], csf + child[2]), csf + child[2] + heuristic(child[0], problem))
    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
reverse_push = SearchProblem.reverse_push
print_push = SearchProblem.print_push
