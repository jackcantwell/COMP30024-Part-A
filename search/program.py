# COMP30024 Artificial Intelligence, Semester 1 2025
# Project Part A: Single Player Freckers

from .core import CellState, Coord, Direction, MoveAction
from .utils import render_board
import heapq
import math

# Board is known to be 8x8
BOARD_N = 8

# Possible directions that the red frog can move in
DIRECTIONS = [Direction.Down, Direction.DownRight, Direction.DownLeft,
              Direction.Right, Direction.Left]

# In this problem, from any given node, we assume that the shortest path can
# be achieved by consecutively hopping over blue frogs to the bottom row, which is considered
# one move. We know that if the red frog is on an even row, it can only "hop" to row six
# and then will still need at least one more move to reach the goal state. Whereas 
# on an odd row there is a possibility that the red frog could "hop" directly to the goal state.
# This is admissible. We prioritise even rows in our heuristic based on this logic.
def heuristic(redRow):
    if (redRow % 2 == 0):
        return 2
    else: 
        return 1
    

# Find the initial coordinate of the red frog
def find_initial_red(board):
    for coord, state in board.items():
        if state == CellState.RED:
            return coord

# Check if the red frog is in the bottom row (goal state)
def is_goal(pos, board):
    if pos.r == (BOARD_N-1):
        return True

# Obtain all possible future positions based on the current position
# Returns a list of tuples, each with a possible move, the resulting board
# and the future position of the red frog on that board.
def get_neighbours(board, pos):

    # Contains tuples of move, nextBoard, and nextPos
    neighbours = [] 

    # Tries each direction in order
    for direction in DIRECTIONS:
            # Will be none, none if this direction yields no possible outcome
            nextBoard, nextPos = apply_move(board, pos, direction)

            # If the move is a move, add it as a possible neighbour
            if nextBoard and nextPos:

                # Adds this move to neighbours assuming no futher hops
                move = MoveAction(pos, direction)
                neighbours.append((move, nextBoard, nextPos))

                # Check if the move we have just made is jumping over a blue
                if ((nextPos.r-pos.r) > 1 or (nextPos.c - pos.c) > 1):
                    # Check all further neighbours after the hop, recursively
                    # There can be more than 2 hops
                    multHops = get_neighbours(nextBoard, nextPos)
                    for hopMove, hopNextBoard, hopNextPos in multHops:
                        # Checks for valid subsequent moves - must be another "blue hop"
                        if ((hopNextPos.r - nextPos.r) > 1 or (hopNextPos.c - nextPos.c) > 1):
                            # Combines the hop directions into one move
                            newDirs = [x for x in move.directions]+[x for x in hopMove.directions]
                            move = MoveAction(pos, newDirs)
                            # Our apply_move method operates by moving the red frog
                            # This ensures that we don't delete any lilypads which are along the hop path
                            hopNextBoard[nextPos] = CellState.LILY_PAD
                            neighbours.append((move, hopNextBoard, hopNextPos))
                
    return neighbours

# Attempts to move the red frog in the given direction
# Returns a None tuple if the move is invalid 
# Returns a tuple of the next board and red frog position if the move is valid
def apply_move(board, pos, direction):
    
    nextBoard = board.copy()

    # The red frog is moving off the current pos, so delete the cell state
    if nextBoard[pos] == CellState.RED:
        del nextBoard[pos]

    # Calculate the next position of the red frog
    nextR = pos.r+direction.r
    nextC = pos.c+direction.c

    # Check if the future position is within the boundaries of the board
    if not (0 <= nextC <= (BOARD_N-1)) or not (0 <= nextR <= (BOARD_N-1)):  
        return (None, None)
    
    # Wrap the next position in a coordinate (given it is valid)
    nextPos = Coord(nextR, nextC)

    # Check if the next position is an empty CellState (e.g., not a lilypad nor blue frog)
    if nextPos not in nextBoard:
        return (None, None)

    # If the next position is a lilypad, it is a valid move
    elif nextBoard[nextPos] == CellState.LILY_PAD:
        nextBoard[nextPos] = CellState.RED
        return (nextBoard, nextPos)
    
    # If the next position is a blue frog, check if the red frog can jump over it 
    elif nextBoard[nextPos] == CellState.BLUE:
        # Check that we are in the spot where the red frog was removed
        # and not just jumping over several blue frogs 
        if (pos not in nextBoard):
            # Recursively call this function, checking the cell on the "other side" of the blue frog
            # if there is a lilypad, it will return that position, if not, None

            return apply_move(nextBoard, nextPos, direction)
        else:
            return (None, None)

def search(
    board: dict[Coord, CellState]
) -> list[MoveAction] | None:
    """
    This is the entry point for your submission. You should modify this
    function to solve the search problem discussed in the Part A specification.
    See `core.py` for information on the types being used here.

    Parameters:
        `board`: a dictionary representing the initial board state, mapping
            coordinates to "player colours". The keys are `Coord` instances,
            and the values are `CellState` instances which can be one of
            `CellState.RED`, `CellState.BLUE`, or `CellState.LILY_PAD`.
    
    Returns:
        A list of "move actions" as MoveAction instances, or `None` if no
        solution is possible.
    """

    # Prints out a board state in a human-readable format. ansi allows for colouring
    #print(render_board(board, ansi=True))
    
    # Our code:

    # Initial key info about the red frog
    initialPos = find_initial_red(board) 
    initialSteps = 0
    initialHeuristic = heuristic(initialPos.r)
    initialCost = initialSteps + initialHeuristic
    initialPath = []
    nodeCounter = 1

    # Initialise the Priority Queue (PQ)
    PQ = []

    # Python's heapq can sort a queue by a set, it trys to sort on the first
    # element of the set, if that fails, it moves on to the second, and so on.
    # Because we are only concerned about the A* cost, we have "nodeCounter" 
    # which acts as a unique ID for each node and avoids sorting errors.

    # The PQ contains: the cost (steps + heuristic), the steps to get there,
    # the red frog position, the board at that state, and the path to get there

    heapq.heappush(PQ, (initialCost, nodeCounter, initialSteps, initialPos, 
                        board, initialPath))

    # Store the nodes visited to avoid unnecessarily expanding the same node multiple times
    visited = set(initialPos)

    while PQ:
        # Expand the node 
        cost, counter, steps, pos, board, path = heapq.heappop(PQ)
        
        # if the node currently being expanded has previously been expanded
        # no need for unnecessary repitition. 
        if (pos in visited):
            continue
        else:
            visited.add(pos)

        # Checks if the goal state has been met (check on expansion)
        if is_goal(pos, board):
            #print(render_board(board, ansi=True))
            return path
        
        # Iterates through potential moves, obtaining the result of making said move
        for move, nextBoard, nextPos in get_neighbours(board, pos):
            # Increment step count, and calculate the new cost based on that step
            # count and the estimated heuristic (steps have uniform cost)
            nextSteps = steps + 1
            nextCost = nextSteps + heuristic(nextPos.r)

            # Add this to the PQ as generated nodes
            nodeCounter += 1
            heapq.heappush(PQ, (nextCost, nodeCounter, nextSteps, nextPos, nextBoard, 
                                path +[move]))
            
    # If PQ is empty it means no solution
    return None