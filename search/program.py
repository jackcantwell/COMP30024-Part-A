# COMP30024 Artificial Intelligence, Semester 1 2025
# Project Part A: Single Player Freckers

from .core import CellState, Coord, Direction, MoveAction
from .utils import render_board
import heapq
import math

BOARD_N = 8

# Possible directions that the red frog can move in
DIRECTIONS = [Direction.Down, Direction.DownRight, Direction.DownLeft,
              Direction.Right, Direction.Left]

# The heuristic used assumes the shortest possible path would be to consecutively
# hop over blue monkeys all the way to the bottom row, this is an admissable
# heuristic
def heuristic(redRow, board):
    return (math.ceil(((BOARD_N-1) - redRow)/2))
    

# Find the initial Coord of the red frog
def find_initial_red(board):
    for coord, state in board.items():
        if state == CellState.RED:
            return coord

# Check if the red frogs position is in the bottom row
def is_goal(pos, board):
    if pos.r == (BOARD_N-1):
        return True

# Obtain the possible future nodes based on the current node
# Returns the move, the future board, and the future position of the red frog
def get_neighbours(board, pos):

    # Contains tuples of move, nextBoard, and nextPos
    neighbours = [] 

    for direction in DIRECTIONS:
            nextBoard, nextPos = apply_move(board, pos, direction)
            move = MoveAction(pos, direction)

            # If the move is a valid move, add it as a possible neighbour
            if nextBoard is not None and nextPos is not None:
                neighbours.append((move, nextBoard, nextPos))
                # print(move, nextPos)
                # print(render_board(nextBoard, ansi=True))  
    return neighbours

# Attempts to move the red frog in the given direction
# Either returns a None tuple if the move is invalid or a tuple of the next board 
# and red frog position if the move is valid
def apply_move(board, pos, direction):
    #print(direction)
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

    # print(nextPos.r, nextPos.c)

    # Check if the next position is an empty CellState (e.g., not a lilypad nor blue frog)
    if nextPos not in nextBoard:
        return (None, None)

    # If the next position is a lilypad, it is a valid move
    elif nextBoard[nextPos] == CellState.LILY_PAD:
        nextBoard[nextPos] = CellState.RED
        #print("OPTION")
        #print(render_board(nextBoard, ansi=True))
        return (nextBoard, nextPos)
    
    # If the next position is a blue frog, check if the red frog can jump over it 
    elif nextBoard[nextPos] == CellState.BLUE:
        # Check that we are in the spot where the red frog was removed
        # and not just jumping over several blue frogs 
        if (pos not in nextBoard):
            # Simply recall this function and check the CellState after the frog
            # if there is a lilypad, it will just return that position, if not
            # it returns None
            #print("Blue Check")
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

    # The render_board() function is handy for debugging. It will print out a
    # board state in a human-readable format. If your terminal supports ANSI
    # codes, set the `ansi` flag to True to print a colour-coded version!
    print(render_board(board, ansi=True))
    

    # PLAN:
    # We need a priority queue for the search. Each time the node
    # is popped, we expand the possible solutions (there can only be 5 at most) 
    # (branching factor = 5) and run them through the heuristic, 
    # then respectively place them in the priority queue in order. the value 
    # is based on the amount of moves taken to get there + the shortest 
    # possible amount of moves that could result in a solution (heuristic)
    # (aka if the frog hopped its way over frogs to the bottom row)

    # Implement PQ using min-heap?

    # For every node in the PQ, we need to store: the board of that state, 
    # location of red, the path it took to get there (as a list of MoveActions), 
    # the amount of steps it took to get to that state, and the final A* cost 
    # (which is the steps to get there  + the heuristic estimate to get
    # to the goal)

    # Also need to store visited nodes, so they aren't re-visited 

    # Note: the heuristic value doesn't actually need to be stored :P


    # So from start to finish: 
    # Locate the initial position of the red frog (DONE)
    # Initialise the PQ
    # Add that initial position to the PQ with its value as
    # the total cost so far (0) plus the estimated heuristic (which always starts as
    # 4) assuming a 8x8 grid
    # Initialise the visited set
    # Then while the PQ is not empty:
    #   pop the first node from the PQ, 
    #   is it the goal? if so END
    #   add to visited set
    #   expand all it's neighbours
    #   if NOT visited, and chuck em in the PQ based on their cost (1) +
    # heuristic


    # Our code:

    # Initial key info about the red frog
    initialPos = find_initial_red(board) 
    initialSteps = 0
    initialHeuristic = heuristic(initialPos.r, board)
    initialCost = initialSteps + initialHeuristic
    initialPath = []

    nodeCounter = 1

    # Initialise the Priority Queue (PQ)
    PQ = []

    # Python's heapq automatically organizes based on the first element which is the cost
    # The PQ contains: the cost (steps + heuristic), the steps to get there,
    # the red frog position, the board at that state, and the path to get there

    heapq.heappush(PQ, (initialCost, nodeCounter, initialSteps, initialPos, 
                        board, initialPath))
    
    #print(PQ)
    
    # Store the nodes visited to ignore duplicates
    # Given that from at a node, the heuristic is unchanging
    # (i.e if you visit the same node twice - the heuristic is the same value)
    # We assume that the first time a node is visited will be the cheapest
    # path TO that node - and therefore the TOTAL cost is minimised through 
    # that node the FIRST time we expand it... hence, there is no need to explore
    # a given spot on the board twice.

    visited = set(initialPos)

    while PQ:
        # Expand the node, 
        cost, counter, steps, pos, board, path = heapq.heappop(PQ)
        #print(path)
        #print("CHOSEN OPTION")
        #print(render_board(board, ansi=True))
        
        # if the node being expanded has already been visited
        # no need to unnecessarily repeat steps.
        if (pos in visited):
            continue
        else:
            visited.add(pos)

        # Is this node at the bottom row?
        if is_goal(pos, board):
            return path
        
        # For every neighbour, obtain the MoveAction, the future board,
        # and the future position of the red frog
        for move, nextBoard, nextPos in get_neighbours(board, pos):
            print(nextPos)
            # Increment step count, and calculate the new cost based on that step
            # count and the estimated heuristic
            nextSteps = steps + 1
            nextCost = nextSteps + heuristic(nextPos.r, board)

            # Add this to the PQ as generated nodes
            ## ISSUE HERE - When heapq adds next value to the PQ, nextCost is used 
            ## If nextCost is equal, it moves on, comparing then to nextSteps, and so on
            ## This is not an issue until we try to compare boards - in which case 
            ## you are NOT able to compare two dictionaries - and we get an error
            ## Proposed solution, have a "nodeNumber" variable (like an ID - unique
            ## to every node) which is the second variable after "nextCost"
            ## this way we will simply put the nodes in order based on when they appear
            ## **UNLESS THE COSTS ARE DIFFERENT

            nodeCounter += 1
            heapq.heappush(PQ, (nextCost, nodeCounter, nextSteps, nextPos, nextBoard, 
                                path + [move]))
    # If PQ is empty it means no solution
    return None

