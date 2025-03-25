# COMP30024 Artificial Intelligence, Semester 1 2025
# Project Part A: Single Player Freckers

from .core import CellState, Coord, Direction, MoveAction
from .utils import render_board
import heapq
import math


DIRECTIONS = [Direction.Down, Direction.DownRight, Direction.DownLeft,
              Direction.Left, Direction.Right]
# The heuristic used assumes the shortest possible path would be to consecutively
# hop over blue monkeys all the way to the bottom row, this is an admissable
# heuristic
def heuristic(redRow, bottomRow):
    return (math.ceil((bottomRow - redRow)/2))
    

def find_initial_red(board):
    for coord, state in board.items():
        if state == CellState.RED:
            return coord

def is_goal(pos, bottomRow):
    if pos.r == bottomRow:
        return True

def get_neighbours(board, pos):

    # Contains a tuple of move, nextBoard, nextPos
    neighbours = [] 
    for direction in DIRECTIONS:
            nextBoard, nextPos = apply_move(board, pos, direction)
            move = MoveAction(pos, direction)
            if nextBoard is not None and nextPos is not None:
                neighbours.append((move, nextBoard, nextPos))
                print(move, nextPos)
                print(render_board(nextBoard, ansi=True))  
    return neighbours

def apply_move(board, pos, direction):
    nextBoard = board.copy()

    if nextBoard[pos] == CellState.RED:
        del nextBoard[pos]

    nextPos = Coord(pos.r+direction.r, pos.c+direction.c)

    # print(nextPos.r, nextPos.c)
    # print(render_board(nextBoard, ansi=True))


    if nextPos not in nextBoard:
        return (None, None)
    
    elif nextBoard[nextPos] == CellState.LILY_PAD:
        nextBoard[nextPos] = CellState.RED
        return (nextBoard, nextPos)
    
    elif nextBoard[nextPos] == CellState.BLUE:
        # jumpPos = Coord(nextPos.r + direction.r, nextPos.c + direction.c)
        
        # if jumpPos in nextBoard and nextBoard[jumpPos] == CellState.LILY_PAD:
        #     nextBoard[jumpPos] = CellState.RED
        #     del nextBoard[nextPos]  # Remove the blue monkey
        #     return (nextBoard, jumpPos)
        if (pos not in nextBoard):
            return apply_move(nextBoard, nextPos, direction)

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
    # and run them through the heuristic, then respectively place them in 
    # the priority queue in order. the value is based on the amount of moves
    # taken to get there + the shortest possible amount of moves that could
    # result in a solution (aka if the frog hopped its way over frogs to the 
    # bottom row)

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

    # Find size of board (to determine the bottom row)
    bottomRow = len(set(coord.r for coord in board)) - 1

    # Initial key info about the red frog
    initialPos = find_initial_red(board) 
    initialSteps = 0
    initialHeuristic = heuristic(initialPos.r, bottomRow)
    initialCost = initialSteps + initialHeuristic
    initialPath = []


    # Initialise the Priority Queue (PQ)
    PQ = []

    # Python's heapq automatically organizes based on the first element which is the cost
    # The PQ contains: the cost (steps + heuristic), the steps to get there,
    # the red frog position, the board at that state, and the path to get there
    heapq.heappush(PQ, (initialCost, initialSteps, initialPos, 
                        board, initialPath))
    
    # Store the nodes visited to ignore duplicates
    # visited = set()

    while PQ:
        cost, steps, pos, board, path = heapq.heappop(PQ)
        print(f"cost: {cost}")
        if is_goal(pos, bottomRow):
            return path
        
        # For every neighbour, obtain the MoveAction, the future board,
        # and the future position of the red frog
        for move, nextBoard, nextPos in get_neighbours(board, pos):
            # Increment step count, and calculate the new cost based on that step
            # count and the estimated heuristic
            nextSteps = steps + 1
            nextCost = nextSteps + heuristic(nextPos.r, bottomRow)
            heapq.heappush(PQ, (nextCost, nextSteps, nextPos, nextBoard, 
                                path + [move]))
    # If PQ is empty it means no solution
    return None


    # ignore this:
    return [
        MoveAction(Coord(0, 5), [Direction.Down]),
        MoveAction(Coord(1, 5), [Direction.DownLeft]),
        MoveAction(Coord(3, 3), [Direction.Left]),
        MoveAction(Coord(3, 2), [Direction.Down, Direction.Right]),
        MoveAction(Coord(5, 4), [Direction.Down]),
        MoveAction(Coord(6, 4), [Direction.Down]),
    ]
