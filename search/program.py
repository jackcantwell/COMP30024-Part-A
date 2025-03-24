# COMP30024 Artificial Intelligence, Semester 1 2025
# Project Part A: Single Player Freckers

from .core import CellState, Coord, Direction, MoveAction
from .utils import render_board
import heapq

def heuristic(board):
    return

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
    # Locate the initial position of the red frog
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

    
    return [
        MoveAction(Coord(0, 5), [Direction.Down]),
        MoveAction(Coord(1, 5), [Direction.DownLeft]),
        MoveAction(Coord(3, 3), [Direction.Left]),
        MoveAction(Coord(3, 2), [Direction.Down, Direction.Right]),
        MoveAction(Coord(5, 4), [Direction.Down]),
        MoveAction(Coord(6, 4), [Direction.Down]),
    ]
