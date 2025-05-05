'''
103
426
758
'''


import heapq
import math

GOAL = [[1, 2, 3], [4, 5, 6], [7, 8, 0]]  # The goal state of the puzzle.
dx = [-1, 1, 0, 0]  # Row movements for up, down.
dy = [0, 0, -1, 1]  # Column movements for left, right.

# PuzzleNode class to represent a state in the puzzle with heuristic value and parent.
class PuzzleNode:
    def __init__(self, board, h, parent):
        self.board = [row[:] for row in board]  # Deep copy of the board.
        self.h = h  # Heuristic value (Euclidean distance).
        self.parent = parent  # Parent node to track the path.

    def __lt__(self, other):
        # Comparison function to use in priority queue: Nodes are prioritized by heuristic value (h).
        return self.h < other.h


def find_zero(board):
    """Find the position of the empty space (0) on the board."""
    for i in range(3):
        for j in range(3):
            if board[i][j] == 0:
                return i, j
    return -1, -1  # In case no empty space is found (shouldn't happen).


def is_valid(x, y):
    """Check if a position is within the bounds of the board."""
    return 0 <= x < 3 and 0 <= y < 3


def swap(board, x1, y1, x2, y2):
    """Swap two tiles on the board."""
    new_board = [row[:] for row in board]  # Create a copy to avoid modifying the original.
    new_board[x1][y1], new_board[x2][y2] = new_board[x2][y2], new_board[x1][y1]  # Swap tiles.
    return new_board


def calculate_heuristic(board):
    """Calculate the Euclidean distance heuristic for the current board state."""
    total = 0
    for i in range(3):
        for j in range(3):
            val = board[i][j]
            if val != 0:  # Ignore the empty tile.
                target_x = (val - 1) // 3  # Target position row.
                target_y = (val - 1) % 3   # Target position column.
                # Calculate Euclidean distance between current position and target.
                total += math.sqrt((i - target_x) ** 2 + (j - target_y) ** 2)
    return total


def print_board(board, step, heuristic):
    """Print the board state along with the current step number and heuristic value."""
    print(f"Step {step}:")
    for row in board:
        print(row)
    print(f"Heuristic: {heuristic}\n")


def solve_puzzle(initial_board):
    """
    Solve the 8-puzzle problem using Best First Search algorithm.
    The algorithm uses a priority queue to select the node with the lowest heuristic value.
    It explores the puzzle by moving tiles into the empty space (0) until the goal is reached.
    """
    visited = set()  # Set to track visited board states to avoid revisiting.
    open_list = []  # Priority queue (min-heap) for Best First Search.
    
    # Calculate the heuristic (Euclidean distance) for the initial board.
    initial_h = calculate_heuristic(initial_board)
    # Create the starting node with the initial board and heuristic value.
    start_node = PuzzleNode(initial_board, initial_h, None)
    
    # Push the starting node onto the priority queue.
    heapq.heappush(open_list, start_node)
    
    step = 1  # Counter to track the number of steps (iterations).

    # Main loop for Best First Search algorithm.
    while open_list:
        # Pop the node with the lowest heuristic value (best candidate).
        current_node = heapq.heappop(open_list)
        
        # Print the current board state and heuristic at this step.
        print_board(current_node.board, step, current_node.h)
        step += 1  # Increment step counter.

        # Check if the current board is the goal state.
        if current_node.board == GOAL:
            print("Puzzle Solved!")  # If goal reached, print a success message.
            return

        # Convert the board to a string and check if it's already visited.
        board_str = str(current_node.board)
        if board_str in visited:
            continue  # Skip this node if it's already been visited.
        visited.add(board_str)  # Mark the current board state as visited.

        # Find the position of the empty tile (0).
        zero_x, zero_y = find_zero(current_node.board)

        # Explore all 4 possible moves (up, down, left, right) from the current empty space.
        for i in range(4):
            new_x = zero_x + dx[i]  # Calculate the new row position.
            new_y = zero_y + dy[i]  # Calculate the new column position.

            # If the new position is valid (within board boundaries).
            if is_valid(new_x, new_y):
                # Generate a new board by swapping the empty tile with a neighboring tile.
                new_board = swap(current_node.board, zero_x, zero_y, new_x, new_y)
                new_board_str = str(new_board)  # Convert the new board to a string for comparison.
                
                # If the new board hasn't been visited yet.
                if new_board_str not in visited:
                    # Calculate the heuristic for the new board state.
                    h = calculate_heuristic(new_board)
                    # Add the new node to the priority queue (open list).
                    heapq.heappush(open_list, PuzzleNode(new_board, h, current_node))

    # If the open list is empty and the goal isn't found, print that no solution exists.
    print("No solution found.")


if __name__ == "__main__":
    initial_board = []  # List to store the user's input board.
    
    print("Enter the 8-puzzle board row-wise (use 0 for empty space):")
    # Get the initial board state from the user input.
    for _ in range(3):
        row = list(map(int, input().split()))  # Read a row of integers.
        initial_board.append(row)

    # Call the solve_puzzle function with the initial board.
    solve_puzzle(initial_board)




































'''

# Theory:

Best First Search (BFS) is a heuristic search algorithm that selects the most promising node to explore next based on its heuristic value. The heuristic function is used to estimate the cost from the current state to the goal state. BFS prioritizes the node with the best (lowest) heuristic value, meaning the algorithm explores the most promising path first. BFS is often used in puzzles or pathfinding problems where the goal is to find the shortest or optimal solution based on some criteria.

In this code, we solve the 8-puzzle problem using Best First Search. The 8-puzzle involves a 3x3 grid with tiles numbered from 1 to 8 and one empty space (represented by 0). The goal is to arrange the tiles in the correct order, with the empty space at the bottom-right corner. The heuristic function used is the Euclidean distance, which calculates the sum of the squared distances of each tile from its goal position.

# Algorithm:

1. Initialize the open list (priority queue) with the start node, which represents the initial state of the puzzle.
2. Set the goal state, which is the target arrangement of the puzzle tiles.
3. Define the heuristic function `calculateHeuristic()`, which calculates the Euclidean distance for each tile from its current position to its goal position.
4. Create a loop that continues until the open list is empty or the goal is reached:
   - Dequeue the node with the lowest heuristic value from the open list.
   - If the current state matches the goal state, print the solution path and exit.
   - Otherwise, generate successor nodes by making valid moves from the current state.
   - For each successor, calculate its heuristic value and add it to the open list if it hasn't been visited before.
5. If the open list is empty and the goal is not found, print "No solution found."

# Heuristic Calculation Formulae:

1. Goal Position Calculation (for tile N, except 0):
   - Target position (targetX, targetY) = ((N-1) // 3, (N-1) % 3), where N is the tile number.

2. Euclidean Distance for a tile at position (x, y) with goal position (targetX, targetY):
   - h = sqrt((x - targetX)^2 + (y - targetY)^2)

3. Total Heuristic (h-value) for the board state:
   - h = sum of Euclidean distances for all non-zero tiles in the board.

The priority queue ensures that nodes with the best heuristic values are expanded first, guiding the algorithm towards the solution efficiently.

'''
