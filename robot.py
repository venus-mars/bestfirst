'''
we’ll simulate a robot searching its way through the grid from start to goal using Best-First Search, 
guided by Manhattan distance as the heuristic and see if the robot can reach that location. Path is hardcoded.
'''

import heapq

# -------- Heuristic Function: Manhattan Distance ----------
def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# -------- Best-First Search Function with Path Printing ----------
def best_first_search(graph, start, goal):
    visited = set()  # To keep track of visited nodes and avoid revisiting
    heap = []  # Priority queue (min-heap) to store nodes based on heuristic value

    # Push the start node into the heap with its Manhattan distance to the goal
    heapq.heappush(heap, (manhattan_distance(start, goal), start))

    steps = {start: 0}  # Dictionary to track number of steps taken to reach each node
    parent = {start: None}  # Dictionary to reconstruct the path later

    while heap:
        # Pop the node with the smallest heuristic value (most promising node)
        _, current = heapq.heappop(heap)

        # If the goal is reached, reconstruct and return the path and number of steps
        if current == goal:
            path = []
            while current is not None:
                path.append(current)  # Add node to path
                current = parent[current]  # Move to its parent
            path.reverse()  # Reverse to get path from start to goal
            return steps[goal], path

        # Skip if the current node has already been visited
        if current in visited:
            continue
        visited.add(current)  # Mark node as visited

        # Explore neighbors of the current node
        for neighbor in graph.get(current, []):
            if neighbor not in visited:
                # Update steps taken to reach neighbor
                steps[neighbor] = steps[current] + 1
                # Record the parent of the neighbor (for path reconstruction)
                parent[neighbor] = current
                # Calculate heuristic value (priority) for the neighbor
                priority = manhattan_distance(neighbor, goal)
                # Push neighbor into the heap
                heapq.heappush(heap, (priority, neighbor))

    # If the goal was not found, return failure result
    return -1, []

# -------- Graph Builder from Path Points ----------
def build_graph(points):
    graph = {}
    for i in range(len(points)):
        graph[points[i]] = []
        if i > 0:
            graph[points[i]].append(points[i-1])
        if i < len(points) - 1:
            graph[points[i]].append(points[i+1])
    return graph

# -------- Main Function ----------
def main():
    path_points = [
        (1, 0),
        (2, 1),
        (3, 2),
        (3, 3),
        (2, 3),
        (1, 4),
        (0, 4)
    ]

    graph = build_graph(path_points)
    start = path_points[0]

    print("Enter the target x and y coordinates (e.g. 0 4):")
    goal_x, goal_y = map(int, input().split())
    goal = (goal_x, goal_y)

    if goal not in graph:
        print("Goal not in path. Robot cannot reach that point.")
        return

    result, path = best_first_search(graph, start, goal)

    if result != -1:
        print(f"Robot reached target {goal} from {start} in {result} step(s) using Best-First Search.")
        print("Path followed:", ' → '.join(map(str, path)))
    else:
        print("Robot could not reach the target.")

if __name__ == '__main__':
    main()



























'''
THEORY:
This program simulates a robot navigating a grid from a starting point to a target 
using Best-First Search (BFS) guided by Manhattan distance as a heuristic. 
The valid movement path is hardcoded as a list of coordinates, which is 
then converted into a graph using the `build_graph()` function. Each node (coordinate) is 
only connected to its previous and next neighbors, creating a path-like structure. Best-First Search uses a 
priority queue to always explore the node closest to 
the goal based on Manhattan distance (|x1 - x2| + |y1 - y2|). T
he robot can only reach the goal if the point exists in this predefined path.

APPLICATIONS:
- Path planning in AI and robotics
- Grid-based game AI
- Simulating intelligent navigation with limited movement

ALGORITHM:
1. Define the path as a list of (x, y) coordinates.
2. Convert this path to a graph where each point connects to its neighbors (using `build_graph()`).
3. Ask user to enter the goal coordinate.
4. If the goal exists in the graph:
   a. Use Best-First Search from start to goal.
   b. Use a priority queue ordered by Manhattan distance to guide search.
   c. Return the number of steps needed to reach the goal.
5. If goal is not in the path, print that the robot cannot reach it.

FORMULA:
Manhattan Distance = |x1 - x2| + |y1 - y2|
This is used as the heuristic in Best-First Search.
'''
