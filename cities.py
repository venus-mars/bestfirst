import heapq
from collections import defaultdict
from math import sqrt

# -------- Heuristic Function: Euclidean Distance ----------
def euclidean_distance(coord1, coord2):
    x1, y1 = coord1
    x2, y2 = coord2
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# -------- Best-First Search Algorithm ----------
# -------- Best-First Search Algorithm ----------
def best_first_search(graph, coordinates, start, goal):
    visited = set()  # A set to track the cities we have already visited
    heap = []  # A priority queue (min-heap) for exploring cities with the lowest heuristic value first
    parent = {}  # A dictionary to keep track of the parent city for path reconstruction

    # Push the start city into the heap with its heuristic distance to the goal
    # The priority value is the Euclidean distance from the start city's coordinates to the goal's coordinates
    heapq.heappush(heap, (euclidean_distance(coordinates[start], coordinates[goal]), start))

    # Process cities until we find the goal or exhaust all possibilities
    while heap:
        # Pop the city with the lowest heuristic (Euclidean distance) from the heap
        _, current = heapq.heappop(heap)

        # If we've reached the goal city, reconstruct the path from start to goal
        if current == goal:
            path = []  # Initialize an empty list to store the path
            while current != start:  # Reconstruct the path by backtracking through parents
                path.append(current)  # Add the current city to the path
                current = parent[current]  # Move to the parent city
            path.append(start)  # Add the start city to the path
            path.reverse()  # Reverse the path to get it in the correct order from start to goal
            return path  # Return the reconstructed path

        # Skip cities that have already been visited
        if current in visited:
            continue
        visited.add(current)  # Mark the current city as visited

        # Explore each neighbor of the current city
        for neighbor in graph[current]:
            # If the neighbor has not been visited yet
            if neighbor not in visited:
                if neighbor not in parent:  # Track the parent only the first time we visit a neighbor
                    parent[neighbor] = current  # Set the current city as the parent of the neighbor
                # Calculate the priority (Euclidean distance from neighbor to the goal)
                priority = euclidean_distance(coordinates[neighbor], coordinates[goal])
                # Push the neighbor into the heap with its priority (heuristic value)
                heapq.heappush(heap, (priority, neighbor))

    # If no path to the goal is found, return None
    return None  # Goal is not reachable from the start city


# -------- Coordinate Assignment to Nodes ----------
def assign_coordinates(graph):
    coordinates = dict()
    visited = set()
    queue = []

    start_city = next(iter(graph))
    coordinates[start_city] = (0, 0)
    visited.add(start_city)
    queue.append(start_city)

    while queue:
        current = queue.pop(0)
        x, y = coordinates[current]
        dx_dy = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        dir_idx = 0

        for neighbor in graph[current]:
            if neighbor not in visited:
                dx, dy = dx_dy[dir_idx % 4]
                coordinates[neighbor] = (x + dx, y + dy)
                dir_idx += 1
                visited.add(neighbor)
                queue.append(neighbor)

    return coordinates


# -------- Read Graph from File ----------
def read_graph(filename):
    with open(filename, 'r') as f:
        lines = f.read().strip().splitlines()

    N = int(lines[0])  # Number of edges only
    graph = defaultdict(list)

    for i in range(1, N + 1):
        a, b = map(int, lines[i].split())
        graph[a].append(b)
        graph[b].append(a)

    return graph


# -------- Main Function ----------
def main():
    filename = 'inpcities.txt'
    graph = read_graph(filename)
    coordinates = assign_coordinates(graph)

    print("Graph loaded from file.")
    print("Enter city numbers to check if a path exists using Best-First Search.")

    while True:
        try:
            start = int(input("Start City (or -1 to quit): "))
            if start == -1:
                break
            goal = int(input("Goal City: "))
            path = best_first_search(graph, coordinates, start, goal)

            if path:
                print(f"Path found from city {start} to {goal}: {' -> '.join(map(str, path))}")
            else:
                print(f"City {goal} is unreachable from city {start}.")

        except ValueError:
            print("Invalid input. Please enter numeric city numbers.\n")

if __name__ == '__main__':
    main()
















'''
# Best-First Search (BEFS) Algorithm

## Theory:
Best-First Search (BEFS) is a search algorithm that explores nodes in a search space by selecting the most promising node based on a given heuristic function. The heuristic evaluates the "cost" or "distance" from the current node to the goal. In the context of this problem, the heuristic is the Euclidean distance between cities. BEFS uses a priority queue (min-heap) to always expand the most promising node first. It is commonly used in pathfinding and graph traversal problems where the goal is to reach a target node with an optimal or near-optimal path based on some evaluation function.

The core logic of this algorithm is that it doesn't guarantee the shortest path but aims to find a path to the goal by evaluating nodes based on their proximity to the goal. In this specific problem, the algorithm uses the Euclidean distance formula to determine the priority of cities to be visited. Once the goal is reached, the path is reconstructed using the parent pointers and returned.

### Euclidean Distance Formula:
The Euclidean distance between two points `(x1, y1)` and `(x2, y2)` is given by:




## Algorithm:

1. **Initialization**:
 - Create a set `visited` to track cities that have been explored.
 - Create a priority queue `heap` to store cities, prioritized by their Euclidean distance to the goal.
 - Create a dictionary `parent` to track the parent city of each city, allowing for path reconstruction.

2. **Push Start City**:
 - Compute the Euclidean distance between the start city and the goal city.
 - Push the start city into the heap along with its calculated priority (heuristic).

3. **Main Search Loop**:
 - While the heap is not empty:
   - Pop the city with the lowest priority (closest to the goal).
   - If this city is the goal city, proceed to reconstruct the path and return it.
   - Otherwise, mark the city as visited and explore its neighbors.
 
4. **Explore Neighbors**:
 - For each unvisited neighbor of the current city:
   - Calculate its Euclidean distance to the goal (priority).
   - Push the neighbor into the heap with its priority.
   - Set the current city as the parent of the neighbor to aid in path reconstruction.

5. **Path Reconstruction**:
 - If the goal city is found, backtrack using the `parent` dictionary to reconstruct the path from start to goal.
 - Return the reconstructed path.

6. **Termination**:
 - If the heap is exhausted and the goal is not found, return `None` (indicating the goal is unreachable).

### Time Complexity:
The time complexity of BEFS primarily depends on the number of nodes `N` and the number of edges `E`. In the worst case, each node will be processed once, and each edge will be examined at most twice (once for each endpoint). Therefore, the time complexity is O(N log N + E), where the logarithmic factor comes from the priority queue operations (heap insertion and extraction).



'''
