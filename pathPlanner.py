import random

"""
Heuristic function: Euclidean distance
This function calculates the Euclidean distance between two points (a and b) in a 2D grid.
The Euclidean distance is used as a heuristic to estimate the cost from the current node 
to the goal node.
"""
def heuristic(a, b):
    return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

"""
A* Path Planning Algorithm
This function implements the A* algorithm to find the shortest path from a given start 
position to an end position in a 2D grid. The grid consists of cells that can either 
be traversable (1) or blocked (0).

Parameters:
- grid: A 2D list representing the environment, where 1 indicates a traversable cell 
        and 0 indicates an obstacle.
- start: A tuple (x, y) representing the starting position.
- end: A tuple (x, y) representing the goal position.
- display_message: A function for printing messages, used for debugging or visualization.

Returns:
- A list of tuples representing the path from the start to the end position. 
  If no valid path is found, an empty list is returned.
"""
def do_a_star(grid, start, end, display_message):
    # Get the number of rows and columns in the grid
    COL = len(grid)  # Number of rows (height of the grid)
    ROW = len(grid[0])  # Number of columns (width of the grid)

    # Define the four possible movement directions: right, down, left, and up.
    # Each tuple represents a movement offset (dx, dy).
    movements = [(0, 1),(1, 0),(0, -1),(-1, 0)] 

    # Open set: stores nodes that need to be evaluated.
    # Each element is a tuple (f_score, g_score, node, parent):
    # - f_score: Estimated total cost (g_score + heuristic)
    # - g_score: Cost from the start node to the current node
    # - node: The current node (x, y)
    # - parent: The previous node in the path (used for backtracking)
    open_set = [(0, 0, start, None)]  

    # Dictionary to store the cost of the shortest known path to each node
    g_scores = {start: 0}

    # Dictionary to store the parent of each node for path reconstruction
    parents = {}

    # Generate a list of 10 random grid positions.
    # These are not part of the A* algorithm but may be used for debugging or visualization.
    path = []
    for i in range(1, 10):
        path.append((random.randint(0, COL-1), random.randint(0, ROW-1)))

    # Debug message to indicate that random path points have been created.
    display_message("Created random path points")
    display_message("Start location is " + str(start))

    """
    Main loop of the A* algorithm:
    - Find the node with the lowest f_score (best estimated cost to the goal).
    - Remove that node from the open set.
    - If the goal is reached, reconstruct and return the path.
    - Otherwise, explore its neighbors and update their costs.
    """
    while open_set:
        # Find the node with the lowest f_score (brute force search)
        current_f, current_g, current, parent = min(open_set, key=lambda x: x[0])
        open_set.remove((current_f, current_g, current, parent))

        # If the current node is the goal, reconstruct the path by backtracking
        if current == end:
            path = []
            while current in parents:
                path.append(current)
                current = parents[current]
            path.append(start)  # Add the start node
            path.reverse()  # Reverse the list to get the path from start to end

            # Print and display the reconstructed path
            print("Reconstructed Path: ", path)
            display_message("Path found!")
            return path  # Return the found path

        """
        Explore the four possible movement directions.
        - Compute the neighbor's coordinates.
        - Check if the neighbor is within bounds and not an obstacle.
        - Calculate the tentative cost to reach the neighbor.
        - If this new path is better, update the neighbor's cost and add it to the open set.
        """
        for dx, dy in movements:
            neighbor = (current[0] + dx, current[1] + dy)

            # Ensure the neighbor is within the grid boundaries and is not an obstacle
            if 0 <= neighbor[0] < COL and 0 <= neighbor[1] < ROW and grid[neighbor[0]][neighbor[1]] == 1:
                # Compute the cost from the start node to this neighbor
                tentative_g = current_g + 1

                # If the neighbor has not been visited, or we found a shorter path, update it
                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g  # Update cost to reach the neighbor
                    parents[neighbor] = current  # Set parent for path reconstruction
                    f_score = tentative_g + heuristic(neighbor, end)  # Compute f_score
                    open_set.append((f_score, tentative_g, neighbor, current))  # Add to open set

    # If the open set is empty and no path was found, return an empty list
    display_message("No path found!")
    return []  # Return an empty list if no valid path is found
