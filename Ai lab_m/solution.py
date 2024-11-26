import math
from queue import PriorityQueue

def heuristic(x1, y1, x2, y2):
    """Calculate the Euclidean distance heuristic."""
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

class State:
    def __init__(self, x, y, g, f, parent=None):
        """
        A state represents a cell in the grid.
        :param x: Row index of the cell
        :param y: Column index of the cell
        :param g: Cost to reach this state
        :param f: Estimated total cost (g + h)
        :param parent: Previous state in the optimal path
        """
        self.x = x
        self.y = y
        self.g = g
        self.f = f
        self.parent = parent

    def __lt__(self, other):
        """Comparison function for PriorityQueue."""
        return self.f < other.f

def reconstruct_path(state):
    """Reconstruct the path from the goal state to the start state."""
    path = []
    while state:
        path.append((state.x, state.y))
        state = state.parent
    return path[::-1]

def a_star(grid, start, goal):
    """
    Implement the A* algorithm.
    :param grid: 2D list representing the grid
    :param start: (x, y) tuple for the starting cell
    :param goal: (x, y) tuple for the destination cell
    :return: Total cost and the optimal path
    """
    n = len(grid)
    sx, sy = start
    dx, dy = goal

    # Directions for moving in preferred order
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0),  # Right, Down, Left, Up
                  (1, 1), (-1, -1), (-1, 1), (1, -1)]  # Diagonal movements

    # PriorityQueue to store states
    pq = PriorityQueue()

    # Start state includes the cost of the starting cell grid[Sx][Sy]
    start_state = State(sx, sy, grid[sx][sy], grid[sx][sy] + heuristic(sx, sy, dx, dy))
    pq.put((start_state.f, start_state))

    visited = set()

    while not pq.empty():
        _, current = pq.get()

        # If the goal is reached, reconstruct the path
        if (current.x, current.y) == goal:
            return current.g, reconstruct_path(current)

        # Mark as visited
        if (current.x, current.y) in visited:
            continue
        visited.add((current.x, current.y))

        # Explore neighbors in preferred order
        for d_x, d_y in directions:
            nx, ny = current.x + d_x, current.y + d_y

            # Check bounds and obstacles
            if 0 <= nx < n and 0 <= ny < n and grid[nx][ny] != -1:
                new_g = current.g + grid[nx][ny]
                new_f = new_g + heuristic(nx, ny, dx, dy)
                neighbor = State(nx, ny, new_g, new_f, current)
                pq.put((neighbor.f, neighbor))

    return None, None  # If no path is found

# Main function for user input and output
def main():
    # User input for the grid
    print("Enter the grid row by row as space-separated integers:")
    grid = []
    while True:
        row = input()
        if row.strip() == "":
            break
        grid.append(list(map(int, row.split())))

    # Input start and destination coordinates
    Sx, Sy = map(int, input("Enter start coordinates (Sx Sy): ").split())
    Dx, Dy = map(int, input("Enter destination coordinates (Dx Dy): ").split())

    # Run the A* algorithm
    cost, path = a_star(grid, (Sx, Sy), (Dx, Dy))

    # Output results
    if path:
        print(f"Optimal Cost: {cost}")
        print(f"Optimal Path: {'→'.join(map(str, path))}")
    else:
        print("No path found.")

if __name__ == "__main__":
    main()


'''

Example 1:
Enter the grid row by row as space-separated integers:
1 1 1 -1
1 -1 1 1
1 -1 1 1
1 1 1 1

Enter start coordinates (Sx Sy): 0 0
Enter destination coordinates (Dx Dy): 3 3
Optimal Cost: 5
Optimal Path: (0, 0)→(0, 1)→(1, 2)→(2, 3)→(3, 3)

Example 2:
Enter the grid row by row as space-separated integers:
2 3 1 -1
1 -1 4 2
1 2 3 1
3 -1 2 1

Enter start coordinates (Sx Sy): 0 0
Enter destination coordinates (Dx Dy): 3 3
Optimal Cost: 8
Optimal Path: (0, 0)→(1, 0)→(2, 1)→(3, 2)→(3, 3)


'''