import time
import heapq
from typing import Tuple, List, Optional

# -------------------------------
# Simulation Parameters
# -------------------------------

# Simulated delay to represent an expensive collision check operation.
COLLISION_CHECK_DELAY = 0.1

# -------------------------------
# Collision Detection Function
# -------------------------------

def check_collision(pos: Tuple[int, int], grid: List[List[int]]) -> bool:
    """
    Simulates a collision check operation that is computationally expensive.
    In this simulation:
      - Returns True if the cell is free (i.e., grid value is 0).
      - Returns False if the cell is an obstacle (i.e., grid value is 1).
    """
    #time.sleep(COLLISION_CHECK_DELAY)  # Simulate heavy computation
    x, y = pos
    return grid[y][x] == 0

# -------------------------------
# Node Definition for A*
# -------------------------------

class Node:
    def __init__(self, x: int, y: int, g: float = 0, h: float = 0, parent: Optional['Node'] = None):
        self.x = x
        self.y = y
        self.g = g    # Cost from start to this node.
        self.h = h    # Heuristic cost to goal.
        self.f = g + h
        self.parent = parent

    def __lt__(self, other: 'Node'):
        return self.f < other.f

    def pos(self) -> Tuple[int, int]:
        return (self.x, self.y)

# -------------------------------
# Helper Functions
# -------------------------------

def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """
    Manhattan distance heuristic is used here.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_neighbors(node: Node, grid: List[List[int]]) -> List[Tuple[int, int]]:
    """
    Returns the list of 4-connected neighboring positions (up, down, left, right).
    """
    x, y = node.x, node.y
    neighbors = []
    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
        nx, ny = x + dx, y + dy
        # Check whether the neighbor is within grid bounds.
        if 0 <= ny < len(grid) and 0 <= nx < len(grid[0]):
            neighbors.append((nx, ny))
    return neighbors

# -------------------------------
# Standard A* Algorithm (Collision-Free)
# -------------------------------

def astar(grid: List[List[int]], start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    """
    A* search that expands nodes in order of increasing f = g + h.
    For each neighbor, a collision check is performed (which we simulate as expensive).
    Only nodes passing the collision check are added to the OPEN list.
    """
    start_node = Node(start[0], start[1], g=0, h=heuristic(start, goal))
    open_list = []
    heapq.heappush(open_list, start_node)
    closed_set = set()

    while open_list:
        current = heapq.heappop(open_list)
        current_pos = current.pos()

        if current_pos in closed_set:
            continue

        closed_set.add(current_pos)

        # Check if we have reached the goal.
        if current_pos == goal:
            path = []
            while current:
                path.append(current.pos())
                current = current.parent
            return path[::-1]  # Return reversed path.

        # For each neighbor, perform collision detection.
        for neighbor_pos in get_neighbors(current, grid):
            if neighbor_pos in closed_set:
                continue
            # Only consider free cells.
            if not check_collision(neighbor_pos, grid):
                continue

            tentative_g = current.g + 1  # Uniform cost for moving to a neighbor.
            h_val = heuristic(neighbor_pos, goal)
            neighbor_node = Node(neighbor_pos[0], neighbor_pos[1], g=tentative_g, h=h_val, parent=current)
            heapq.heappush(open_list, neighbor_node)

    return None  # No path found.

# -------------------------------
# Utility: Grid Printer
# -------------------------------

def print_grid(grid: List[List[int]], path: Optional[List[Tuple[int, int]]] = None):
    for y in range(len(grid)):
        row = ""
        for x in range(len(grid[0])):
            if path and (x, y) in path:
                row += "P "  # Mark the path.
            elif grid[y][x] == 1:
                row += "X "  # Mark obstacles.
            else:
                row += ". "  # Free cell.
        print(row)



if __name__ == "__main__":
    # Define a 10x10 grid:
    # 0 indicates free space; 1 indicates an obstacle.
    grid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 1, 0],
        [0, 0, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
        [0, 1, 1, 1, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 1, 1, 1, 0],
        [0, 1, 0, 1, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
    ]

    start = (0, 0)
    goal = (9, 9)

    print("Initial grid:")
    print_grid(grid)

    print("\nRunning A* search with collision checking simulation...")
    start_time = time.time()
    path = astar(grid, start, goal)
    end_time = time.time()

    if path:
        print(f"\nPath found in {end_time - start_time:.2f} seconds:")
        print(path)
        print("\nGrid with path marked (P):")
        print_grid(grid, path=path)
    else:
        print("No path found.")
