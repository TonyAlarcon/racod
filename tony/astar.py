import time
import heapq
from typing import Tuple, List, Optional
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading
import math
import json 

from dataset.test_maps import tests, print_grid
from dataset.utils import load_map, save_map_with_markers

# ----------------------------------------------------
# Helper to wrap collision + cache access under a lock
# ----------------------------------------------------
def check_and_cache(pos: Tuple[int, int], grid: List[List[int]], radius: int, cache: dict, lock: threading.Lock) -> Tuple[Tuple[int,int], bool]:
    with lock:
        if pos in cache and cache[pos] is not None:
            return pos, cache[pos]
    is_free = check_collision_obb(pos, grid, radius)
    with lock:
        cache[pos] = is_free
    return pos, is_free


# ----------------------------------------------------
# Collision Detection Function for OBB (circular robot footprint)
# ----------------------------------------------------
def check_collision_obb(pos: Tuple[int, int], grid: List[List[int]], radius: int) -> bool:
    x0, y0 = pos
    height, width = len(grid), len(grid[0])
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x, y = x0 + dx, y0 + dy
            if x < 0 or x >= width or y < 0 or y >= height:
                return False
            if grid[y][x] == '@':  # obstacle
                return False
    return True

# ----------------------------------------------------
# Node Definition for A*
# ----------------------------------------------------
class Node:
    def __init__(self, x: int, y: int, g: float = 0, h: float = 0, parent: Optional['Node'] = None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        
    def __lt__(self, other: 'Node'):
        return self.f < other.f
    
    def pos(self) -> Tuple[int, int]:
        return (self.x, self.y)

# ----------------------------------------------------
# Heuristic and Neighbors
# ----------------------------------------------------
def heuristic(a: Tuple[int, int], b: Tuple[int, int], connectivity:int=8) -> float:
    if connectivity == 4:
        #use manhattan distance
        distance = abs(a[0] - b[0]) + abs(a[1] - b[1])
    if connectivity == 8:
        #use euclidean distance
        distance = math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
    return distance

def get_neighbors(node: Node, grid: List[List[int]], connectivity: int=8) -> List[Tuple[int, int]]:
    x, y = node.x, node.y
    nbrs: List[Tuple[int,int]] = []
    
    # 4-connectivity
    if connectivity == 4:
        directions = [(0,1),(1,0),(0,-1),(-1,0)]
    # 8-connectivity
    elif connectivity == 8:
        directions = [(0,1),(1,0),(0,-1),(-1,0), (1,1),(-1,-1),(1,-1),(-1,1)]
    
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= ny < len(grid) and 0 <= nx < len(grid[0]):
            # forbid corner‐cutting
            if abs(dx)==1 and abs(dy)==1:
                if grid[y][x+dx]=='@' or grid[y+dy][x]=='@':
                    continue
            
            nbrs.append((nx, ny))
    return nbrs

# ----------------------------------------------------
# SERIAL: A* with OBB and Collision Caching
# ----------------------------------------------------
def astar_serial(grid: List[List[int]], start: Tuple[int,int], goal: Tuple[int,int], robot_radius: int) -> Optional[List[Tuple[int,int]]]:
    """
    A* search that includes OBB collision detection (radius in cells)
    with memoization of collision status.
    """
    start_node = Node(start[0], start[1], g=0, h=heuristic(start, goal))
    open_list = []
    heapq.heappush(open_list, start_node)
    closed_set = set()
    collision_cache = {}  # pos -> bool
    connectivity = 8  # 4 or 8 connectivity

    while open_list:
        current = heapq.heappop(open_list)
        pos = current.pos()
        if pos in closed_set:
            continue
        closed_set.add(pos)

        if pos == goal: #reconstruct path
            # Reconstruct path
            path = []
            n = current
            while n:
                path.append(n.pos())
                n = n.parent
            return path[::-1]

        # Examine neighbors
        for npos in get_neighbors(current, grid, connectivity):
            if npos in closed_set:
                continue

            # Check cache first
            if npos in collision_cache:
                free = collision_cache[npos]
            else:
                free = check_collision_obb(npos, grid, robot_radius)
                collision_cache[npos] = free

            if not free:
                continue  # Collision, skip
            # Valid neighbor: compute costs and push
            # compute step cost
            dx, dy = npos[0] - current.x, npos[1] - current.y
            step_cost = math.sqrt(dx*dx + dy*dy)
            tentative_g = current.g + step_cost # 1.0 for orthgonal, ~1.414 for diagonal
            h_val = heuristic(npos, goal, connectivity)
            neighbor = Node(npos[0], npos[1], g=tentative_g, h=h_val, parent=current)
            heapq.heappush(open_list, neighbor)

    return None  # No path found

# ----------------------------------------------------
# MULTITHREADED: A* with OBB Collision + Caching + Persistent Thread‑Pool
# ----------------------------------------------------
def astar_mt(grid: List[List[int]], start: Tuple[int,int], goal: Tuple[int,int], 
                      robot_radius: int, max_workers: int = 4) -> Optional[List[Tuple[int,int]]]:
    
    executor        = ThreadPoolExecutor(max_workers=max_workers)
    collision_cache = {}             # Cache of {pos: bool}
    cache_lock      = threading.Lock()
    open_list       = []             # heap of Node
    closed_set      = set()          # visited positions
    connectivity    = 8             # 4 or 8 connectivity

    start_node = Node(start[0], start[1], g=0, h=heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    result_path: Optional[List[Tuple[int,int]]] = None

    while open_list:
        current = heapq.heappop(open_list)
        pos     = current.pos()
        if pos in closed_set:
            continue
        closed_set.add(pos)

        if pos == goal:
            # Reconstruct path
            path: List[Tuple[int,int]] = []
            n = current
            while n:
                path.append(n.pos())
                n = n.parent
            result_path = path[::-1]
            break

        # 1) Gather unvisited neighbors
        nbrs = [npos for npos in get_neighbors(current, grid, connectivity)
                if npos not in closed_set]

        # 2) Fire off all collision checks at once
        futures = [ executor.submit(check_and_cache, npos, grid, robot_radius, collision_cache, cache_lock) 
                   for npos in nbrs ]

        # 3) Wait for *all* of them to finish
        results = [f.result() for f in futures]

        # 4) Now enqueue *all* the collision-free neighbors
        for npos, free in results:
            if not free:
                continue
            # compute step cost
            dx, dy = npos[0] - current.x, npos[1] - current.y
            step_cost = math.sqrt(dx*dx + dy*dy)
            tentative_g = current.g + step_cost # 1.0 for orthgonal, ~1.414 for diagonal
            h_val       = heuristic(npos, goal, connectivity)
            neighbor    = Node(npos[0], npos[1], g=tentative_g, h=h_val, parent=current)
            heapq.heappush(open_list, neighbor)

    executor.shutdown()
    return result_path



def astar_rasexp(grid: List[List[int]],
                 start: Tuple[int,int],
                 goal: Tuple[int,int],
                 robot_radius: int,
                 connectivity: int = 8,
                 num_workers: int = 8,
                 max_depth: int = 5
                ) -> Optional[List[Tuple[int,int]]]:
    open_heap = []
    closed    = set()
    collision_cache = {}             
    cache_lock      = threading.Lock()

    # push start
    start_h = heuristic(start, goal, connectivity)
    heapq.heappush(open_heap, Node(start[0], start[1], g=0, h=start_h))

    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        while open_heap:
            current = heapq.heappop(open_heap)
            pos = (current.x, current.y)
            if pos in closed:
                continue
            closed.add(pos)

            if pos == goal:
                path, node = [], current
                while node:
                    path.append((node.x, node.y))
                    node = node.parent
                return path[::-1]

            neighbors = get_neighbors(current, grid, connectivity)
            futures   = {}

            # Demand checks
            for npos in neighbors:
                if npos in closed:
                    continue
                with cache_lock:
                    val = collision_cache.get(npos, None)
                    if val is True or val is False:
                        continue
                    collision_cache[npos] = None
                fut = executor.submit(check_and_cache, npos, grid, robot_radius,
                                      collision_cache, cache_lock)
                futures[fut] = npos

            # Speculative run‑ahead, bounded by num_workers
            if current.parent and futures:
                dx = current.x - current.parent.x
                dy = current.y - current.parent.y
                depth = max_depth
                step  = 1
                # explicit while loop
                while depth > 0 and len(futures) < num_workers:
                    px = current.x + dx * step
                    py = current.y + dy * step
                    if not (0 <= py < len(grid) and 0 <= px < len(grid[0])):
                        break
                    # check each neighbor of the speculative point
                    spec_nbrs = get_neighbors(Node(px, py), grid, connectivity)
                    for npos in spec_nbrs:
                        if len(futures) >= num_workers:
                            break
                        with cache_lock:
                            val = collision_cache.get(npos, None)
                            if val is True or val is False:
                                continue
                            collision_cache[npos] = None
                        fut = executor.submit(check_and_cache, npos, grid, robot_radius,
                                              collision_cache, cache_lock)
                        futures[fut] = npos
                    depth -= 1
                    step  += 1

            # await all collision checks
            for fut in as_completed(futures):
                _, free = fut.result()

            # enqueue only the truly free neighbors
            for npos in neighbors:
                if npos in closed:
                    continue
                with cache_lock:
                    free = collision_cache.get(npos, False)
                if not free:
                    continue
                g2 = current.g + math.hypot(npos[0]-current.x, npos[1]-current.y)
                h2 = heuristic(npos, goal, connectivity)
                heapq.heappush(open_heap, Node(npos[0], npos[1], g=g2, h=h2, parent=current))

    return None






if __name__ == "__main__":

    #grid, start, goal, robot_radius = initial_test()
    #NOTE: grid is in row-major order
    boston_map_path = "tony/dataset/boston/Boston_0_1024.map"
    boston_state_path = "tony/dataset/boston/boston_0_1024_sampled_pairs.json"
    
    #load first test case
    with open(boston_state_path, 'r') as f:
        test_cases = json.load(f)
    test_case = test_cases["pairs"][0]
    grid          = load_map(boston_map_path)
    start, goal   = tuple(test_case[0]), tuple(test_case[1])
    robot_radius  = 2
    max_workers   = 10
    max_depth    = 8
    connectivity   = 8
    print("Start:", start)
    print("Goal:", goal)
    algorithm = "rasexp"  # "serial", "mt", "rasexp"
    
    
    if algorithm == "serial":
        print("Running A* with OBB (serial)...")
        t0 = time.time()
        path = astar_serial(grid, start, goal, robot_radius)    
        t1 = time.time()
    elif algorithm == "mt":
        print("Running A* with OBB (multithreaded)...")
        t0 = time.time()
        path = astar_mt(grid, start, goal, robot_radius, max_workers=max_workers)
        t1 = time.time()
    elif algorithm == "rasexp":
        print("Running A* with OBB (rasexp)...")
        t0 = time.time()
        path = astar_rasexp(grid, start, goal, robot_radius, connectivity=connectivity, num_workers=max_workers, max_depth=max_depth)
        t1 = time.time()
    else:
        raise ValueError("Unknown algorithm: {}".format(algorithm))

    if path:
        print(f"Path found in {t1-t0:.5f}s, length={len(path)}")
        #print("Path:", path)
        save_map_with_markers(grid, path=path, start=start, goal=goal, filename="astar_boston_test1.png")
        
    else:
        print("No path found.")
        save_map_with_markers(grid, start=start, goal=goal, filename="astar_boston_test1_failed.png")
        
        
