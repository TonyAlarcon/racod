# dijkstra_baseline_runner.py
import csv
import os
import time
import json
import math
import heapq
from typing import Tuple, List, Optional

MAP_PATH = "./dataset/boston/Boston_0_1024.map"
SAMPLES_CSV_PATH = "./dataset/boston/Boston_0_1024_samples.csv"
OUTPUT_CSV_PATH = "./dataset/boston/Boston_0_1024_results_dijkstra.csv"

# ---------------------------------------------
def load_map(path: str) -> List[List[str]]:
    with open(path, 'r') as f:
        f.readline()
        H = int(f.readline().split()[1])
        W = int(f.readline().split()[1])
        f.readline()
        grid = [list(f.readline().rstrip()) for _ in range(H)]
    return grid

# ---------------------------------------------
def check_collision(pos: Tuple[int, int], grid: List[List[str]], radius: int) -> bool:
    x0, y0 = pos
    H, W = len(grid), len(grid[0])
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            x, y = x0 + dx, y0 + dy
            if not (0 <= x < W and 0 <= y < H):
                return False
            if grid[y][x] == '@':
                return False
    return True

# ---------------------------------------------
def get_neighbors(pos: Tuple[int, int], grid: List[List[str]]) -> List[Tuple[int, int]]:
    x, y = pos
    H, W = len(grid), len(grid[0])
    nbrs = []
    for dx, dy in [(0,1),(1,0),(0,-1),(-1,0),(1,1),(-1,-1),(1,-1),(-1,1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < W and 0 <= ny < H:
            if abs(dx)==1 and abs(dy)==1 and (grid[y][x+dx]=='@' or grid[y+dy][x]=='@'):
                continue  # corner cut
            nbrs.append((nx, ny))
    return nbrs

# ---------------------------------------------
def dijkstra(grid: List[List[str]], start: Tuple[int,int], goal: Tuple[int,int], radius: int) -> Optional[List[Tuple[int,int]]]:
    open_list = []
    heapq.heappush(open_list, (0.0, start, None))
    came_from = {}
    cost_so_far = {start: 0.0}

    while open_list:
        g, current, parent = heapq.heappop(open_list)

        if current in came_from:
            continue
        came_from[current] = parent

        if current == goal:
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for neighbor in get_neighbors(current, grid):
            if not check_collision(neighbor, grid, radius):
                continue
            new_cost = cost_so_far[current] + math.hypot(neighbor[0]-current[0], neighbor[1]-current[1])
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                heapq.heappush(open_list, (new_cost, neighbor, current))

    return None

# ---------------------------------------------
def run():
    grid = load_map(MAP_PATH)

    with open(SAMPLES_CSV_PATH, newline='') as f:
        reader = csv.DictReader(f)
        samples = list(reader)

    fieldnames = reader.fieldnames + ['algo', 'time_ms', 'path']

    with open(OUTPUT_CSV_PATH, 'w', newline='') as outf:
        writer = csv.DictWriter(outf, fieldnames=fieldnames)
        writer.writeheader()

        for row in samples:
            sx = int(row['start_x'])
            sy = int(row['start_y'])
            gx = int(row['goal_x'])
            gy = int(row['goal_y'])
            radius = int(row['radius'])

            start = (sx, sy)
            goal = (gx, gy)

            t0 = time.time()
            path = dijkstra(grid, start, goal, radius)
            t1 = time.time()
            elapsed_ms = (t1 - t0) * 1000.0

            row['algo'] = 'dijkstra'
            row['time_ms'] = f"{elapsed_ms:.3f}"
            row['path'] = json.dumps(path) if path else "[]"
            writer.writerow(row)

    print(f"Results written to {OUTPUT_CSV_PATH}")

if __name__ == "__main__":
    run()