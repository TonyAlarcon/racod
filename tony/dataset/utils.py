from typing import List, Tuple, Optional, Set
from PIL import Image, ImageDraw
import random
from typing import List, Tuple
import json
from collections import deque
import os
import csv

def load_map(path):
    with open(path, 'r') as f:
        f.readline()                        # skip "type octile"
        H = int(f.readline().split()[1])    # height
        W = int(f.readline().split()[1])    # width
        f.readline()                        # skip "map"
        grid = [list(f.readline().rstrip()) for _ in range(H)]
    return grid   #returns in classic row-major order (y,x) format



def sample_feasible_pairs(grid: List[List[int]], robot_radius: int,  sample_count: int) -> List[Tuple[Tuple[int,int],Tuple[int,int]]]:
    """
    Returns up to sample_count (start, goal) pairs where neither the start nor goal OBB overlaps an obstacle.
    """
    h, w = len(grid), len(grid[0])
    pairs    = []
    attempts = 0
    max_attempts = sample_count * 10

    while len(pairs) < sample_count and attempts < max_attempts:
        sx, sy = random.randrange(w), random.randrange(h)
        gx, gy = random.randrange(w), random.randrange(h)

        if check_collision_obb((sx,sy), grid, robot_radius) and check_collision_obb((gx,gy), grid, robot_radius):
            pairs.append(((sx,sy),(gx,gy)))
        attempts += 1

    return pairs

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

    
    
def save_map_with_markers(
    grid: List[List[str]],
    path: Optional[List[Tuple[int, int]]] = None,
    start: Optional[Tuple[int, int]] = None,
    goal: Optional[Tuple[int, int]] = None,
    robot_radius: int = 10,
    filename: str = "map_circle.png"
):
    """
    Save the map as a PNG with:
    - obstacles in black
    - free space in white
    - start marked with a green circle of radius robot_radius
    - goal marked with a blue circle of radius robot_radius
    - path in red
    """
    h, w = len(grid), len(grid[0])
    img = Image.new("RGB", (w, h), color=(255, 255, 255))
    draw = ImageDraw.Draw(img)
    obstacle_chars = {"@", "O", "T"}
    path_set = set(path) if path else set()
    
    # Draw map + path
    for y in range(h):
        for x in range(w):
            if (x, y) in path_set:
                draw.point((x, y), fill=(255, 0, 0))
            elif grid[y][x] in obstacle_chars:
                draw.point((x, y), fill=(0, 0, 0))
    # Draw start/goal circles
    for center, color in [(start, (0, 255, 0)), (goal, (0, 0, 255))]:
        if center:
            cx, cy = center
            bbox = [cx - robot_radius, cy - robot_radius,
                    cx + robot_radius, cy + robot_radius]
            draw.ellipse(bbox, outline=color)
    img.save(filename)
    


def is_reachable(grid: List[List[str]],
                 start: Tuple[int,int],
                 goal:  Tuple[int,int],
                 radius: int = 1,
                 connectivity: int = 8) -> bool:
    """
    Return True if goal is reachable from start on `grid` when
    the robot has circular footprint `radius` and we use
    4- or 8-connectivity (with corner-cutting forbidden).
    """
    h, w = len(grid), len(grid[0])
    obstacle = {"@", "O", "T"}

    def in_bounds(x,y):
        return 0 <= x < w and 0 <= y < h

    def collides(pos):
        x0,y0 = pos
        # same OBB check as in your A*
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                x, y = x0+dx, y0+dy
                if not in_bounds(x,y) or grid[y][x] in obstacle:
                    return True
        return False

    # Early exit if endpoints themselves collide
    if collides(start) or collides(goal):
        return False

    # Precompute neighbor offsets
    ortho = [(0,1),(1,0),(0,-1),(-1,0)]
    diag = [(1,1),(-1,-1),(1,-1),(-1,1)]
    deltas = ortho + (diag if connectivity==8 else [])

    seen: Set[Tuple[int,int]] = {start}
    q = deque([start])

    while q:
        x0,y0 = q.popleft()
        if (x0,y0) == goal:
            return True
        for dx,dy in deltas:
            x,y = x0+dx, y0+dy
            if not in_bounds(x,y) or (x,y) in seen:
                continue
            # corner-cut guard
            if abs(dx)==1 and abs(dy)==1:
                if grid[y0][x0+dx] in obstacle or grid[y0+dy][x0] in obstacle:
                    continue
            # OBB collision check
            if collides((x,y)):
                continue
            seen.add((x,y))
            q.append((x,y))

    return False

def save_samples_to_csv(map_path: str, samples: List[Tuple[Tuple[int, int], Tuple[int, int]]], radius: int, output_dir: str = '.') -> str:
    """
    Save start/goal pairs and radius to a CSV file named <mapname>_samples.csv in output_dir.
    Returns the path to the CSV file.
    """
    # Extract map name without extension
    map_name = os.path.splitext(os.path.basename(map_path))[0]
    filename = f"{map_name}_samples.csv"
    file_path = os.path.join(output_dir, filename)

    # Write CSV
    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Header row
        writer.writerow(['start_x', 'start_y', 'goal_x', 'goal_y', 'radius'])
        for (sx, sy), (gx, gy) in samples:
            writer.writerow([sx, sy, gx, gy, radius])

    return file_path


if __name__ == "__main__":
    # Example usage
    map_file = "tony/dataset/boston/Boston_0_1024.map"
    grid = load_map(map_file)

    # Sample feasible start/goal pairs
    robot_radius = 3
    sample_count = 100
    pairs = sample_feasible_pairs(grid, robot_radius, sample_count)

    # Save to CSV
    output_dir = "tony/dataset/boston"
    csv_path = save_samples_to_csv(map_file, pairs, robot_radius, output_dir)
    print(f"Saved {len(pairs)} samples to {csv_path}")
    
    
    
    
    
    
    
    
