from typing import Tuple, List, Dict, Optional

# A single test case consists of:
#   – grid: the 2D map  
#   – start: (x,y)  
#   – goal:  (x,y)  
#   – robot_radius (optional; default can be 1)
TestCase = Dict[str, object]


# ----------------------------------------------------
# Utility to Print the Grid & Path
# ----------------------------------------------------
def print_grid(grid: List[List[str]], path: Optional[List[Tuple[int,int]]] = None):
    path_set = set(path or ())
    for y,row in enumerate(grid):
        line = []
        for x,c in enumerate(row):
            if (x,y) in path_set:
                line.append('P')
            else:
                line.append(c)   # '.' or '@'
        print(' '.join(line))
    print()

tests: Dict[str, TestCase] = {
    "figure1": {
        "grid": [
            ['.', '.', '.', '@', '@', '@', '@', '@', '@', '.', '.', '.'],
            ['.', '.', '.', '@', '@', '@', '@', '@', '@', '.', '.', '.'],
            ['.', '.', '.', '@', '@', '@', '@', '@', '@', '.', '.', '.'],
            ['.', '.', '.', '@', '@', '@', '@', '@', '@', '.', '.', '.'],
            ['.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.'],
            ['.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.'],
            ['.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.'],
            ['.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.'],
        ],
        "start": (1, 1),
        "goal":  (10, 1),
        "robot_radius": 1,
    },
    "open_corridor": {
        "grid": [['.'] * 20 for _ in range(20)],   # 20×20 empty
        "start": (3, 3),
        "goal":  (17, 17),
        "robot_radius": 2,
    },
}


