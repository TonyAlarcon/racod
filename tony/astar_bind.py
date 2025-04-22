import astar_ext

cfg = {
    "map_path": "./tony/dataset/boston/Boston_0_1024.map",
    "start": (332, 758),
    "goal" : (80, 773),
    "radius": 2,
    "threads": 10,
    "max_depth": 8,
    "algo": "serial",  #serial, pool, createjoin, pool_ras, createjoin_ras
}

path, elapsed = astar_ext.run_astar(
    cfg["map_path"],
    cfg["start"],
    cfg["goal"],
    radius=cfg["radius"],
    threads=cfg["threads"],
    max_depth=cfg["max_depth"],
    algo=cfg["algo"]
)

print(f"Algorithm   : {cfg['algo']}")
print(f"Start/Goal  : {cfg['start']} -> {cfg['goal']}")
print(f"Radius      : {cfg['radius']}")
print(f"Threads     : {cfg['threads']}")
print(f"Max depth   : {cfg['max_depth']}")
print(f"Path length : {len(path)}")
print(f"Time (ms)   : {elapsed:.3f}")
