import os 
import csv
import json

import astar_ext

def run_tests_from_csv(samples_csv_path: str, map_path: str, threads: int, max_depth: int,algo: str, output_dir: str = '.') -> str:
    """
    Read start/goal pairs and radius from samples_csv_path, run A* for each
    using the given parameters, and write results to <mapname>_results_<algo>.csv.
    Returns the results CSV path.
    """
    map_name = os.path.splitext(os.path.basename(map_path))[0]
    results_filename = f"{map_name}_results_{algo}_{threads}.csv"
    results_path = os.path.join(output_dir, results_filename)

    with open(samples_csv_path, 'r', newline='') as infile, \
         open(results_path, 'w', newline='') as outfile:
        reader = csv.DictReader(infile)
        # Prepare output fields: existing plus config and results
        fieldnames = reader.fieldnames + ['threads', 'max_depth', 'algo', 'time_ms', 'path']
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            # Parse inputs
            sx = int(row['start_x'])
            sy = int(row['start_y'])
            gx = int(row['goal_x'])
            gy = int(row['goal_y'])
            radius = int(row['radius'])

            # Run the A* extension
            path, elapsed = astar_ext.run_astar(
                map_path,
                (sx, sy),
                (gx, gy),
                radius=radius,
                threads=threads,
                max_depth=max_depth,
                algo=algo
            )

            # Build and write output row
            out_row = row.copy()
            out_row.update({
                'threads': threads,
                'max_depth': max_depth,
                'algo': algo,
                'time_ms': f"{elapsed:.3f}",
                'path': json.dumps(path)
            })
            writer.writerow(out_row)

    return results_path




if __name__ == "__main__":
    
    #algorithms: pool, serial, pool_ras, create_join, createjoin_ras

    print("A* extension initialized.")
    # Configuration parameters
    threads = [1, 2, 4, 8, 16, 32]
    algorithms = ["pool", "serial", "pool_ras", "create_join", "createjoin_ras"]
    #algorithms = ["createjoin", "createjoin_ras"]
    max_depth = 8
    algo = "pool"
    output_dir = "./tony/results/speedup"
    samples_csv = "./tony/dataset/boston/Boston_0_1024_samples.csv"
    map_file = "./tony/dataset/boston/Boston_0_1024.map"
    
    for algo in algorithms:
        for thread in threads:
            print(f"Running A* with {thread} threads and {algo} algorithm.")
            results_csv = run_tests_from_csv(
                samples_csv,
                map_file,
                thread,
                max_depth,
                algo,
                output_dir
            )
            print(f"Results saved to {results_csv}")

