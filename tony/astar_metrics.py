import os
import csv
import json
import astar_metrics_ext  

def run_tests_from_csv(samples_csv_path: str,
                       map_path: str,
                       threads: int,
                       runahead: int,
                       algo: str,
                       output_dir: str = '.') -> str:
    map_name = os.path.splitext(os.path.basename(map_path))[0]
    fn = f"{map_name}_results_{algo}_{threads}th_{runahead}ra.csv"
    out_path = os.path.join(output_dir, fn)

    with open(samples_csv_path, 'r', newline='') as inf, \
         open(out_path,        'w', newline='') as outf:

        reader = csv.DictReader(inf)
        fieldnames = reader.fieldnames + [
            'threads','runahead','algo','time_ms',
            'expansions','demand_checks',
            'total_speculations','successful_speculations'
        ]
        writer = csv.DictWriter(outf, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            sx,sy = int(row['start_x']), int(row['start_y'])
            gx,gy = int(row['goal_x']),  int(row['goal_y'])
            radius = int(row['radius'])

            path, elapsed, metrics = astar_metrics_ext.run_astar(
                map_path,
                (sx, sy),
                (gx, gy),
                radius=radius,
                threads=threads,
                max_depth=runahead,
                algo=algo,
                enable_metrics=True
            )

            out = row.copy()
            out.update({
                'threads': threads,
                'runahead': runahead,
                'algo': algo,
                'time_ms': f"{elapsed:.3f}",
                # unpack metrics dict
                'expansions':            metrics['expansions'],
                'demand_checks':         metrics['demand_checks'],
                'total_speculations':    metrics['total_speculations'],
                'successful_speculations': metrics['successful_speculations'],
            })
            writer.writerow(out)

    return out_path


if __name__ == "__main__":
    samples_csv = "./dataset/boston/Boston_0_1024_samples.csv"
    map_file     = "./dataset/boston/Boston_0_1024.map"
    output_dir   = "./results/spec_depth"

    threads   = 32
    runaheads = [2,4,8,16,32]
    algos     = ["serial", "pool","pool_ras","createjoin_ras", "createjoin"]

    for algo in algos:
        for ra in runaheads:
            print(f"→ algo={algo}, threads={threads}, runahead={ra}")
            csvp = run_tests_from_csv(
                samples_csv,
                map_file,
                threads,
                ra,
                algo,
                output_dir
            )
            print(f"   saved: {csvp}")
