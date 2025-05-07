import os
import pandas as pd
import matplotlib.pyplot as plt


def plot_time_vs_threads(results_dir: str, map_name: str, algorithm: str, threads: list, save:bool=False):

    mean_times = []
    for t in threads:
        csv_path = os.path.join(results_dir, f"{map_name}_results_{algorithm}_{t}.csv")
        df = pd.read_csv(csv_path)
        df['time_ms'] = pd.to_numeric(df['time_ms'], errors='coerce')
        mean_times.append(df['time_ms'].mean())

    plt.figure()
    plt.plot(threads, mean_times, marker='o')
    plt.xlabel('Threads')
    plt.ylabel('Average Time (ms)')
    plt.title(f'Time vs Threads: {algorithm}')
    plt.grid(True)
    plt.tight_layout()
    if save:
        plt.savefig(os.path.join(results_dir, f"{map_name}_{algorithm}_time_vs_threads.png"))
    else:
        plt.show()
    


def plot_relative_speedup(results_dir: str, map_name: str, threads: list, algorithms: list, save: bool=False):
    """
    For each algorithm, normalize its multi-thread times to its own 1-thread time,
    then plot speedup vs threads for all algorithms on one chart.
    """
    # 1) Read each algorithm’s 1-thread baseline mean
    baseline_times = {}
    for algo in algorithms:
        base_csv = os.path.join(results_dir, f"{map_name}_results_{algo}_1.csv")
        base_df = pd.read_csv(base_csv)
        base_df['time_ms'] = pd.to_numeric(base_df['time_ms'], errors='coerce')
        baseline_times[algo] = base_df['time_ms'].mean()

    # 2) Build and plot each curve
    plt.figure()
    for algo in algorithms:
        speedups = []
        for t in threads:
            test_csv = os.path.join(results_dir, f"{map_name}_results_{algo}_{t}.csv")
            df = pd.read_csv(test_csv)
            df['time_ms'] = pd.to_numeric(df['time_ms'], errors='coerce')
            mean_time = df['time_ms'].mean()
            speedups.append(baseline_times[algo] / mean_time if mean_time > 0 else float('nan'))
        plt.plot(threads, speedups, marker='o', label=algo)

    plt.xlabel('Threads')
    plt.ylabel('Speedup over own 1-thread')
    plt.title('Relative Speedup vs Threads')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    if save:
        out_path = os.path.join(results_dir, f"{map_name}_relative_speedup.png")
        plt.savefig(out_path)
        print(f"Saved speedup plot to {out_path}")
    else:
        plt.show()
        

if __name__ == "__main__":

    results_dir = "./tony/results/speedup"
    map_name = "Boston_0_1024"
    algorithm = "pool_ras"
    threads = [1, 2, 4, 8, 10]
    
    plot_time_vs_threads(results_dir, map_name, algorithm, threads)
    
    algorithms = ["pool", "pool_ras", "serial", "createjoin", "createjoin_ras"]
    baseline_csv = os.path.join(results_dir, f"{map_name}_results_pool_1.csv")
    plot_relative_speedup(results_dir, map_name, threads, algorithms, baseline_csv)