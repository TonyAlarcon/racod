import pandas as pd
import matplotlib.pyplot as plt
import ast
from collections import Counter
import numpy as np
import os


def run_lengths_from_paths(path_strs):
    def normalize_dir(dx, dy):
        return (int(np.sign(dx)), int(np.sign(dy)))
    runs = []
    for s in path_strs:
        pts = ast.literal_eval(s)
        dirs = [normalize_dir(x2-x1, y2-y1)
                for (x1,y1),(x2,y2) in zip(pts, pts[1:])]
        if not dirs:
            continue
        run_len = 1
        for i in range(1, len(dirs)):
            if dirs[i] == dirs[i-1]:
                run_len += 1
            else:
                runs.append(run_len)
                run_len = 1
        runs.append(run_len)
    return runs

def plot_runlength_cdf(run_lengths, save_path=None):
    cnt = Counter(run_lengths)
    total_steps = sum(k*v for k,v in cnt.items())
    ks = sorted(cnt)
    # compute fraction of steps in runs >= k
    frac_ge = []
    for k in ks:
        steps_ge = sum(length*count
                       for length,count in cnt.items()
                       if length >= k)
        frac_ge.append(steps_ge / total_steps)
    plt.figure()
    plt.plot(ks, frac_ge, marker='o')
    plt.xlabel('Minimum run‐length k')
    plt.ylabel('Fraction of all steps in runs ≥ k')
    plt.title('CDF of run‐lengths')
    plt.grid(True)
    plt.tight_layout()
    
    
    if save_path:
        plt.savefig(save_path)
        
    #plt.show()
        


def compute_step_based_stats(path_strs, targets=(0.5, 0.8, 0.9)):
    runs = run_lengths_from_paths(path_strs)
    cnt = Counter(runs)
    total_steps = sum(k * v for k, v in cnt.items())

    # 1) Proportion of steps that are a straight continuation
    prop_straight = 1 - (cnt[1] / total_steps)

    # 2) Step-weighted mean run-length (E[K] under step-distribution)
    mean_weighted = sum(k * k * v for k, v in cnt.items()) / total_steps

    # 3) Max observed run-length
    max_run = max(cnt)

    # 4) For each target coverage, find smallest k with
    #    Fraction of steps in runs ≥ k ≥ target
    steps_cum = 0
    coverage_k = {}
    for k in sorted(cnt):
        steps_cum += k * cnt[k]
        frac = steps_cum / total_steps
        for t in targets:
            if t not in coverage_k and frac >= t:
                coverage_k[t] = k

    return {
        'prop_straight': prop_straight, #fraction of all steps that continue in the same direction as the previous one.
        'mean_weighted_run_length': mean_weighted, # the average straight-run length you’d “experience” if you pick a random step (steps in longer runs are more likely to be picked
        'max_run_length': max_run, #the longest straight streak observed—tells you your absolute upper bound.
        'coverage_depths': coverage_k #a dict mapping your coverage targets (e.g. 50%, 80%, 90%) to the smallest look-ahead k that covers at least that fraction of steps.
    }


if __name__ == "__main__":
    # Example usage:
    csv_path = "./tony/results/speedup/Boston_0_1024_results_pool_ras_1.csv" 
    df = pd.read_csv(csv_path)
    paths = df['path']  # your pandas column of path‐strings
    dir = './tony/analyze'
    title = 'runlength_cdf.png'
    save_path = os.path.join(dir, title)
    
    rl = run_lengths_from_paths(paths)
    plot_runlength_cdf(rl, save_path=save_path)
    stats = compute_step_based_stats(paths)
    print("Run-length stats:", stats)