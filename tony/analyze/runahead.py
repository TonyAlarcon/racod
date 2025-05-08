import os
import pandas as pd
import matplotlib.pyplot as plt

def compute_per_depth_accuracy(results_dir: str, basename: str, algo: str, threads: int, depth_list: list[int]) -> dict[int, float]:
    """
    For each runahead R in depth_list, load the CSV:
      <basename>_results_<algo>_<threads>_<R>.csv
    and compute per-row accuracy_i = successful_speculations / total_speculations
    (treating 0/0 → 0), then return the MEAN accuracy for each R.
    """
    acc_by_R = {}
    for R in depth_list:
        fn = os.path.join(results_dir,
             f"{basename}_results_{algo}_{threads}th_{R}ra.csv")
        df = pd.read_csv(fn)
        # per‐run accuracy
        # if total_speculations==0, define accuracy_i = 0.0
        per_run = df.apply(
            lambda row: (row.successful_speculations / row.total_speculations)
                        if row.total_speculations>0 else 0.0,
            axis=1
        )
        acc_by_R[R] = per_run.mean()
    return acc_by_R

def plot_prediction_accuracy_bars(
    acc_by_R: dict[int, float],
    algo: str,
    threads: int
):
    """
    Given a dict R→accuracy, makes a bar chart with a log2 x‐axis.
    """
    Rs = sorted(acc_by_R)
    vals = [acc_by_R[R] for R in Rs]

    plt.figure(figsize=(6,4))
    plt.bar(Rs, vals, width=[0.8*R for R in Rs], align='center', alpha=0.7)
    plt.xscale('log', base=2)
    plt.xticks(Rs, Rs)
    plt.ylim(0,1.0)
    plt.xlabel('Run‐ahead depth (R)')
    plt.ylabel('Prediction accuracy')
    plt.title(f'RASExp Prediction Accuracy ({algo}, {threads} threads)')
    plt.grid(axis='y', ls='--', alpha=0.5)
    plt.tight_layout()
    plt.show()


#TODO: the below uses total_speculations which contains inccorrect ones not in the critical path. therefore, we need to subtract the incorrect from the total
def compute_per_depth_coverage(results_dir: str, basename: str, algo: str, threads: int, depth_list: list[int]) -> dict[int, float]:
    """
    For each runahead R in depth_list, load the CSV named
      <basename>_results_<algo>_<threads>_<R>.csv
    and compute per-row coverage_i = total_speculations /
                                   (demand_checks + total_speculations)
    (if denominator==0 we treat coverage_i as 0.0).  Return the MEAN
    coverage for each R.
    """
    cov_by_R = {}
    for R in depth_list:
        fn = os.path.join(
            results_dir,
            f"{basename}_results_{algo}_{threads}th_{R}ra.csv"
        )
        df = pd.read_csv(fn)
        per_row = df.apply(
            lambda row: (
                row.total_speculations
                / (row.demand_checks + row.total_speculations)
            ) if (row.demand_checks + row.total_speculations) > 0 else 0.0,
            axis=1
        )
        cov_by_R[R] = per_row.mean()
    return cov_by_R

def plot_prediction_coverage( cov_by_R: dict[int, float], algo: str, threads: int):
    """
    Line plot of mean prediction coverage vs. runahead R.
    """
    Rs   = sorted(cov_by_R)
    vals = [cov_by_R[R] for R in Rs]

    plt.figure(figsize=(6,4))
    plt.plot(Rs, vals, marker='o', linestyle='-')
    plt.xscale('log', base=2)
    plt.xticks(Rs, Rs)
    plt.ylim(0,1.0)
    plt.xlabel('Run‐ahead depth (R)')
    plt.ylabel('Prediction coverage')
    plt.title(f'RASExp Prediction Coverage ({algo}, {threads} threads)')
    plt.grid(True, ls='--', alpha=0.5)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    results_dir = "/home/droneresponse/tony/repos/racod/tony/results/spec_depth"
    basename    = "Boston_0_1024"
    algo        = "createjoin_ras"
    threads     = 32
    depth_list  = [2,4,8,16,32]

    acc_by_R = compute_per_depth_accuracy(
        results_dir, basename, algo, threads, depth_list
    )
    
    plot_prediction_accuracy_bars(acc_by_R, algo, threads)
    
    cov_by_R = compute_per_depth_coverage(
        results_dir, basename, algo, threads, depth_list
    )
    plot_prediction_coverage(cov_by_R, algo, threads)
    
