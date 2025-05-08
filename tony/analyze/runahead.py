import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick

def compute_per_depth_accuracy(results_dir: str, basename: str, algo: str, threads: int, depth_list: list[int]) -> dict[int, float]:
    acc_by_R = {}
    for R in depth_list:
        fn = os.path.join(results_dir,
             f"{basename}_results_{algo}_{threads}th_{R}ra.csv")
        df = pd.read_csv(fn)
        per_run = df.apply(
            lambda row: (row.successful_speculations / row.total_speculations)
                        if row.total_speculations > 0 else 0.0,
            axis=1
        )
        acc_by_R[R] = per_run.mean()
    return acc_by_R

def compute_per_depth_coverage(results_dir: str, basename: str, algo: str, threads: int, depth_list: list[int]) -> dict[int, float]:
    cov_by_R = {}
    for R in depth_list:
        fn = os.path.join(results_dir,
             f"{basename}_results_{algo}_{threads}th_{R}ra.csv")
        df = pd.read_csv(fn)
        per_run = df.apply(
            lambda row: (
                row.total_speculations
                / (row.demand_checks + row.total_speculations)
            ) if (row.demand_checks + row.total_speculations) > 0 else 0.0,
            axis=1
        )
        cov_by_R[R] = per_run.mean()
    return cov_by_R



def plot_accuracy_and_coverage( acc_by_R: dict[int, float], cov_by_R: dict[int, float], algo: str, threads: int, savepath: str=''):
    Rs       = sorted(acc_by_R)
    acc_vals = [acc_by_R[R] for R in Rs]
    cov_vals = [cov_by_R[R] for R in Rs]

    fig, ax = plt.subplots(figsize=(8,5))


    bar_widths = [0.2 * R for R in Rs]
    ax.bar(
        Rs, acc_vals,
        width=bar_widths,
        align='center',
        alpha=0.7,
        label='Prediction Accuracy'
    )

    ax.plot(
        Rs, cov_vals,
        marker='o',
        linewidth=2,
        label='Prediction Coverage'
    )


    ax.set_xscale('log', base=2)
    ax.set_xticks(Rs)
    ax.set_xticklabels(Rs)

 
    ax.set_ylim(0, 1.0)
    ax.yaxis.set_major_formatter(mtick.PercentFormatter(xmax=1.0))


    ax.set_xlabel('Run-ahead depth (R)')
    ax.set_ylabel('Value (%)')
    ax.set_title(f'RASExp Accuracy & Coverage ({algo}, {threads} threads)')
    ax.grid(axis='y', ls='--', alpha=0.5)
    ax.legend(loc='best')

    plt.tight_layout()
    if savepath:
        plt.savefig(os.path.join(
            savepath,
            f"accuracy_coverage.png"
        ))
    plt.show()
  


if __name__ == "__main__":
    results_dir = "/Users/tonyalarcon/repos/research/racod/tony/results/spec_depth"
    basename    = "Boston_0_1024"
    algo        = "createjoin_ras"
    threads     = 10
    depth_list  = [2,4,8,16,32]

    acc_by_R = compute_per_depth_accuracy(
        results_dir, basename, algo, threads, depth_list
    )
    cov_by_R = compute_per_depth_coverage(
        results_dir, basename, algo, threads, depth_list
    )
    
    savepath = 'tony/analyze'
    plot_accuracy_and_coverage(acc_by_R, cov_by_R, algo, threads, savepath)
