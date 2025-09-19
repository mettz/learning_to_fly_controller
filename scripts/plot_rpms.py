import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def extract_run_name(path: Path) -> str:
    stem = path.stem
    parts = stem.split('_')
    return parts[2] if len(parts) >= 3 else stem

def main():
    parser = argparse.ArgumentParser(description="Plot motor RPMs from CSV log.")
    parser.add_argument("csv", help="CSV file path")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    run_name = extract_run_name(csv_path)

    # ---------- load ----------
    df = pd.read_csv(csv_path)
    t = df['timestamp (ms)'].to_numpy(float) * 1e-3  # seconds
    m = df[['motor.m1','motor.m2','motor.m3','motor.m4']].to_numpy(float)

    # ---------- plot ----------
    fig, ax = plt.subplots(figsize=(8,4), dpi=150)
    plt.rcParams['lines.linewidth'] = 1.0
    ax.plot(t, m[:,0], label='m1')
    ax.plot(t, m[:,1], label='m2')
    ax.plot(t, m[:,2], label='m3')
    ax.plot(t, m[:,3], label='m4')
    # ax.set_xlim(5, 35)
    ax.set_ylim(0, 70000)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('motor RPM')
    ax.set_title(f"{run_name} – Motor RPMs vs Time")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    # ---------- show ----------
    plt.show()

    # ---------- save ----------
    out_file = f"{run_name}_motors.png"
    fig.savefig(out_file, bbox_inches='tight', dpi=300)
    print(f"Saved plot as: {out_file}")

    window_size = 50  # number of samples in the sliding window
    kernel = np.ones(window_size) / window_size

    fig, ax = plt.subplots(figsize=(8,4), dpi=150)
    plt.rcParams['lines.linewidth'] = 1.0
    for i in range(4):
        rolling_mean = np.convolve(m[:, i], kernel, mode='valid')
        ax.plot(t[window_size-1:], rolling_mean, label=f'mean_m{i+1}')

    # ax.set_xlim(5, 35)
    ax.set_ylim(0, 70000)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('motor RPM')
    ax.set_title(f"{run_name} – Motor RPMs vs Time (sliding window = {window_size})")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()

    # ---------- show ----------
    plt.show()

    # ---------- save ----------
    out_file = f"{run_name}_motors_mean.png"
    fig.savefig(out_file, bbox_inches='tight', dpi=300)
    print(f"Saved plot as: {out_file}")

if __name__ == "__main__":
    main()