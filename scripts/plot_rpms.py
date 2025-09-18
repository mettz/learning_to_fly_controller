import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

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
    ax.plot(t, m[:,0], label='m1')
    ax.plot(t, m[:,1], label='m2')
    ax.plot(t, m[:,2], label='m3')
    ax.plot(t, m[:,3], label='m4')
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

if __name__ == "__main__":
    main()