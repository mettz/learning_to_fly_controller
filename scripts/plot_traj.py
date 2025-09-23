#!/usr/bin/env python3
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# ---------- helpers ----------
def colored_trajectory(ax, P, speed, cmap='turbo', lw=2.0):
    segs = np.stack([P[:-1], P[1:]], axis=1)
    lc = Line3DCollection(segs, cmap=cmap, linewidth=lw)
    lc.set_array(speed)
    ax.add_collection(lc)
    ax.scatter(*P[0], marker='v', s=60, depthshade=False, label='start')
    ax.scatter(*P[-1], marker='s', s=60, depthshade=False, label='end')
    return lc

def load_csv(path, velocity=True):
    df = pd.read_csv(path)
    t  = df['timestamp (ms)'].to_numpy(float) * 1e-3
    P  = df[['stateEstimate.x','stateEstimate.y','stateEstimate.z']].to_numpy(float)
    if velocity:
        V  = df[['stateEstimate.vx','stateEstimate.vy','stateEstimate.vz']].to_numpy(float)
        spd = np.linalg.norm(V, axis=1)
        spd_seg = 0.5 * (spd[:-1] + spd[1:])
        return t, P, V, spd_seg
    else:
        return t, P, None, None

def extract_run_name(path: Path) -> str:
    stem = path.stem
    parts = stem.split('_')
    return parts[2] if len(parts) >= 3 else stem

# ---------- main ----------
def main():
    parser = argparse.ArgumentParser(description="Plot drone trajectory from CSV log.")
    parser.add_argument("csv", help="CSV file path")
    parser.add_argument("--target", type=float, nargs=3, metavar=('X','Y','Z'),
                        help="Target position [m]")
    parser.add_argument("--no-velocity", action='store_true', help="Do not plot velocity")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    run_name = extract_run_name(csv_path)

    t, P, V, spd_seg = load_csv(csv_path, velocity=not args.no_velocity)

    # ========== (1) 3D trajectory ==========
    if spd_seg is not None:
        fig1 = plt.figure(figsize=(6, 5), dpi=150)
        ax1 = fig1.add_subplot(111, projection='3d')
        mappable = colored_trajectory(ax1, P, spd_seg)
        cb = plt.colorbar(mappable, ax=ax1, pad=0.02, fraction=0.046)
        cb.set_label('speed [m/s]')
        if args.target is not None:
            ax1.scatter(*args.target, marker='*', s=160, depthshade=False, label='target')
        ax1.set_xlabel('x [m]')
        ax1.set_ylabel('y [m]')
        ax1.set_zlabel('z [m]')
        ax1.set_xlim(-1, 1)
        ax1.set_ylim(-1, 1)
        ax1.set_zlim(0.0, 1.0)
        ax1.view_init(elev=50, azim=-60)
        ax1.legend(loc='upper right', fontsize=8)
        ax1.set_title(f"{run_name} – 3D trajectory")
        fig1.tight_layout()
        fig1.savefig(f"{run_name}_trajectory3d.png", bbox_inches='tight', dpi=300)


    # ========== (2) position vs time ==========
    fig2, ax2 = plt.subplots(figsize=(6,4), dpi=150)
    ax2.plot(t, P[:,0], label='x')
    ax2.plot(t, P[:,1], label='y')
    ax2.plot(t, P[:,2], label='z')
    ax2.set_xlabel('time [s]')
    ax2.set_ylabel('position [m]')
    ax2.set_title(f"{run_name} – Position vs Time")
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    fig2.tight_layout()
    fig2.savefig(f"{run_name}_position.png", bbox_inches='tight', dpi=300)

    # ========== (3) velocity vs time ==========
    if V is not None:
        fig3, ax3 = plt.subplots(figsize=(6,4), dpi=150)
        ax3.plot(t, V[:,0], label='vx')
        ax3.plot(t, V[:,1], label='vy')
        ax3.plot(t, V[:,2], label='vz')
        ax3.set_xlabel('time [s]')
        ax3.set_ylabel('velocity [m/s]')
        ax3.set_title(f"{run_name} – Velocity vs Time")
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        fig3.tight_layout()
        fig3.savefig(f"{run_name}_velocity.png", bbox_inches='tight', dpi=300)

    # ---------- show first ----------
    plt.show()
    

    # ---------- compute XY position error ----------
    if args.target is not None:
        target_xy = np.array(args.target[:2])
        pos_xy = P[:, :2]
        errors = np.linalg.norm(pos_xy - target_xy, axis=1)
        print("\nXY position error statistics [m]:")
        print(f"  median:  {np.median(errors):.4f}")
        print(f"  mean:    {np.mean(errors):.4f}")
        print(f"  minimum: {np.min(errors):.4f}")

if __name__ == "__main__":
    main()