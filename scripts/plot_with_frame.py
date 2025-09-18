#!/usr/bin/env python3
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# ---------- helpers ----------
def rpy_to_rotmat(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    # ZYX intrinsic rotation
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    Ry = np.array([[cp, 0, sp],
                   [ 0, 1,  0],
                   [-sp,0, cp]])
    Rx = np.array([[1,  0,  0],
                   [0, cr,-sr],
                   [0, sr, cr]])
    return Rz @ Ry @ Rx

def colored_trajectory(ax, P, speed, cmap='turbo', lw=2.0):
    segs = np.stack([P[:-1], P[1:]], axis=1)
    lc = Line3DCollection(segs, cmap=cmap, linewidth=lw)
    lc.set_array(speed)
    ax.add_collection(lc)
    ax.scatter(*P[0], marker='v', s=60, depthshade=False, label='start')
    ax.scatter(*P[-1], marker='s', s=60, depthshade=False, label='end')
    return lc

def load_csv(path):
    df = pd.read_csv(path)
    t  = df['timestamp (ms)'].to_numpy(float) * 1e-3
    P  = df[['stateEstimate.x','stateEstimate.y','stateEstimate.z']].to_numpy(float)
    RPY = df[['stabilizer.roll','stabilizer.pitch','stabilizer.yaw']].to_numpy(float)
    # velocities (finite differences)
    dP = np.diff(P, axis=0)
    dt = np.diff(t)[:, None]
    V = dP / dt
    V = np.vstack([V, V[-1]])  # pad to length N
    spd = np.linalg.norm(V, axis=1)
    return t, P, V, RPY, spd

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
    parser.add_argument("--frame_step", type=int, default=None,
                        help="Draw reference frame every N samples (default: 50)")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    run_name = extract_run_name(csv_path)

    t, P, V, RPY, spd = load_csv(csv_path)

        # ========== (1) 3D trajectory ==========
    fig1 = plt.figure(figsize=(6, 5), dpi=150)
    ax1 = fig1.add_subplot(111, projection='3d')
    mappable = colored_trajectory(ax1, P, spd)
    cb = plt.colorbar(mappable, ax=ax1, pad=0.02, fraction=0.046)
    cb.set_label('speed [m/s]')
    if args.target is not None:
        ax1.scatter(*args.target, marker='*', s=160, depthshade=False, label='target')
    ax1.set_xlabel('x [m]'); ax1.set_ylabel('y [m]'); ax1.set_zlabel('z [m]')
    ax1.view_init(elev=50, azim=-60)
    ax1.legend(loc='upper right', fontsize=8)
    ax1.set_title(f"{run_name} – 3D trajectory")

    # ---------- auto axis limits ----------
    mins = P.min(axis=0)
    maxs = P.max(axis=0)
    span = maxs - mins
    pad = 0.1 * span
    ax1.set_xlim(mins[0]-pad[0], maxs[0]+pad[0])
    ax1.set_ylim(mins[1]-pad[1], maxs[1]+pad[1])
    ax1.set_zlim(mins[2]-pad[2], maxs[2]+pad[2])

    if args.frame_step is not None:
        # draw reference frames along the path
        colors = ['r','g','b']
        length = 0.1  # 10cm axis
        for i in range(0, len(P), args.frame_step):
            R = rpy_to_rotmat(*RPY[i])
            origin = P[i]
            for j in range(3):
                vec = R[:, j] * length
                ax1.quiver(origin[0], origin[1], origin[2],
                        vec[0], vec[1], vec[2],
                        color=colors[j], linewidth=1, arrow_length_ratio=0.3)

    fig1.tight_layout()

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

    # ========== (3) attitude vs time ==========
    fig3, ax3 = plt.subplots(figsize=(6,4), dpi=150)
    ax3.plot(t, RPY[:,0], label='roll')
    ax3.plot(t, RPY[:,1], label='pitch')
    ax3.plot(t, RPY[:,2], label='yaw')
    ax3.set_xlabel('time [s]')
    ax3.set_ylabel('angle [rad]')
    ax3.set_title(f"{run_name} – Attitude vs Time")
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    fig3.tight_layout()

    # ========== (4) velocity vs time ==========
    fig4, ax4 = plt.subplots(figsize=(6,4), dpi=150)
    ax4.plot(t, V[:,0], label='vx')
    ax4.plot(t, V[:,1], label='vy')
    ax4.plot(t, V[:,2], label='vz')
    ax4.set_xlabel('time [s]')
    ax4.set_ylabel('velocity [m/s]')
    ax4.set_title(f"{run_name} – Velocity vs Time")
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    fig4.tight_layout()

    # ---------- show first ----------
    plt.show()

    # ---------- then save ----------
    out_prefix = run_name
    fig1.savefig(f"{out_prefix}_trajectory3d.png", bbox_inches='tight', dpi=300)
    fig2.savefig(f"{out_prefix}_position.png", bbox_inches='tight', dpi=300)
    fig3.savefig(f"{out_prefix}_attitude.png", bbox_inches='tight', dpi=300)
    fig4.savefig(f"{out_prefix}_velocity.png", bbox_inches='tight', dpi=300)
    print(f"Saved plots as: {out_prefix}_trajectory3d.png / _position.png / _attitude.png / _velocity.png")

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