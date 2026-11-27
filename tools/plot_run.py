#!/usr/bin/env python3
"""Plot node trajectories and progress from logs/node*.csv (written by --csv).

usage: python3 tools/plot_run.py [logs_dir] [-o out.png]
requires: matplotlib
"""
import argparse
import csv
import glob
import os

import matplotlib.pyplot as plt


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="?", default="logs")
    ap.add_argument("-o", "--out", default="swarm_run.png")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.logs, "node*.csv")))
    if not files:
        raise SystemExit(f"no node*.csv in {args.logs} - run scripts/run_demo.sh first")

    fig, (ax_xy, ax_prog) = plt.subplots(1, 2, figsize=(13, 6))
    for path in files:
        rows = load(path)
        if not rows:
            continue
        nid = rows[0]["id"]
        xs = [int(r["x_dm"]) / 10 for r in rows]
        ys = [int(r["y_dm"]) / 10 for r in rows]
        ax_xy.plot(xs, ys, marker=".", label=f"UAV-{nid}")
        ax_xy.plot(xs[-1], ys[-1], "kx")
        ts = [float(r["t_s"]) for r in rows]
        ax_prog.step(ts, [int(r["total"]) for r in rows], where="post", linestyle="--")
        ax_prog.plot(ts, [int(r["done"]) for r in rows], label=f"UAV-{nid} done")
        for prev, cur in zip(rows, rows[1:]):
            if cur["epoch"] != prev["epoch"]:
                ax_prog.axvline(float(cur["t_s"]), color="red", alpha=0.3)

    ax_xy.set(title="Trajectories (local frame)", xlabel="East [m]", ylabel="North [m]")
    ax_xy.set_aspect("equal")
    ax_xy.grid(True)
    ax_xy.legend()
    ax_prog.set(title="Route progress (dashed = assigned cells, red = re-allocation)",
                xlabel="time [s]", ylabel="cells")
    ax_prog.grid(True)
    ax_prog.legend()
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
