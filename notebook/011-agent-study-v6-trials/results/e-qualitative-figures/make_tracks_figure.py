#!/usr/bin/env python3
"""Qualitative R7 track figure for the paper (Sec. VI-C scene figure, panel c).

Two officially scored R7 flights on the EVAL pillar field, drawn from the
judge's own odometry capture and issued route:
  left  — A1 (AirStack scaffolded, closed loop), claude-opus-5 trial #1:
          PASS, min clearance 1.65 m
  right — A1, claude-opus-5 trial #5: FAIL — the fresh evaluation route
          was not followed in corridor order (two checkpoints never
          reached), although clearance was kept; the campaign's one
          fresh-route generalization failure

Answer-key note: the EVAL layout was withheld from every figure until the
campaign concluded (strategy choice 2026-08-28). Campaign v6 is complete
(40/40 scored, 2026-09-02); pass --withhold-eval-layout to draw only the
penetrated pillar if the layout must stay private for an add-on campaign.

Outputs: fig_r7_tracks.{pdf,png} next to this script.
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

HERE = Path(__file__).resolve().parent
STUDY = Path.home() / "Development/AirStack/agent_study"
FLY_Z = 0.3  # matches the judge's clearance check (ground samples ignored)

# (panel title, raw trial dir, official R7 artifact dir)
# Both flights are AirStack-arm scoring flights: the eval scene is staged
# into the Isaac workspace at judge time, so the pillars drawn here were
# physically present in the simulator. (Bare-parts-arm flights are NOT
# drawn: the judge never staged the eval world into that arm's Gazebo
# workspace, so their eval-layout clearance verdicts are not ground truth
# — see results_summary.md §(e).)
TRIALS = [
    ("A1 opus-5 #1: PASS", "A1_claude-opus-5_ladder_claude_001",
     "r5_artifacts_92124"),
    ("A1 opus-5 #5: FAIL (route order)", "A1_claude-opus-5_ladder_claude_005",
     "r5_artifacts_81731"),
]


def load_csv(p):
    pts = []
    for line in Path(p).read_text().splitlines():
        q = line.split(",")
        if len(q) >= 7:
            pts.append((float(q[4]), float(q[5]), float(q[6])))
    return pts


def parse_route(s):
    return [tuple(float(v) for v in w.split(",")) for w in s.split(";")]


def nearest_pillar(pts, pillars):
    best = (float("inf"), None)
    for x, y, z in pts:
        if z < FLY_Z:
            continue
        for p in pillars:
            if z <= p[3]:
                d = math.dist((x, y), (p[0], p[1])) - p[2]
                if d < best[0]:
                    best = (d, p)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--withhold-eval-layout", action="store_true")
    ap.add_argument("--out", default=str(HERE / "fig_r7_tracks"))
    args = ap.parse_args()

    layout = json.loads((STUDY / "obstacles/layout_r7.json").read_text())
    pillars = [(p["x"], p["y"], p["r"], p["h"]) for p in layout["pillars"]]

    plt.rcParams.update({"font.size": 7, "font.family": "serif"})
    fig, axes = plt.subplots(1, 2, figsize=(3.45, 1.95))
    for ax, (label, trial, art) in zip(axes, TRIALS):
        d = STUDY / "runs" / trial / art
        route = parse_route(json.loads((d / "r7_route.json").read_text())["route"])
        verdict = json.loads((d / "verdict.json").read_text())
        pts = load_csv(d / "odom.csv")
        mc, hit = nearest_pillar(pts, pillars)
        if (d / "r7_clearance.json").exists():
            clr = json.loads((d / "r7_clearance.json").read_text())
            assert abs(mc - clr["min_clearance_m"]) < 0.05, (mc, clr)
        ok = verdict["success"] and mc >= 1.0

        shown = pillars if not args.withhold_eval_layout else ([hit] if hit else [])
        for px, py, pr, ph in shown:
            ax.add_patch(Circle((px, py), pr, color="0.45", zorder=2))
            ax.add_patch(Circle((px, py), pr + 1.0, fill=False, color="0.75",
                                ls=":", lw=0.5, zorder=1))
        rx = [0.0] + [w[0] for w in route]
        ry = [0.0] + [w[1] for w in route]
        ax.plot(rx, ry, "k--", lw=0.7, alpha=0.7, zorder=3)
        ax.plot(rx[1:], ry[1:], "k*", ms=5, zorder=4)
        fly = [(x, y) for x, y, z in pts if z >= FLY_Z]
        col = "tab:blue" if ok else "tab:red"
        ax.plot([p[0] for p in fly], [p[1] for p in fly], color=col, lw=1.0,
                zorder=5)
        ax.plot(0, 0, "g^", ms=4, zorder=6)
        # mark checkpoints the corridor check found unreached, in order
        for w in verdict["waypoints"]:
            if not w["reached"]:
                ax.add_patch(Circle((w["target"][0], w["target"][1]), 2.2,
                                    fill=False, color="tab:red", lw=0.9,
                                    zorder=6))
        ax.set_title(f"{label}\nmin clearance {mc:.2f} m", fontsize=6.5)
        ax.set_aspect("equal")
        ax.set_xlim(-3, 53)
        ax.set_ylim(-8, 38)
        ax.set_xticks([0, 25, 50])
        ax.set_yticks([0, 25])
        ax.tick_params(labelsize=6, length=2, pad=1)
        ax.grid(alpha=0.2, lw=0.3)
    axes[0].set_ylabel("y (m)", labelpad=1)
    for ax in axes:
        ax.set_xlabel("x (m)", labelpad=1)
    fig.tight_layout(pad=0.3, w_pad=0.6)
    for ext in ("pdf", "png"):
        fig.savefig(f"{args.out}.{ext}", dpi=300)
        print("wrote", f"{args.out}.{ext}")


if __name__ == "__main__":
    main()
