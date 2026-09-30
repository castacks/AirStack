#!/usr/bin/env python3
"""Residual belief of a planned track, the metric every planner is compared on.

    python3 scripts/planner_benchmark/score.py --scenario S.json --track robot_1_track.json [--res 10]

A numpy port of ``mtl::eval::computeResidualBelief`` (and of the logger's ``planned_residual``
at dt = dt_ref): every track sample is one look at its scheduled boresight point; the footprint
is a disc of radius slant_to_boresight * tan(fov/2); a pixel in it at 3-D distance d gets
``log(1 - P(d))`` added, P = 1 / (a + exp(b (d - c))) up to beta and p_out_of_range past it;
residual = sum prior * exp(sum log-miss). Lower is better.

Also returns the residual after 5 %, 10 %, ... of the flown distance (the "anytime" curve), so
planners that search the most belief early can be told apart from ones that only catch up.
Validated against the C++ metric to <1e-3 on the flown runs (see README).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "robot/ros_ws/src/local/controls/mtl_trajectory_follower"))
from priors import rasterize  # noqa: E402

# The gimbal every planner's plan is flown through in "hardware" scoring: the Isaac scene's
# gimbal (stacks/*/config/mission.yaml sim_gimbal) driven by mtl_trajectory_follower's law.
HW = {"slew_rate_deg_s": 120.0, "roll_limit_deg": 80.0, "pitch_limit_deg": (-20.0, 110.0),
      "gimbal_max_deg": 80.0, "pitch_nudge_max_deg": 5.0}

CURVE_FRACS = [round(0.05 * k, 2) for k in range(1, 21)]


def scenario_prior(sc: dict, res: float):
    a = sc["mission"]["area"]
    size = float(a["size_m"])
    cn, ce = a.get("center_ned", [0.0, 0.0])
    area = (cn - size / 2, cn + size / 2, ce - size / 2, ce + size / 2)
    bel = sc["airstack"]["belief"]
    return rasterize(bel["bumps"], float(bel.get("base_uncertainty", 0.0)), area, res=res,
                     cap=float(bel.get("belief_cap", 0.85)))


def looks_from_track(track: dict, hardware: dict | None = None):
    """(n, e, h) of the aircraft and (n, e) of the boresight per sample, mission NED.

    ``hardware=None``: the PLANNED boresight. Otherwise the boresight a real gimbal achieves when
    the follower points it at the planned one: the single-axis mount law of
    mtl_trajectory_follower (cross-track angle clamped to ``gimbal_max_deg``, airframe pitch
    nudge clamped to ``pitch_nudge_max_deg``), each earth-frame axis slew-limited to
    ``slew_rate_deg_s`` and clamped to the roll / pitch limits. The same model for every planner,
    so no plan is credited with gimbal motion the hardware cannot make.
    """
    s = track["samples"]
    hz = float((track.get("home_enu") or [0, 0, 0])[2])
    n, e = np.asarray(s["n"], float), np.asarray(s["e"], float)
    h = np.asarray(s["z_map"], float) + hz
    bn, be = np.asarray(s["sensor_n"], float), np.asarray(s["sensor_e"], float)
    if hardware is None:
        return n, e, h, bn, be
    from mtl_trajectory_follower.gimbal_math import boresight_from_euler, single_axis_command, slew_limit
    hw = dict(HW, **hardware)
    dt = float(track.get("dt_s", 0.1))
    tilt = float(track.get("tilt_rad", math.radians(30.0)))
    gmax, nudge = math.radians(hw["gimbal_max_deg"]), math.radians(hw["pitch_nudge_max_deg"])
    step = math.radians(hw["slew_rate_deg_s"]) * dt
    rl, (p0, p1) = math.radians(hw["roll_limit_deg"]), (math.radians(v) for v in hw["pitch_limit_deg"])
    yaw = np.asarray(s.get("yaw_enu") or np.zeros(n.size), float)
    obn, obe = bn.copy(), be.copy()
    state = None
    for k in range(n.size):
        pos = (e[k], n[k], h[k])  # ENU
        cmd, _ = single_axis_command(pos, (be[k], bn[k], 0.0), float(yaw[k]), tilt, gmax, nudge)
        state = cmd if state is None else slew_limit(state, cmd, step)
        roll, pitch, yw = state
        state = (max(-rl, min(rl, roll)), max(p0, min(p1, pitch)), yw)
        b = boresight_from_euler(state[1], state[2])
        if b[2] < -1e-6:
            t = h[k] / -b[2]
            obe[k], obn[k] = e[k] + t * b[0], n[k] + t * b[1]
        else:  # at or above the horizon: no ground look
            obe[k], obn[k] = np.nan, np.nan
    return n, e, h, obn, obe


def hardware_for(track: dict, **over) -> dict:
    """The mount limits a track's planner declares (TIGRIS writes them; MTL uses the follower's)."""
    hw = {}
    tg = track.get("tigris") or {}
    if tg:
        hw["gimbal_max_deg"] = math.degrees(float(tg.get("gimbal_max_rad", math.radians(80.0))))
        hw["pitch_nudge_max_deg"] = math.degrees(float(tg.get("pitch_nudge_max_rad", 0.0)))
    hw.update(over)
    return hw


def residual(sc: dict, tracks: list[dict], res: float = 10.0, prior=None, hardware=None) -> dict:
    """``hardware``: None (planned boresight) or a dict of overrides for :data:`HW`, applied on
    top of each track's own declared mount limits (:func:`hardware_for`)."""
    na, ea, V = prior if prior is not None else scenario_prior(sc, res)
    det = sc["sensor"]["detection"]
    A, B, C, beta = float(det["a"]), float(det["b"]), float(det["c"]), float(det["beta"])
    log_out = math.log1p(-float(det.get("p_out_of_range", 1e-6)))
    tan_half = math.tan(math.radians(float(sc["sensor"]["fov_deg"])) / 2.0)
    n0, e0, r = float(na[0]), float(ea[0]), float(na[1] - na[0])
    L = np.zeros_like(V)

    total_len, out_curve = 0.0, []
    seqs = [looks_from_track(t, None if hardware is None else hardware_for(t, **hardware)) for t in tracks]
    lens = [float(np.hypot(np.diff(q[0]), np.diff(q[1])).sum()) for q in seqs]
    team_len = sum(lens)
    marks = [f * team_len for f in CURVE_FRACS]
    mi = 0
    for (n, e, h, bn, be) in seqs:
        N = n.size
        k = 0
        while k < N:
            run = 1  # identical consecutive states (hover padding): integrate once, weighted
            while k + run < N and n[k + run] == n[k] and e[k + run] == e[k] and h[k + run] == h[k] \
                    and bn[k + run] == bn[k] and be[k + run] == be[k]:
                run += 1
            pn, pe, ph, ln, le = n[k], e[k], h[k], bn[k], be[k]
            if ln != ln or le != le:  # no ground look this sample
                if k > 0:
                    total_len += math.hypot(n[k] - n[k - 1], e[k] - e[k - 1])
                k += run
                continue
            rad = math.sqrt((pn - ln) ** 2 + (pe - le) ** 2 + ph * ph) * tan_half
            i1, i2 = max(0, math.ceil((ln - rad - n0) / r)), min(na.size - 1, math.floor((ln + rad - n0) / r))
            j1, j2 = max(0, math.ceil((le - rad - e0) / r)), min(ea.size - 1, math.floor((le + rad - e0) / r))
            if i1 <= i2 and j1 <= j2:
                sn, se = na[i1:i2 + 1], ea[j1:j2 + 1]
                dn2 = (sn - ln) ** 2
                de2 = (se - le) ** 2
                inside = dn2[:, None] + de2[None, :] <= rad * rad
                d3 = np.sqrt((sn - pn)[:, None] ** 2 + (se - pe)[None, :] ** 2 + ph * ph)
                with np.errstate(over="ignore"):
                    lm = np.where(d3 > beta, log_out, np.log1p(-1.0 / (A + np.exp(B * (d3 - C)))))
                L[i1:i2 + 1, j1:j2 + 1] += np.where(inside, run * lm, 0.0)
            if k > 0:
                total_len += math.hypot(n[k] - n[k - 1], e[k] - e[k - 1])
            while mi < len(marks) and total_len >= marks[mi] - 1e-6:
                out_curve.append(float((V * np.exp(L)).sum()))
                mi += 1
            k += run
    resid = float((V * np.exp(L)).sum())
    while len(out_curve) < len(marks):
        out_curve.append(resid)
    return {"residual": resid, "flown_m": team_len, "curve": [round(v, 6) for v in out_curve]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", type=Path, required=True)
    ap.add_argument("--track", type=Path, action="append", required=True, help="repeat for a team")
    ap.add_argument("--res", type=float, default=10.0)
    ap.add_argument("--hardware", action="store_true", help="score the boresight a slew-limited gimbal achieves")
    a = ap.parse_args(argv)
    sc = json.loads(a.scenario.read_text())
    tr = [json.loads(t.read_text()) for t in a.track]
    out = residual(sc, tr, a.res, hardware={} if a.hardware else None)
    print(json.dumps({"residual": out["residual"], "flown_m": out["flown_m"]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
