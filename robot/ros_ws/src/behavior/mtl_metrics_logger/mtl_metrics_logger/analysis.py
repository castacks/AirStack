"""Score recorded telemetry and write the run outputs (pure Python, no ROS).

Used twice with the same code path:

* per robot, by ``mtl_metrics_logger`` at the end of its sortie
  (``runs/<run_id>/<robot>/``), scoring that agent alone;
* per team, by ``scripts/analyze_mtl_run.py`` (``runs/<run_id>/``), fusing
  every agent's ``telemetry.csv`` on one timeline, so the joint miss product
  and the responsible agent are computed across the team.

Coordinates: everything here is WORLD ENU (x = East = e, y = North = n).

Headline metric: the RESIDUAL BELIEF MASS, ``P(target missed by the search)``
(lower is better; see ``detection.py``), computed from the flown looks over the
whole normalised prior. When the planned tracks carry their boresight schedule
(``planned_by_agent[name]["looks"]``) the PLANNED residual is scored the same way,
so plan and flight can be compared in one unit.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from mtl_metrics_logger.detection import (DetectionModel, PriorGrid, TeamScorer, planned_residual,
                                          prior_from_scenario, resample_hold)
from mtl_metrics_logger.report import (build_report_data, residual_map_payload, write_json, write_report,
                                       write_residual_csv, write_telemetry_csv)

__all__ = ["targets_world", "cells_world", "area_world", "score_agents", "write_run_outputs",
           "planned_looks_from_track", "RESIDUAL_CSV_BLOCK_M"]

#: Block edge of ``residual_belief.csv`` [m] (10 m, as ``mtl_demo --residual-block 10`` at 1 m).
RESIDUAL_CSV_BLOCK_M = 10.0


def targets_world(ground_truth: Mapping[str, Any]) -> list[tuple[float, float, float]]:
    return [(float(t["e"]), float(t["n"]), 0.0) for t in ground_truth.get("targets", [])]


def cells_world(scenario: Mapping[str, Any]) -> tuple[list[tuple[float, float]], list[float]]:
    c = scenario["cells"]
    return [(float(e), float(n)) for n, e in c["centers"]], [float(m) for m in c["mass"]]


def area_world(scenario: Mapping[str, Any], margin_m: float = 40.0,
               include: Sequence[Sequence[float]] = ()) -> dict[str, Any]:
    area = scenario["mission"]["area"]
    half = float(area["size_m"]) / 2.0
    cn, ce = (float(v) for v in area.get("center_ned", (0.0, 0.0)))
    box = {"x_min": ce - half, "x_max": ce + half, "y_min": cn - half, "y_max": cn + half}
    xs = [box["x_min"], box["x_max"]] + [p[0] for p in include]
    ys = [box["y_min"], box["y_max"]] + [p[1] for p in include]
    res = float(area.get("belief_res_m", 2.0))
    return {"x_min": min(xs) - margin_m, "x_max": max(xs) + margin_m,
            "y_min": min(ys) - margin_m, "y_max": max(ys) + margin_m,
            "box": box, "belief": {k: v for k, v in box.items()},
            "belief_res_m": res, "cell_size": float(scenario["mapping"]["target_cell_size_m"])}


def _median(vals: Sequence[float], default: float) -> float:
    v = sorted(x for x in vals if x > 0)
    return v[len(v) // 2] if v else default


def planned_looks_from_track(track: Mapping[str, Any], home: Sequence[float] | None = None) -> dict[str, list]:
    """``<robot>/track.json`` (mtl_search_planner) -> planned looks in world ENU for
    :func:`~mtl_metrics_logger.detection.planned_residual`."""
    s = track["samples"]
    hx, hy, hz = (list(home) + [0.0, 0.0, 0.0])[:3] if home is not None else track.get("home_enu", [0.0, 0.0, 0.0])
    n = len(s["t"])
    return {"t": list(s["t"]),
            "pos": [(s["x_map"][k] + hx, s["y_map"][k] + hy, s["z_map"][k] + hz) for k in range(n)],
            "bore": [(s["bx_map"][k] + hx, s["by_map"][k] + hy, s["bz_map"][k] + hz) for k in range(n)]}


def score_agents(rows_by_agent: Mapping[str, Sequence[Mapping[str, Any]]], scenario: Mapping[str, Any],
                 ground_truth: Mapping[str, Any], *, use_command_fallback: bool = True,
                 dt: float | None = None, prior: PriorGrid | None = None,
                 residual_snapshot_s: float | None = None
                 ) -> tuple[TeamScorer, dict[str, dict[str, list]], dict[str, float]]:
    """Fuse every agent's telemetry rows on one hold-last-value timeline.

    Rows need ``t``, ``x_world``, ``y_world``, ``z_world`` and the measured gimbal
    ``meas_pitch``/``meas_yaw`` (falling back to ``cmd_*`` when a sample has no
    measurement, if ``use_command_fallback``). Returns the scorer, per-agent
    resampled series for the report, and the per-agent measured fraction.
    The scorer also accumulates the residual belief over ``prior`` (default: the
    scenario's own prior, :func:`~mtl_metrics_logger.detection.prior_from_scenario`).
    """
    model = DetectionModel.from_scenario(scenario["sensor"]["detection"])
    fov = math.radians(float(scenario["sensor"]["fov_deg"]))
    cells, masses = cells_world(scenario)
    if prior is None:
        prior = prior_from_scenario(scenario)
    scorer = TeamScorer(targets_world(ground_truth), cells, masses, model, fov, prior=prior,
                        residual_snapshot_s=residual_snapshot_s)

    tracks: dict[str, dict[str, list]] = {}
    measured_fraction: dict[str, float] = {}
    for name, rows in rows_by_agent.items():
        rows = sorted((r for r in rows if r.get("t") is not None and r.get("x_world") is not None),
                      key=lambda r: r["t"])
        if not rows:
            continue
        pitch, yaw, meas = [], [], 0
        for r in rows:
            if r.get("meas_pitch") is not None and r.get("meas_yaw") is not None and r.get("gimbal_measured", 1):
                pitch.append(r["meas_pitch"])
                yaw.append(r["meas_yaw"])
                meas += 1
            elif use_command_fallback:
                pitch.append(r.get("cmd_pitch"))
                yaw.append(r.get("cmd_yaw"))
            else:
                pitch.append(None)
                yaw.append(None)
        measured_fraction[name] = meas / len(rows)
        tracks[name] = {
            "t": [r["t"] for r in rows],
            "pos": [(r["x_world"], r["y_world"], r["z_world"]) for r in rows],
            "pitch": pitch, "yaw": yaw,
            "xte": [r.get("xte_m") for r in rows],
            "pointing_error": [r.get("pointing_error_m") for r in rows],
            "cmd_pitch": [r.get("cmd_pitch") for r in rows],
            "meas_pitch": [r.get("meas_pitch") for r in rows],
        }
    if not tracks:
        return scorer, {}, measured_fraction

    if dt is None:
        dt = min(_median([b - a for a, b in zip(tr["t"], tr["t"][1:])], 0.05) for tr in tracks.values())
    t0 = min(tr["t"][0] for tr in tracks.values())
    t1 = max(tr["t"][-1] for tr in tracks.values())
    n = max(1, int(math.floor((t1 - t0) / dt + 1e-9)) + 1)
    timeline = [t0 + k * dt for k in range(n)]

    held = {}
    for name, tr in tracks.items():
        held[name] = {k: resample_hold(tr["t"], tr[k], timeline)
                      for k in ("pos", "pitch", "yaw", "xte", "pointing_error", "cmd_pitch", "meas_pitch")}
    for k, t in enumerate(timeline):
        samples = {}
        for name, tr in tracks.items():
            if t < tr["t"][0] or t > tr["t"][-1] + 1e-9:
                continue  # before take-over / after the agent finished: not looking
            h = held[name]
            samples[name] = {"pos": h["pos"][k], "pitch": h["pitch"][k], "yaw": h["yaw"][k], "ground_z": 0.0}
        scorer.step(t - t0, samples, dt if k else 0.0)

    per_agent = {name: {"t": [t - t0 for t in timeline], "xte": held[name]["xte"],
                        "pointing_error": held[name]["pointing_error"],
                        "cmd_pitch": held[name]["cmd_pitch"], "meas_pitch": held[name]["meas_pitch"]}
                 for name in tracks}
    return scorer, per_agent, measured_fraction


def write_run_outputs(out_dir: str | Path, *, scenario: Mapping[str, Any], ground_truth: Mapping[str, Any],
                      rows_by_agent: Mapping[str, Sequence[Mapping[str, Any]]],
                      planned_by_agent: Mapping[str, Mapping[str, Any]], title: str, subtitle: str,
                      belief_png: bytes | None = None, write_telemetry: bool = True,
                      extra: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Score and write ``telemetry.csv``, ``detection.json``, ``residual_belief.csv`` and ``report.html``.

    ``planned_by_agent[name]``: ``{"planned": [[x, y]...] (world), "home": [x, y],
    "serviced_cells": [...], "planned_length_m": ..., "looks": {"t", "pos", "bore"} (optional,
    world ENU; see :func:`planned_looks_from_track`)}``.
    """
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    prior = prior_from_scenario(scenario)
    scorer, per_agent, measured = score_agents(rows_by_agent, scenario, ground_truth, prior=prior)

    planned_rb = None
    looks = [p["looks"] for p in planned_by_agent.values() if p.get("looks")]
    if prior is not None and looks:
        planned_rb = planned_residual(prior, scorer.model, scorer.fov, looks)

    cells, masses = cells_world(scenario)
    planned_cells = sorted({int(c) for p in planned_by_agent.values() for c in p.get("serviced_cells", [])
                            if 0 <= int(c) < len(cells)})
    planned_mass = sum(masses[c] for c in planned_cells) if planned_cells else None

    summary = scorer.summary()
    summary["agents"] = sorted(rows_by_agent)
    summary["gimbal_measured_fraction"] = {k: round(v, 4) for k, v in sorted(measured.items())}
    summary["planned_cells"] = len(planned_cells)
    summary["planned_belief_mass"] = round(planned_mass, 3) if planned_mass is not None else None
    if planned_mass:
        summary["realized_over_planned_mass"] = round(scorer.covered_mass / planned_mass, 4)
    summary["planned_residual_belief_mass"] = (round(planned_rb.exact_mass(), 6)
                                               if planned_rb is not None else None)
    targets = scorer.target_table()
    gt = ground_truth.get("targets", [])
    for row, g in zip(targets, gt):
        row["n"], row["e"] = g.get("n"), g.get("e")
    detection = {
        "schema": "mtl.detection/1",
        "scenario": scenario["mission"]["name"],
        "frame": "x/y = world ENU (x = e, y = n); n/e = mission NED",
        "model": {"name": "Moon et al. (2022) sigmoid", **scorer.model.__dict__,
                  "fov_deg": math.degrees(scorer.fov),
                  "rate_normalisation": "P_miss *= (1 - P)^(dt/dt_ref)",
                  "residual_belief": "residual(x) = prior(x) * prod_looks (1 - P(z|x))^(dt/dt_ref) over "
                                     "every pixel of the prior normalised to 1; residual_belief_mass = "
                                     "sum_x residual(x) = P(target missed), lower is better"},
        "summary": summary,
        "targets": targets,
        "curves": {"t_s": [round(v, 3) for v in scorer.t], "targets_detected": scorer.detected_count,
                   "team_distance_m": [round(v, 3) for v in scorer.distance_curve],
                   "covered_mass": [round(v, 6) for v in scorer.mass_curve],
                   "residual_mass": [None if v is None else round(v, 6) for v in scorer.residual_curve]},
        **(dict(extra) if extra else {}),
    }
    write_json(out / "detection.json", detection)
    if scorer.residual is not None:
        block = max(1, int(round(RESIDUAL_CSV_BLOCK_M / scorer.residual.prior.res)))
        write_residual_csv(out / "residual_belief.csv", scorer.residual.blocks(block))

    if write_telemetry:
        merged = [dict(r, agent=name) for name, rows in sorted(rows_by_agent.items()) for r in rows]
        merged.sort(key=lambda r: (r.get("t") or 0.0, r.get("agent")))
        write_telemetry_csv(out / "telemetry.csv", merged)

    covered = set(i for i, c in enumerate(scorer.cell_covered) if c)
    agents = []
    homes = []
    for name in sorted(set(rows_by_agent) | set(planned_by_agent)):
        rows = rows_by_agent.get(name, [])
        p = planned_by_agent.get(name, {})
        home = p.get("home")
        if home:
            homes.append(home)
        agents.append({"name": name, "home": home, "planned": p.get("planned", []),
                       "flown": [(r["x_world"], r["y_world"]) for r in rows if r.get("x_world") is not None],
                       "bore": [(r["bore_x_world"], r["bore_y_world"]) for r in rows
                                if r.get("bore_x_world") is not None]})
    data = build_report_data(
        title=title, subtitle=subtitle, summary=summary, targets=targets,
        area=area_world(scenario, include=homes),
        cells=[{"x": c[0], "y": c[1], "mass": m, "covered": i in covered, "planned": i in planned_cells}
               for i, (c, m) in enumerate(zip(cells, masses))],
        agents=agents,
        curves={"t": scorer.t, "detected": scorer.detected_count, "distance": scorer.distance_curve,
                "mass": scorer.mass_curve, "residual": scorer.residual_curve},
        planned_mass=planned_mass, per_agent=per_agent, belief_png=belief_png,
        residual_map=residual_map_payload(scorer.residual) if scorer.residual is not None else None,
        notes=[
            "Residual belief mass = P(target missed): every pixel of the prior (normalised to sum to 1) "
            "gets the same miss update as a target standing there; lower is better. "
            + ("The planned value scores the planned track and boresight schedule the same way."
               if planned_rb is not None else ""),
            "Scored from the flown pose and the MEASURED gimbal state "
            "(commanded angles substitute only where no measurement arrived; see gimbal_measured_fraction).",
            f"Detection: Moon et al. (2022) sigmoid a={scorer.model.a}, b={scorer.model.b}, c={scorer.model.c}, "
            f"beta={scorer.model.beta} m; found at P_det >= {scorer.model.threshold}; "
            f"per-look rate normalised to dt_ref = {scorer.model.dt_ref_s} s.",
        ])
    write_report(out / "report.html", data)
    return {"summary": summary, "detection": detection, "scorer": scorer}
