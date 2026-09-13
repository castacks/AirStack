#!/usr/bin/env python3
"""Generate RRM-1_Tracker.xlsx — the project tracker, for Google Sheets or Excel.

Regenerate rather than hand-editing the workbook, so the schedule in
docs/roadmap.md and the metrics in docs/evaluation_matrix.md stay the source of
truth and the spreadsheet cannot silently drift from them.

    python3 scripts/make_tracker.py [-o RRM-1_Tracker.xlsx]

Upload the result to Google Drive and open with Google Sheets; formatting,
formulas and the frozen headers survive the import.
"""

from __future__ import annotations

import argparse
from datetime import date
from pathlib import Path

from openpyxl import Workbook
from openpyxl.formatting.rule import CellIsRule
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

PROJECT_START = date(2026, 8, 16)
SUBMISSION = date(2027, 3, 1)

# Palette matches docs/roadmap.md and the critical-path artifact.
INK = "1B1E1A"
MOSS = "5C6F52"
MOSS_WASH = "E6EBDF"
RISK = "9E4A3C"
RISK_WASH = "F2E2DE"
ACTIVE_WASH = "F6ECD8"
DONE_WASH = "DFEBE2"
GREY_WASH = "EDEEEB"

thin = Side(style="thin", color="D9DDD2")
BORDER = Border(bottom=thin)


def header(ws, row: int = 1) -> None:
    for cell in ws[row]:
        if cell.value is None:
            continue
        cell.font = Font(bold=True, color="FFFFFF", size=10)
        cell.fill = PatternFill("solid", fgColor=MOSS)
        cell.alignment = Alignment(vertical="center", wrap_text=True)
    ws.freeze_panes = ws.cell(row=row + 1, column=1)
    ws.row_dimensions[row].height = 30


def widths(ws, *pairs: tuple[str, int]) -> None:
    for col, w in pairs:
        ws.column_dimensions[col].width = w


def stripe(ws, first_row: int, last_row: int, last_col: int) -> None:
    for r in range(first_row, last_row + 1):
        for c in range(1, last_col + 1):
            ws.cell(row=r, column=c).border = BORDER
            ws.cell(row=r, column=c).alignment = Alignment(
                vertical="top", wrap_text=True
            )


# ---------------------------------------------------------------------------
# 1. Schedule
# ---------------------------------------------------------------------------

SCHEDULE = [
    # workstream, task, start, end, kind
    ("W1 Isaac backend", "Provision instance, verify environment", "2026-08-16", "2026-08-20", "Build"),
    ("W1 Isaac backend", "USD scene — Panda, table, cup, annotations", "2026-08-20", "2026-08-31", "Build"),
    ("W1 Isaac backend", "Deterministic episode reset", "2026-08-31", "2026-09-09", "Gate"),
    ("W1 Isaac backend", "observe() — poses, relation inference", "2026-09-05", "2026-09-14", "Build"),
    ("W1 Isaac backend", "apply() — trajectory publish, sim step", "2026-09-12", "2026-09-20", "Build"),
    ("W2 GR00T policy", "Load N1.7-3B, LIBERO_PANDA embodiment", "2026-09-21", "2026-09-25", "Build"),
    ("W2 GR00T policy", "EmbodimentAdapter — verb to instruction + pose", "2026-09-26", "2026-10-02", "Build"),
    ("W2 GR00T policy", "Observation dict, chunk to JointTrajectory", "2026-10-03", "2026-10-10", "Build"),
    ("W2 GR00T policy", "Numeric safety — URDF limits, swept volume", "2026-10-08", "2026-10-18", "Gate"),
    ("W3 Learned reasoner", "vLLM serving, guided_json smoke test", "2026-10-19", "2026-10-21", "Build"),
    ("W3 Learned reasoner", "Prompt generated from VERB_TABLE, versioned", "2026-10-22", "2026-10-27", "Build"),
    ("W3 Learned reasoner", "LocalReasoner — parser, validator, repair", "2026-10-28", "2026-11-03", "Build"),
    ("W3 Learned reasoner", "Oracle vs learned reasoner comparison", "2026-11-04", "2026-11-08", "Build"),
    ("W4 Tasks and metrics", "T3, T4, T7 — long-horizon and dynamic scenes", "2026-11-09", "2026-11-20", "Build"),
    ("W4 Tasks and metrics", "T5 ambiguity, T10 held-out layouts", "2026-11-21", "2026-11-28", "Build"),
    ("W4 Tasks and metrics", "Safety-labelled scenarios", "2026-11-23", "2026-12-02", "Gate"),
    ("W4 Tasks and metrics", "Close metric instrumentation gaps", "2026-11-30", "2026-12-06", "Build"),
    ("W5 Baseline arms", "Baseline A — VLA direct from mission", "2026-12-07", "2026-12-13", "Build"),
    ("W5 Baseline arms", "Baseline B — reasoner, no world model", "2026-12-14", "2026-12-20", "Build"),
    ("W5 Baseline arms", "Baseline C — world model, no predictive layer", "2026-12-21", "2026-12-27", "Slack"),
    ("W5 Baseline arms", "Arm-switching harness", "2026-12-28", "2027-01-03", "Build"),
    ("W6 Benchmark", "Seeded sweeps — N episodes x task x arm", "2027-01-04", "2027-01-17", "Build"),
    ("W6 Benchmark", "Ablations — world model, predictive, safety", "2027-01-18", "2027-01-24", "Build"),
    ("W6 Benchmark", "Analysis, results frozen", "2027-01-25", "2027-01-31", "Build"),
    ("W7 Paper", "Draft", "2027-02-01", "2027-02-14", "Write"),
    ("W7 Paper", "Figures generated from traces", "2027-02-08", "2027-02-18", "Write"),
    ("W7 Paper", "Review and revision", "2027-02-15", "2027-02-25", "Write"),
    ("W7 Paper", "IROS 2027 submission", "2027-02-25", "2027-03-01", "Gate"),
]


def sheet_schedule(wb: Workbook) -> None:
    ws = wb.create_sheet("Schedule")
    ws.append([
        "Workstream", "Task", "Start", "End", "Days", "Offset", "Kind",
        "Status", "% Done", "Notes",
    ])
    header(ws)

    for stream, task, start, end, kind in SCHEDULE:
        s, e = date.fromisoformat(start), date.fromisoformat(end)
        ws.append([
            stream, task, s, e,
            (e - s).days,
            (s - PROJECT_START).days,
            kind,
            "Not started",
            0,
            "",
        ])

    last = ws.max_row
    for r in range(2, last + 1):
        ws.cell(row=r, column=3).number_format = "yyyy-mm-dd"
        ws.cell(row=r, column=4).number_format = "yyyy-mm-dd"
        ws.cell(row=r, column=9).number_format = "0%"
        if ws.cell(row=r, column=7).value == "Gate":
            for c in range(1, 11):
                ws.cell(row=r, column=c).fill = PatternFill("solid", fgColor=RISK_WASH)
            ws.cell(row=r, column=2).font = Font(bold=True, color=RISK)

    stripe(ws, 2, last, 10)
    widths(ws, ("A", 22), ("B", 46), ("C", 12), ("D", 12), ("E", 7),
           ("F", 8), ("G", 9), ("H", 14), ("I", 9), ("J", 34))

    ws.cell(row=last + 2, column=1, value="Offset = days from project start; pair with Days to build a stacked-bar Gantt.").font = Font(italic=True, size=9, color="7C8277")
    ws.cell(row=last + 3, column=1, value="Gate = slippage reaches the submission date instead of being absorbed. See Gates tab.").font = Font(italic=True, size=9, color="7C8277")

    ws.conditional_formatting.add(
        f"H2:H{last}",
        CellIsRule(operator="equal", formula=['"Done"'],
                   fill=PatternFill("solid", fgColor=DONE_WASH)),
    )
    ws.conditional_formatting.add(
        f"H2:H{last}",
        CellIsRule(operator="equal", formula=['"In progress"'],
                   fill=PatternFill("solid", fgColor=ACTIVE_WASH)),
    )
    ws.conditional_formatting.add(
        f"H2:H{last}",
        CellIsRule(operator="equal", formula=['"Blocked"'],
                   fill=PatternFill("solid", fgColor=RISK_WASH)),
    )


# ---------------------------------------------------------------------------
# 2. Gates
# ---------------------------------------------------------------------------

GATES = [
    ("2026-08-31", "2026-09-09", "Deterministic episode reset", "W1",
     "Isaac physics is not reproducible by default and every reported number needs "
     "seeded episodes. Nothing in W4-W6 is publishable without it, which is why it "
     "precedes observe() despite being the less interesting task."),
    ("2026-10-08", "2026-10-18", "Swept-volume collision checking", "W2",
     "Safety #2 checks scalars against placeholder ranges today: structurally correct, "
     "numerically fake. Collision rate and safety violation rate are both meaningless "
     "until it is real. Most likely item to be underestimated."),
    ("2026-11-23", "2026-12-02", "Safety-labelled scenarios", "W4",
     "Verifier precision, recall and false-negative rate cannot come from traces — "
     "traces record what the verifier decided, never whether it was right. Scenarios "
     "must carry known-unsafe actions labelled in advance. Retrofitting invalidates "
     "every episode collected before the change."),
    ("2027-02-25", "2027-03-01", "IROS 2027 submission", "W7",
     "Four days of buffer is thin. Mitigation is freezing results on 31 January and "
     "refusing to re-run sweeps during write-up."),
]


def sheet_gates(wb: Workbook) -> None:
    ws = wb.create_sheet("Gates")
    ws.append(["Start", "End", "Gate", "Stream", "Why it blocks", "Status", "Risk notes"])
    header(ws)
    for start, end, name, stream, why in GATES:
        ws.append([
            date.fromisoformat(start), date.fromisoformat(end),
            name, stream, why, "Not started", "",
        ])
    last = ws.max_row
    for r in range(2, last + 1):
        ws.cell(row=r, column=1).number_format = "yyyy-mm-dd"
        ws.cell(row=r, column=2).number_format = "yyyy-mm-dd"
        ws.cell(row=r, column=3).font = Font(bold=True, color=RISK)
        ws.row_dimensions[r].height = 76
    stripe(ws, 2, last, 7)
    widths(ws, ("A", 12), ("B", 12), ("C", 30), ("D", 8), ("E", 74), ("F", 14), ("G", 30))


# ---------------------------------------------------------------------------
# 3. Metrics
# ---------------------------------------------------------------------------

METRICS = [
    # category, metric, definition, target, source, group, unblocked_by
    ("Task Success", "Task Success Rate", "Successful missions / total missions", ">85%", "task_success", "Live now", "-"),
    ("Task Success", "Partial Completion Rate", "Missions partially completed / total", ">90%", "goal predicate subsets", "New instrumentation", "W4"),
    ("Task Success", "Goal Verification Accuracy", "Correct identification of task completion", ">90%", "system must claim completion", "New instrumentation", "W4"),
    ("Reasoning", "Task Decomposition Accuracy", "Correct subtasks / expected subtasks", ">85%", "diff vs Oracle task graph", "Free from Oracle", "W3"),
    ("Reasoning", "Step Ordering Accuracy", "Correct ordering / total dependencies", ">90%", "diff vs Oracle ordering", "Free from Oracle", "W3"),
    ("Reasoning", "Spatial Reasoning Accuracy", "Correct spatial relationships / tested", ">90%", "dedicated probe set", "Needs ground truth", "W4"),
    ("Reasoning", "Temporal Reasoning Accuracy", "Correct temporal dependencies / tested", ">90%", "dedicated probe set", "Needs ground truth", "W4"),
    ("World Model", "Object-State Accuracy", "Correct object states / total objects", ">90%", "WorldState vs USD stage", "Unlocked by Isaac", "W1"),
    ("World Model", "Spatial State Accuracy", "Correct positions and relations", ">90%", "infer_relations vs stage", "Unlocked by Isaac", "W1"),
    ("World Model", "State Transition Accuracy", "Correct state updates after actions", ">90%", "divergence records", "Live now", "-"),
    ("World Model", "Memory Consistency", "Correct historical state retrieval", ">90%", "historical state queries", "New instrumentation", "W4"),
    ("Planning", "Plan Validity Rate", "Executable plans / generated plans", ">95%", "validator accept / total", "Free from Oracle", "W3"),
    ("Planning", "Planning Time", "Time to generate a valid plan", "Minimize", "planning_latency_ms", "Live now", "-"),
    ("Planning", "Action Efficiency", "Required actions / executed actions", ">85%", "Oracle action_count as optimum", "Free from Oracle", "W3"),
    ("Planning", "Replanning Rate", "Tasks requiring replanning / total", "<20%", "replans", "Live now", "-"),
    ("Execution", "Action Success Rate", "Successful actions / attempted", ">90%", "dispatch termination", "Live now", "-"),
    ("Execution", "Manipulation Success", "Successful grasps and placements / attempts", ">90%", "per-verb dispatch outcome", "Unlocked by Isaac", "W2"),
    ("Execution", "Navigation Success", "Successful navigation tasks / attempts", ">95%", "per-verb dispatch outcome", "Deferred", "Phase 6"),
    ("Execution", "Execution Time", "First action to completion", "Minimize", "simulator clock", "Unlocked by Isaac", "W1"),
    ("Safety", "Safety Violation Rate", "Unsafe actions / total actions", "~0%", "unsafe_actions / action_count", "Live now", "-"),
    ("Safety", "Collision Rate", "Collisions / total trials", "~0%", "Isaac contact reporting", "Unlocked by Isaac", "W2"),
    ("Safety", "Safety Verifier Recall", "Unsafe actions correctly rejected", ">99%", "labelled unsafe actions", "Needs ground truth", "W4 GATE"),
    ("Safety", "Safety Verifier Precision", "Rejected actions that were unsafe", ">95%", "labelled unsafe actions", "Needs ground truth", "W4 GATE"),
    ("Safety", "False-Negative Rate", "Unsafe actions incorrectly permitted", "<1%", "labelled unsafe actions", "Needs ground truth", "W4 GATE"),
    ("Recovery", "Failure Detection Rate", "Detected failures / injected failures", ">90%", "divergences / injected faults", "New instrumentation", "W4"),
    ("Recovery", "Recovery Success Rate", "Recovered / detected failures", ">80%", "recoveries / divergences", "Live now", "-"),
    ("Recovery", "Recovery Time", "Detection to successful recovery", "Minimize", "trace timestamps", "New instrumentation", "W4"),
    ("Recovery", "Recovery Action Overhead", "Additional actions after failure", "Minimize", "action_count delta", "New instrumentation", "W4"),
    ("Generalization", "Environment Generalization", "Performance on unseen environments", ">75% retained", "held-out scene library", "Deferred", "W4 partial"),
    ("Generalization", "Object Generalization", "Performance on unseen objects", ">75% retained", "held-out object set", "Deferred", "W4 partial"),
    ("Generalization", "Instruction Generalization", "Performance on novel phrasings", ">80% retained", "paraphrase set", "Needs ground truth", "W4"),
    ("Generalization", "Embodiment Transfer", "Performance on unseen embodiment", ">70% retained", "UNITREE_G1 adapter swap", "Deferred", "Post-IROS"),
    ("Robustness", "Sensor Robustness", "Performance under noise and dropout", ">80% retained", "real perception stack", "Deferred", "Phase 6"),
    ("Robustness", "Occlusion Robustness", "Performance under partial occlusion", ">80% retained", "real perception stack", "Deferred", "Phase 6"),
    ("Robustness", "Dynamic Environment Robustness", "Performance with moving obstacles", ">80% retained", "T6, T7 scenes", "Unlocked by Isaac", "W4"),
    ("Robustness", "Disturbance Recovery", "Recovery after perturbation", ">80%", "T7 scenes", "Unlocked by Isaac", "W4"),
    ("Sim-to-Real", "Sim Success Rate", "Success rate in simulation", ">85%", "task_success", "Live now", "-"),
    ("Sim-to-Real", "Real Success Rate", "Success rate on physical robot", ">75%", "physical robot", "Deferred", "Post-IROS"),
    ("Sim-to-Real", "Sim-to-Real Gap", "Simulation minus real performance", "<15%", "physical robot", "Deferred", "Post-IROS"),
    ("Efficiency", "Inference Latency", "Model inference per reasoning cycle", "Minimize", "model call timing", "New instrumentation", "W3"),
    ("Efficiency", "GPU Memory", "Peak GPU memory usage", "Minimize", "sampled during run", "Unlocked by Isaac", "W2"),
    ("Efficiency", "Compute Cost", "Compute per completed task", "Minimize", "wall clock x instance rate", "New instrumentation", "W6"),
    ("Efficiency", "Token / Model Calls", "AI calls required per task", "Minimize", "reasoner call counter", "New instrumentation", "W3"),
    ("Long-Horizon", "Success vs Task Horizon", "Success as action count increases", ">70% at 10+", "success binned by action count", "New instrumentation", "W4"),
    ("Long-Horizon", "State Consistency", "Correct world state throughout task", ">90%", "WorldState vs stage per step", "Unlocked by Isaac", "W1"),
    ("Long-Horizon", "Long-Horizon Recovery", "Recovery rate in multi-step tasks", ">80%", "recoveries filtered by horizon", "New instrumentation", "W4"),
]

GROUP_FILL = {
    "Live now": DONE_WASH,
    "Free from Oracle": MOSS_WASH,
    "Unlocked by Isaac": ACTIVE_WASH,
    "New instrumentation": GREY_WASH,
    "Needs ground truth": RISK_WASH,
    "Deferred": "FFFFFF",
}


def sheet_metrics(wb: Workbook) -> None:
    ws = wb.create_sheet("Metrics")
    ws.append([
        "Category", "Metric", "Definition", "Target", "Source",
        "Instrumentation group", "Unblocked by", "Current", "Measured on", "Notes",
    ])
    header(ws)
    for cat, name, definition, target, source, group, unblock in METRICS:
        ws.append([cat, name, definition, target, source, group, unblock, "", "", ""])

    last = ws.max_row
    for r in range(2, last + 1):
        group = ws.cell(row=r, column=6).value
        fill = GROUP_FILL.get(str(group), "FFFFFF")
        if fill != "FFFFFF":
            ws.cell(row=r, column=6).fill = PatternFill("solid", fgColor=fill)
        if "GATE" in str(ws.cell(row=r, column=7).value):
            ws.cell(row=r, column=7).font = Font(bold=True, color=RISK)
        ws.cell(row=r, column=9).number_format = "yyyy-mm-dd"

    stripe(ws, 2, last, 10)
    widths(ws, ("A", 15), ("B", 30), ("C", 40), ("D", 13), ("E", 30),
           ("F", 21), ("G", 13), ("H", 11), ("I", 13), ("J", 30))

    ws.cell(row=last + 2, column=1,
            value="Grouped by what unblocks each metric, which is what the schedule sequencing derives from.").font = Font(italic=True, size=9, color="7C8277")


# ---------------------------------------------------------------------------
# 4. Baselines
# ---------------------------------------------------------------------------

ARMS = [
    ("Oracle", "Ceiling on everything except reasoning. Not a baseline to beat — when RRM-1 trails it the gap is the reasoner, when it fails the bug is elsewhere."),
    ("Classical Robotics", "Scripted planner, no learned components."),
    ("LLM + Classical Planner", "Language to symbolic plan, classical motion."),
    ("VLM + Classical Planner", "Vision-language grounding, classical motion."),
    ("VLA (Baseline A)", "Mission plus camera straight to policy. No world model, no symbolic safety gate."),
    ("RRM without World Model (B)", "Reasoner and planner, stateless."),
    ("RRM without Predictive Planning (C)", "World model, no forward simulation of candidate actions."),
    ("RRM-1", "Full stack."),
]

CATEGORIES = ["Task Success", "Reasoning", "World Model", "Safety",
              "Recovery", "Generalization", "Efficiency"]


def sheet_baselines(wb: Workbook) -> None:
    ws = wb.create_sheet("Baselines")
    ws.append(["System"] + CATEGORIES + ["Weighted score", "Notes"])
    header(ws)
    for name, note in ARMS:
        ws.append([name] + [""] * len(CATEGORIES) + ["", note])

    last = ws.max_row
    for r in range(2, last + 1):
        if ws.cell(row=r, column=1).value in ("RRM-1", "Oracle"):
            ws.cell(row=r, column=1).font = Font(bold=True, color=MOSS)
        ws.row_dimensions[r].height = 32
    stripe(ws, 2, last, len(CATEGORIES) + 3)
    widths(ws, ("A", 34), ("I", 15), ("J", 62))
    for i in range(2, 2 + len(CATEGORIES)):
        ws.column_dimensions[get_column_letter(i)].width = 14

    ws.cell(row=last + 2, column=1,
            value="Do not fill the weighted score column until the weights are validated — see Weights tab.").font = Font(italic=True, size=9, color=RISK)


# ---------------------------------------------------------------------------
# 5. Weights
# ---------------------------------------------------------------------------

WEIGHTS = [
    ("Task Success", 0.25), ("Reasoning", 0.15), ("World Model", 0.15),
    ("Safety", 0.15), ("Recovery", 0.10), ("Generalization", 0.10),
    ("Efficiency", 0.10),
]


def sheet_weights(wb: Workbook) -> None:
    ws = wb.create_sheet("Weights")
    ws.append(["Category", "Weight", "Status"])
    header(ws)
    for cat, w in WEIGHTS:
        ws.append([cat, w, "Hypothesis — not validated"])
    last = ws.max_row
    ws.append(["Total", f"=SUM(B2:B{last})", ""])
    ws.cell(row=last + 1, column=1).font = Font(bold=True)
    ws.cell(row=last + 1, column=2).font = Font(bold=True)
    for r in range(2, last + 2):
        ws.cell(row=r, column=2).number_format = "0%"
    stripe(ws, 2, last + 1, 3)
    widths(ws, ("A", 22), ("B", 11), ("C", 30))

    note = (
        "These weights are a hypothesis. A single number computed from unvalidated "
        "weights invites a reviewer to reject the framing rather than engage with the "
        "result. Report the per-metric table until there is evidence about which "
        "metrics actually co-vary."
    )
    ws.cell(row=last + 3, column=1, value=note).font = Font(italic=True, size=9, color=RISK)
    ws.cell(row=last + 4, column=1,
            value="Safety is not tradeable: any hard violation fails the run regardless of task success."
            ).font = Font(italic=True, size=9, color=RISK)
    ws.cell(row=last + 6, column=1,
            value="Note: docs/evaluation_matrix.md targets >85% here while docs/benchmarks.md advises no composite. Resolve before publication."
            ).font = Font(italic=True, size=9, color="7C8277")


# ---------------------------------------------------------------------------
# 0. Overview
# ---------------------------------------------------------------------------

def sheet_overview(wb: Workbook) -> None:
    ws = wb.active
    ws.title = "Overview"

    ws["A1"] = "RRM-1 — Project Tracker"
    ws["A1"].font = Font(bold=True, size=17, color=INK)
    ws["A2"] = "Robotics Reasoning Model · github.com/o-abolade/rrm"
    ws["A2"].font = Font(size=10, color="7C8277")

    rows = [
        ("", ""),
        ("Anchor deadline", "IROS 2027 — 1 March 2027"),
        ("Project start", str(PROJECT_START)),
        ("Days to submission", (SUBMISSION - PROJECT_START).days),
        ("", ""),
        ("ICRA 2027 main track", "15 September 2026 — not a target; workshop outlet at best"),
        ("CoRL 2027", "~April 2027 — secondary"),
        ("", ""),
        ("Benchmark tasks", "5 of 10 implemented"),
        ("Metrics instrumented", "10 of 45"),
        ("Hardware", "A10G · 22.35 GiB usable VRAM · 8 vCPU · 32 GiB RAM"),
        ("", ""),
        ("Tabs", "Schedule · Gates · Metrics · Baselines · Weights"),
    ]
    r = 3
    for label, value in rows:
        ws.cell(row=r, column=1, value=label).font = Font(bold=True, size=10)
        ws.cell(row=r, column=2, value=value)
        r += 1

    ws.cell(row=r + 1, column=1,
            value="Regenerate with: python3 scripts/make_tracker.py").font = Font(italic=True, size=9, color="7C8277")
    ws.cell(row=r + 2, column=1,
            value="Source of truth is docs/roadmap.md and docs/evaluation_matrix.md — edit those, then regenerate."
            ).font = Font(italic=True, size=9, color="7C8277")
    widths(ws, ("A", 24), ("B", 68))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-o", "--output", type=Path, default=Path("RRM-1_Tracker.xlsx"))
    args = ap.parse_args()

    wb = Workbook()
    sheet_overview(wb)
    sheet_schedule(wb)
    sheet_gates(wb)
    sheet_metrics(wb)
    sheet_baselines(wb)
    sheet_weights(wb)
    wb.save(args.output)

    print(f"wrote {args.output}  ({args.output.stat().st_size / 1024:.0f} KB)")
    print(f"  tabs: {', '.join(wb.sheetnames)}")
    print(f"  {len(SCHEDULE)} tasks, {len(GATES)} gates, {len(METRICS)} metrics, {len(ARMS)} arms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
