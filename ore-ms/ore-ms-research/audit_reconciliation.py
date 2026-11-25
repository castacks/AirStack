"""Cross-check logged M01 results and task traceability against their artifacts."""
from collections import Counter
import csv
import json
from pathlib import Path
import re
import sys


def read_csv(path):
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


def main(capture, log):
    config = json.loads(Path("scene.json").read_text())
    pixels = read_csv(capture)
    counts = Counter(row["case"] for row in pixels)
    visible = Counter(row["case"] for row in pixels if row["target_visible"] == "1")
    assert counts == {"clear": 221, "attenuated": 221}
    assert set(counts) == set(config["cases"]) and sum(counts.values()) == 442
    assert visible == {"clear": 197, "attenuated": 197}
    assert all(count == config["width"] * config["height"] for count in counts.values())
    print(f"442 observations; per case: {dict(counts)}; target visible: {dict(visible)}")

    verification = log.read_text()
    assert len(re.findall(r"^test_\w+", verification, re.MULTILINE)) == 5
    assert "Ran 5 tests" in verification and verification.strip().endswith("OK")
    integrals = re.findall(r"reference=([\d.]+) errors=\[[^\]]+, ([\d.e+-]+)\]", verification)
    assert len(integrals) == 4
    worst = max(100 * float(error) / float(reference) for reference, error in integrals)
    assert round(worst, 5) == 0.00068
    print(f"5 passing tests; maximum relative LWIR integration error {worst:.9f}%")

    tasks = read_csv(Path("14_WORK_BREAKDOWN_STRUCTURE.csv"))
    edges = read_csv(Path("17_TASK_DEPENDENCIES.csv"))
    by_id = {row["ID"]: row for row in tasks}
    assert len(tasks) == len(by_id) == 105
    workstreams = Counter(row["Workstream"] for row in tasks)
    assert workstreams == {"MATH": 25, "PHY": 30, "ML": 30, "INT": 20}
    milestones = {f"M{i:02}" for i in range(1, 14)}
    assert {row["Milestone"] for row in tasks} == milestones
    roadmap = Path("RESEARCH_ROADMAP_MANDATE.md").read_text()
    assert len(re.findall(r"^\*\*Phase [IVX]+", roadmap, re.MULTILINE)) == 5
    assert set(re.findall(r"^\| (M\d{2}) \| Pending \| Not Started", roadmap, re.MULTILINE)) == milestones - {"M01"}
    assert "| M01 | Authorized | Verification |" in roadmap
    assert all(row["Disposition"] == "PLANNED" for row in tasks if row["Milestone"] != "M01")
    dependencies = {key: set() for key in by_id}
    for edge in edges:
        task, predecessor = edge["Task"], edge["Depends on"]
        assert by_id[predecessor]["Milestone"] <= by_id[task]["Milestone"]
        dependencies[task].add(predecessor)
    assert len(edges) == 147 == sum(map(len, dependencies.values()))
    while dependencies:
        ready = {key for key, needs in dependencies.items() if not needs}
        assert ready, "cyclic task dependencies"
        dependencies = {key: needs - ready for key, needs in dependencies.items() if key not in ready}
    for task in tasks:
        assert all(task[field] for field in (
            "Scientific objective", "Verification methodology", "Acceptance evidence"))
    assert [p.name[:2] for p in sorted(Path(".").glob("[0-9][0-9]_*"))] == [
        f"{i:02}" for i in range(27)]
    print(f"105 tasks {dict(workstreams)}, 147 acyclic milestone-ordered edges, "
          "13 milestones, 5 phases, 27 numbered deliverables")


if __name__ == "__main__":
    main(*(Path(arg) for arg in sys.argv[1:]))
