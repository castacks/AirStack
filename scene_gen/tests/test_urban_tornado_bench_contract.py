"""Offline contract checks for the urban-tornado review bench."""

import ast
from pathlib import Path


LAUNCHER = (Path(__file__).parents[2] / "simulation" / "isaac-sim" /
            "launch_scripts" / "urban_tornado_bench_launch_script.py")


def _assignments():
    tree = ast.parse(LAUNCHER.read_text())
    return {node.targets[0].id: node.value for node in tree.body
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)}


def test_default_cells_exclude_retired_c_row():
    value = _assignments()["DEFAULT_CELL_ORDER"]
    assert ast.literal_eval(value) == [
        "A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4", "B5"]


def test_a_row_has_explicit_review_orientation():
    cells = _assignments()["CELLS"]
    rows = {}
    for key, value in zip(cells.keys, cells.values):
        cid = ast.literal_eval(key)
        if cid not in {"A1", "A2", "A3", "A4"}:
            continue
        row = {ast.literal_eval(k): ast.literal_eval(v)
               for k, v in zip(value.keys, value.values)
               if isinstance(v, ast.Constant)}
        rows[cid] = (row["seat"], row["cell_bearing"])
    assert rows == {"A1": (180.0, 180.0), "A2": (180.0, 180.0),
                    "A3": (180.0, 180.0), "A4": (90.0, 270.0)}


def test_seed_index_is_independent_of_filtered_cell_order():
    source = LAUNCHER.read_text()
    fn = next(n for n in ast.parse(source).body
              if isinstance(n, ast.FunctionDef) and n.name == "_seed_for")
    body = ast.unparse(fn)
    assert "CELL_SEED_INDEX.get" in body
    assert "CELL_ORDER.index" not in body
