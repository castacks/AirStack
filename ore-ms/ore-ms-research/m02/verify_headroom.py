"""Audit the separately frozen detector-headroom diagnostic."""
import csv
import json
from pathlib import Path
import sys


def table(root, filename):
    with (root / filename).open(newline="") as source:
        return list(csv.DictReader(source))


def main(primary, diagnostic):
    base = Path(__file__).parent
    initial = json.loads((base / "scene.json").read_text())
    modified = json.loads((base / "headroom_scene.json").read_text())
    assert (initial.pop("full_well_e"), modified.pop("full_well_e")) == (1800, 4000)
    assert initial == modified
    first, second = (table(run, "pixels.csv") for run in (primary, diagnostic))
    fields = ("case", "row", "column", "channel", "boundary_W_m2_sr",
              "direct_W_m2_sr", "path_W_m2_sr")
    assert len(first) == len(second) == 26136
    assert all(all(a[key] == b[key] for key in fields) for a, b in zip(first, second, strict=True))
    metrics = {row["case"]: row for row in table(diagnostic, "metrics.csv")}
    assert float(metrics["clear"]["high_clip_fraction"]) == 0
    print("PASS: all radiance fields identical and clear capture has zero upper clipping.")
    for case in ("clear", "out_scattering", "isotropic_path", "forward_path"):
        values = metrics[case]
        print(case, *(f"{key}={float(values[key]):.8f}"
                      for key in ("contrast_G", "SSIM", "radiance_RMSE_W_m2_sr")))


if __name__ == "__main__":
    main(*(Path(value) for value in sys.argv[1:]))
