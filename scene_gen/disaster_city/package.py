"""Copy the assembled scene and exactly what it references into a self-contained folder.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh package.py [out_dir] [--scene autumn|all] [--zip]   # Kit USD: the Nucleus mirrors are kit crates

default out_dir: data/dist/disaster_city_<scene>. --scene autumn packages disaster_city_autumn.usda (it references
summer, so summer's layers come along; the autumn tree wrappers and ground_tex_autumn/ are extra). --scene all
packages every root in one folder sharing its layers -- disaster_city.usda (the video's survivors),
disaster_city_empty.usda (nobody), disaster_city_v<k>.usda (generated survivors), disaster_city_autumn.usda --
with survivors/ (candidates, each variant's spec and ground truth) and a DisasterWorld manifest.json
(default out_dir data/dist/disaster_city). --zip also writes <out_dir>.zip with the folder at its root.

Walks the used layers of the root stages and the asset paths in them, so only
referenced layers, textures and MDLs are copied (not the reconstruction
working files). Layout is preserved relative to data/recon/, so the relative asset
paths keep working; the folder can be uploaded to Nucleus as-is.
"""
import json, shutil, sys
from pathlib import Path
try:                                        # Kit's USD reads the kit-written Nucleus crates; pip usd-core cannot
    from isaacsim import SimulationApp
    _app = SimulationApp({"headless": True})
except ImportError:
    _app = None

from _paths import DATA, R
scene = sys.argv[sys.argv.index("--scene") + 1] if "--scene" in sys.argv else "summer"
pos = [a for i, a in enumerate(sys.argv[1:], 1) if not a.startswith("--") and sys.argv[i - 1] != "--scene"]
out = Path(pos[0]) if pos else DATA / ("dist/disaster_city" if scene == "all" else f"dist/disaster_city_{scene}")
ROOTS = {"summer": ["disaster_city.usda"], "autumn": ["disaster_city_autumn.usda"],
         "all": ["disaster_city.usda", "disaster_city_empty.usda", "disaster_city_autumn.usda"]
                + sorted(p.name for p in R.glob("disaster_city_v*.usda"))}[scene]
# the used layers of the composed stages, plus every asset path authored in them (textures, MDLs) --
# UsdUtils.ComputeAllDependencies aborts on some kit-written crates, this does not
from pxr import Usd, Sdf
stages = [Usd.Stage.Open(str(R / root)) for root in ROOTS]              # held: their layers expire with them
layers = list({l.identifier: l for s in stages for l in s.GetUsedLayers() if l.realPath}.values())
assets, unresolved = set(), []
for layer in layers:
    def visit(path, layer=layer):
        spec = layer.GetObjectAtPath(path)
        if not isinstance(spec, Sdf.AttributeSpec): return
        try: v = spec.default
        except Exception: return
        for x in (v if type(v).__name__.endswith("Array") or isinstance(v, (list, tuple)) else [v]):
            if isinstance(x, Sdf.AssetPath) and x.path and "://" not in x.path:
                a_ = Path(layer.ComputeAbsolutePath(x.path.split("[")[0]))
                if "<UDIM>" in a_.name:                  # a UDIM set: every tile on disk
                    tiles = list(a_.parent.glob(a_.name.replace("<UDIM>", "[0-9]" * 4))); assets.update(tiles)
                    if not tiles: unresolved.append(str(a_))
                else: (assets.add(a_) if a_.is_file() else unresolved.append(str(a_)))
    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
files = {Path(l.realPath) for l in layers} | assets
files |= {R / f for f in ("georeference.json", "geofence.geojson") if (R / f).exists()}   # georef.py --write
# MDLs pull their textures by relative path, which the USD dependency walk cannot see
for m in [f for f in files if f.suffix == ".mdl"]:
    tex = m.parent / "textures"
    if tex.is_dir(): files |= {p for p in tex.rglob("*") if p.is_file()}
# third-party library trees ship whole: the Nucleus mirrors are kit-written crates this (pip) USD
# cannot read, so the dependency walk above stops at them. They are small (assets/lib + mirrors).
for sub in ("assets/lib", "assets/nucleus", "assets/objaverse"):
    if (R / sub).exists(): files |= {q for q in (R / sub).rglob("*") if q.is_file()}
if scene == "all":   # the survivor variants' specs, ground truth and candidates; not the scratch meshes
    files |= {q for q in (R / "survivors").rglob("*") if q.is_file() and q.suffix in (".json", ".yaml", ".png")}
if out.exists(): shutil.rmtree(out)
total = 0
for f in sorted(files):
    try: rel = f.resolve().relative_to(R.resolve())   # data/ is usually a symlink
    except ValueError: print(f"  outside data/recon/, skipped: {f}"); continue
    dst = out / rel; dst.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(f, dst); total += f.stat().st_size
print(f"{len(files)} files, {total / 1e6:.0f} MB -> {out}")
if scene == "all":
    import numpy as np
    ras = np.load(R / "site_rasters.npz")
    def ground(x, y):
        return float(ras["dtm"][int((float(ras["y1"]) - y) / float(ras["res"])), int((x - float(ras["x0"])) / float(ras["res"]))])
    # DisasterWorld numbers variants from 1: generated v1..v5 keep their numbers, the video's is 6
    variants = {p.stem.split("_v")[1]: (p.name, p.stem.split("_v")[1]) for p in sorted(R.glob("disaster_city_v*.usda"))}
    variants[str(len(variants) + 1)] = ("disaster_city.usda", "0")
    (out / "manifest.json").write_text(json.dumps({
        "schema_version": 1, "units": "meters", "world_frame": "world",
        "source": "Disaster City (TEEX) twin, AirStack scene_gen/disaster_city. Variants 1-5: sample_survivors.py "
                  "seeds 1-5 over survivors/placement_candidates.json, each with the video's class mix; variant "
                  f"{len(variants)}: the survivors seen in the drone video (survivors/v0, specs/survivors.yaml)",
        "whole_map": [[-60, -480], [120, -480], [120, -320], [-60, -320]],
        "affected_area": [[-30, -460], [90, -460], [90, -360], [-30, -360]],     # the operator's search area
        "spawn_points": [[x, -335.0, round(ground(x, -335.0) + 0.4, 2)] for x in (-1.0, 7.0, 15.0, 23.0)],
        "variants": {k: {"stage_file": root, "ground_truth_file": f"survivors/v{gt}/ground_truth.json"}
                     for k, (root, gt) in variants.items()},
        "geometry_file": "disaster_city_empty.usda",
        "placement_candidates_file": "survivors/placement_candidates.json"}, indent=2))
if "--zip" in sys.argv:
    print("zip ->", shutil.make_archive(str(out), "zip", out.parent, out.name))
if unresolved: print(f"  {len(unresolved)} unresolved (listed in the assets' own layers, missing on disk):", sorted(set(unresolved))[:5])
if _app: _app.close()
