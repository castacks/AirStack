"""Copy the assembled scene and exactly what it references into a self-contained folder.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh package.py [out_dir] [--scene autumn] [--zip]   # Kit USD: the Nucleus mirrors are kit crates

default out_dir: data/dist/disaster_city_<scene>. --scene autumn packages disaster_city_autumn.usda (it references
summer, so summer's layers come along; the autumn tree wrappers and ground_tex_autumn/ are extra). --zip also writes
<out_dir>.zip with the folder at its root.

Walks the used layers of data/recon/disaster_city.usda and the asset paths in them, so only
referenced layers, textures and MDLs are copied (not the reconstruction
working files). Layout is preserved relative to data/recon/, so the relative asset
paths keep working; the folder can be uploaded to Nucleus as-is.
"""
import shutil, sys
from pathlib import Path
try:                                        # Kit's USD reads the kit-written Nucleus crates; pip usd-core cannot
    from isaacsim import SimulationApp
    _app = SimulationApp({"headless": True})
except ImportError:
    _app = None

from _paths import DATA, R
scene = sys.argv[sys.argv.index("--scene") + 1] if "--scene" in sys.argv else "summer"
pos = [a for i, a in enumerate(sys.argv[1:], 1) if not a.startswith("--") and sys.argv[i - 1] != "--scene"]
out = Path(pos[0]) if pos else DATA / f"dist/disaster_city_{scene}"
# the used layers of the composed stage, plus every asset path authored in them (textures, MDLs) --
# UsdUtils.ComputeAllDependencies aborts on some kit-written crates, this does not
from pxr import Usd, Sdf
stage = Usd.Stage.Open(str(R / ("disaster_city_autumn.usda" if scene == "autumn" else "disaster_city.usda")))
layers = [l for l in stage.GetUsedLayers() if l.realPath]; assets, unresolved = set(), []
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
# MDLs pull their textures by relative path, which the USD dependency walk cannot see
for m in [f for f in files if f.suffix == ".mdl"]:
    tex = m.parent / "textures"
    if tex.is_dir(): files |= {p for p in tex.rglob("*") if p.is_file()}
# third-party library trees ship whole: the Nucleus mirrors are kit-written crates this (pip) USD
# cannot read, so the dependency walk above stops at them. They are small (assets/lib + mirrors).
for sub in ("assets/lib", "assets/nucleus", "assets/objaverse"):
    if (R / sub).exists(): files |= {q for q in (R / sub).rglob("*") if q.is_file()}
if out.exists(): shutil.rmtree(out)
total = 0
for f in sorted(files):
    try: rel = f.resolve().relative_to(R.resolve())   # data/ is usually a symlink
    except ValueError: print(f"  outside data/recon/, skipped: {f}"); continue
    dst = out / rel; dst.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(f, dst); total += f.stat().st_size
print(f"{len(files)} files, {total / 1e6:.0f} MB -> {out}")
if "--zip" in sys.argv:
    print("zip ->", shutil.make_archive(str(out), "zip", out.parent, out.name))
if unresolved: print(f"  {len(unresolved)} unresolved (listed in the assets' own layers, missing on disk):", sorted(set(unresolved))[:5])
if _app: _app.close()
