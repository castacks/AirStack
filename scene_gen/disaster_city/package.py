"""Copy the assembled scene and exactly what it references into a self-contained folder.

    ~/.venvs/recon/bin/python package.py [out_dir]      # default data/dist/disaster_city

Uses UsdUtils.ComputeAllDependencies on data/recon/disaster_city.usda, so only
referenced layers, textures and MDLs are copied (not the reconstruction
working files). Layout is preserved relative to data/recon/, so the relative asset
paths keep working; the folder can be uploaded to Nucleus as-is.
"""
import shutil, sys
from pathlib import Path
from pxr import UsdUtils

from _paths import DATA, R
out = Path(sys.argv[1]) if len(sys.argv) > 1 else DATA / "dist/disaster_city"
layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(R / "disaster_city.usda"))
files = {Path(l.realPath) for l in layers if l.realPath} | {Path(a) for a in assets}
# MDLs pull their textures by relative path, which the USD dependency walk cannot see
for m in [f for f in files if f.suffix == ".mdl"]:
    tex = m.parent / "textures"
    if tex.is_dir(): files |= {p for p in tex.rglob("*") if p.is_file()}
if out.exists(): shutil.rmtree(out)
total = 0
for f in sorted(files):
    try: rel = f.resolve().relative_to(R)
    except ValueError: print(f"  outside data/recon/, skipped: {f}"); continue
    dst = out / rel; dst.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(f, dst); total += f.stat().st_size
print(f"{len(files)} files, {total / 1e6:.0f} MB -> {out}")
if unresolved: print(f"  {len(unresolved)} unresolved (listed in the assets' own layers, missing on disk):", sorted(set(unresolved))[:5])
