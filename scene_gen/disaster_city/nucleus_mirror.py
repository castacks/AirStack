"""Mirror Nucleus assets (and everything they reference) into data/recon/assets/nucleus/.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh nucleus_mirror.py FactoryDistrict/Meshes/SemiTrailer_mdl.usd ...

Paths are relative to Projects/SEI-COA/ on the lab Nucleus, or to the server root with a leading /
(e.g. /Library/Stages/Muyang/People/Assets/rp_carla_rigged_001_ue4.usd). Runs on the HOST: the
host Isaac Sim (~/isaacsim) has omni.client once
SimulationApp boots, using the cached login (leave OMNI_USER / OMNI_PASS unset).
For each asset, a tolerant walker collects layers, textures and MDLs;
each dependency on the same server is copied to the same relative path under the
mirror, so the relative asset paths inside the layers keep resolving. MDLs pull
textures by relative path the USD walk cannot see: their `textures/` folders are
copied too. Files already present with the same size are skipped.
"""
import os, sys
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.client
from pxr import UsdUtils, Sdf

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import R
SERVER = "omniverse://airlab-nucleus.andrew.cmu.edu:443/"
ROOT = SERVER + "Projects/SEI-COA/"
OUT = R / "assets/nucleus"

def local_of(url):
    u = url.replace("omniverse://airlab-nucleus.andrew.cmu.edu/", SERVER)
    return OUT / u[len(SERVER):] if u.startswith(SERVER) else None

def fetch(url):
    dst = local_of(url)
    if dst is None: return "skip-foreign"
    res, entry = omni.client.stat(url)
    if res != omni.client.Result.OK: return f"missing ({res})"
    if dst.exists() and dst.stat().st_size == entry.size: return "cached"
    res, _v, content = omni.client.read_file(url)
    if res != omni.client.Result.OK: return f"read failed ({res})"
    dst.parent.mkdir(parents=True, exist_ok=True); dst.write_bytes(memoryview(content)); return "copied"

def textures_dir(dir_url):
    res, entries = omni.client.list(dir_url)
    if res != omni.client.Result.OK: return 0
    n = 0
    for e in entries:
        if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN: n += textures_dir(dir_url + e.relative_path + "/")
        elif fetch(dir_url + e.relative_path) == "copied": n += 1
    return n

def walk(url, seen, assets):
    """Layers reachable from url (sublayers, references, payloads) and every asset-valued attribute
    default in them. Tolerant: UsdUtils.ComputeAllDependencies aborts on some kit-written crates."""
    if url in seen: return
    seen.add(url)
    try: layer = Sdf.Layer.FindOrOpen(url)
    except Exception as e: print(f"  cannot open {url}: {str(e)[:80]}"); return
    if layer is None: return
    for dep in layer.GetCompositionAssetDependencies():
        walk(layer.ComputeAbsolutePath(dep), seen, assets)
    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.AttributeSpec):
            try: v = spec.default
            except Exception: return
            vals = v if isinstance(v, (list, tuple)) or type(v).__name__.endswith("Array") else [v]
            for x in vals:
                if isinstance(x, Sdf.AssetPath) and x.path: assets.add(layer.ComputeAbsolutePath(x.path))
    layer.Traverse(Sdf.Path.absoluteRootPath, visit)

total = 0
for rel in sys.argv[1:]:
    url = SERVER + rel[1:] if rel.startswith("/") else ROOT + rel      # a leading / = from the server root
    seen, assets = set(), set(); walk(url, seen, assets); unresolved = []
    deps = sorted(seen) + sorted(a for a in assets if "://" in a or os.path.isabs(a))
    st = {}
    for d in deps: st[d] = fetch(d); total += st[d] == "copied"
    for d in [d for d in deps if d.endswith(".mdl")]:
        total += textures_dir(d.rsplit("/", 1)[0] + "/textures/")
    bad = {k: v for k, v in st.items() if v not in ("copied", "cached")}
    print(f"{rel}: {len(deps)} deps ({sum(v == 'copied' for v in st.values())} copied){'; ' + str(bad) if bad else ''}"
          f"{'; unresolved ' + str(list(unresolved)[:3]) if unresolved else ''}", flush=True)
print(f"mirror: {total} files copied -> {OUT}")
app.close()
