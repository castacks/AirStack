#!/usr/bin/env python
"""Bake VRAM census — measurement only, touches nothing else.

Opens each `fire_bakes/*.usd` cold. Walks every UsdShade.Shader input whose
value is an Sdf.AssetPath to find unique textures, resolves `omniverse://`
urls via a cache-dir search first then a one-time fetch into
`.cache/vram_census/` (same `omni.client.read_file` pattern as
`nucleus_fetch.py`), sizes them with PIL (a hand-rolled header reader for
.exr, which PIL cannot open), and estimates geometry+BVH cost.
    docker exec isaac-sim bash -c \
        "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \
         /isaac-sim/AirStack/scene_gen/tools/bake_vram_census.py \
         /isaac-sim/.cache/fire_bakes/*.usd"
"""
import glob
import os
import struct
import sys
from pxr import Sdf, Usd, UsdGeom, UsdShade

CACHE_DIRS = ["/isaac-sim/.cache", os.path.expanduser("~/.cache/ov"),
              "/root/.cache/ov", "/isaac-sim/.nvidia-omniverse"]
FETCH_DIR = "/isaac-sim/.cache/vram_census"
IMG_EXT = (".png", ".jpg", ".jpeg", ".tga", ".exr", ".dds", ".hdr")

def build_cache_index():
    idx = {}
    for root in CACHE_DIRS:
        if not os.path.isdir(root):
            continue
        for dp, _, files in os.walk(root):
            for fn in files:
                if fn.lower().endswith(IMG_EXT) and fn not in idx:
                    idx[fn] = os.path.join(dp, fn)
    return idx

def exr_dims(path):
    data = open(path, "rb").read()
    if data[:4] != b"\x76\x2f\x31\x01": return None
    pos, w, h, nchan, pixtype = 8, None, None, None, 1
    while True:
        end = data.index(b"\x00", pos)
        name = data[pos:end]
        if not name: break
        pos = data.index(b"\x00", end + 1) + 1
        size = struct.unpack_from("<i", data, pos)[0]
        body = data[pos + 4:pos + 4 + size]
        pos += 4 + size
        if name == b"dataWindow":
            xmin, ymin, xmax, ymax = struct.unpack_from("<iiii", body); w, h = xmax - xmin + 1, ymax - ymin + 1
        elif name == b"channels":
            nchan, off = 0, 0
            while off < len(body) and body[off]:
                nul = body.index(b"\x00", off)
                pixtype = struct.unpack_from("<i", body, nul + 1)[0]
                off, nchan = nul + 1 + 16, nchan + 1
    return None if w is None else (w, h, nchan * (2 if pixtype == 1 else 4))

def role_of(basename):
    b = basename.lower()
    if b.startswith("sootbake_") or b.startswith("gacsoot_"): return "soot"
    if "basecolor" in b or "diffuse" in b: return "BaseColor"
    if "normal" in b: return "Normal"
    if "orm" in b or "roughness" in b or "metallic" in b: return "ORM"
    return "other"

def image_stats(path):
    if path.lower().endswith(".exr"): return exr_dims(path)
    try:
        from PIL import Image
        im = Image.open(path)
        w, h = im.size
        return w, h, {"RGB": 4, "RGBA": 4, "L": 2}.get(im.mode, 4)
    except Exception as e:
        print("    ! image open failed %s: %r" % (path, e))
        return None

def resolve_texture(url, cache_idx, fetched):
    if not url.startswith("omniverse://"): return url if os.path.exists(url) else None
    base = url.rsplit("/", 1)[-1]
    hit = cache_idx.get(base)
    if hit:
        print("    cache-hit %s -> %s" % (base, hit)); return hit
    dest = os.path.join(FETCH_DIR, base)
    if os.path.exists(dest) or url in fetched: return fetched.get(url, dest if os.path.exists(dest) else None)
    import omni.client as oc
    r, _ver, content = oc.read_file(url)
    if r != oc.Result.OK:
        print("    ! fetch failed %s: %s" % (url, r)); fetched[url] = None; return None
    os.makedirs(FETCH_DIR, exist_ok=True)
    with open(dest, "wb") as fh:
        fh.write(bytes(memoryview(content)))
    print("    fetched %s (%d bytes)" % (base, len(content)))
    fetched[url] = dest
    return dest

def collect_texture_urls(stage):
    urls = set()
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        sh = UsdShade.Shader(prim)
        if not sh: continue
        try:
            inputs = sh.GetInputs()
        except Exception:
            continue
        for inp in inputs:
            try:
                v = inp.Get()
            except Exception:
                continue
            if isinstance(v, Sdf.AssetPath):
                key = v.resolvedPath or v.path
                if key: urls.add(key)
    return urls

def geometry_stats(stage):
    n_mesh = points = tris = subsets = 0
    mats = set()
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not prim.IsActive() or not prim.IsA(UsdGeom.Mesh): continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if not pts: continue
        n_mesh, points = n_mesh + 1, points + len(pts)
        counts = me.GetFaceVertexCountsAttr().Get() or []
        tris += sum(max(c - 2, 0) for c in counts)
        m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if m and m.GetPrim().IsValid(): mats.add(str(m.GetPrim().GetPath()))
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
        subsets += len(subs)
        for s in subs:
            sm = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
            if sm and sm.GetPrim().IsValid(): mats.add(str(sm.GetPrim().GetPath()))
    geo_bytes = (points * 12 + tris * 12) * 2
    return n_mesh, points, tris, len(mats), subsets, geo_bytes

def report_bake(path, cache_idx, fetched, global_tex):
    name = os.path.basename(path)
    print("\n==== %s ====" % name)
    stage = Usd.Stage.Open(path)
    if stage is None:
        print("  OPEN FAILED"); return None
    by_group, rows, soot_rows = {}, [], []
    for url in sorted(collect_texture_urls(stage)):
        base = os.path.basename(url)
        local = resolve_texture(url, cache_idx, fetched)
        if local is None:
            print("    ! unresolved %s" % url); continue
        st = image_stats(local)
        if st is None: continue
        w, h, bpt = st
        mb = (w * h * bpt * 1.33) / (1024.0 * 1024.0)
        grp = role_of(base)
        by_group[grp] = by_group.get(grp, 0.0) + mb
        rows.append((base, w, h, mb, grp))
        if base.lower().startswith("sootbake_"): soot_rows.append((base, w, h))
        g = global_tex.setdefault(url, {"mb": mb, "grp": grp, "base": base,
                                         "dims": "%dx%d" % (w, h), "bakes": set()})
        g["bakes"].add(name)
    print("  unique textures: %d" % len(rows))
    for grp in sorted(by_group):
        print("    %-10s %8.2f MB" % (grp, by_group[grp]))
    print("  total (this bake, not deduped): %.2f MB\n  top 8 textures:" % sum(by_group.values()))
    for base, w, h, mb, grp in sorted(rows, key=lambda r: -r[3])[:8]:
        print("    %-60s %5dx%-5d %7.2f MB  [%s]" % (base, w, h, mb, grp))
    dims = sorted(set((w, h) for _b, w, h in soot_rows))
    print("  sootbake_*.png referenced: %d  dims: %s" %
          (len(soot_rows), ", ".join("%dx%d" % d for d in dims) or "n/a"))
    n_mesh, points, tris, n_mat, subsets, geo_bytes = geometry_stats(stage)
    geo_mb = geo_bytes / (1024.0 * 1024.0)
    print("  meshes=%d points=%d tris=%d materials=%d geomsubsets=%d  geometry VRAM (rough, x2 BVH): %.2f MB"
          % (n_mesh, points, tris, n_mat, subsets, geo_mb))
    kind = ("gac" if name.startswith("gac_") else
            "kit" if name.startswith("kit_") else "other")
    return {"name": name, "kind": kind, "soot_mb": by_group.get("soot", 0.0),
            "total_mb": sum(by_group.values()), "geo_mb": geo_mb,
            "n_soot_tex": len(soot_rows)}

def report_totals(global_tex):
    print("\n==== DEDUPLICATED TEXTURE TOTAL (all bakes) ====")
    print("unique texture urls: %d" % len(global_tex))
    dedup = {}
    for g in global_tex.values():
        dedup[g["grp"]] = dedup.get(g["grp"], 0.0) + g["mb"]
    for grp in sorted(dedup):
        print("  %-10s %8.2f MB" % (grp, dedup[grp]))
    print("  TOTAL DEDUPED: %.2f MB" % sum(dedup.values()))
    print("\nshared across >=2 bakes:")
    for _url, g in sorted(global_tex.items(), key=lambda kv: -kv[1]["mb"]):
        if len(g["bakes"]) >= 2:
            print("  %-60s %-10s %8.2f MB  in %s" %
                  (g["base"], g["dims"], g["mb"], ",".join(sorted(g["bakes"]))))

BC1_VS_RGBA8_MIPS = 8.0
# BC1 (opaque, no-alpha DXT1) is exactly 0.5 bytes/texel vs. RGBA8's 4 --
# and both scale by the same 4/3 full-mip-pyramid factor this whole
# script's `mb = w*h*bpt*1.33` already assumes, so the ratio is exactly 8x
# at any resolution. See `disaster/tex_compress.py` (`SOOT_TEX_COMPRESS`)
# and `tools/soot_dds_probe.py` (measured against the real corpus: exactly
# 8.00x, mean 42.7 dB PSNR on a 400-file sample, 2026-09-01).

def report_projection(records, project_counts=(71, 130)):
    """Per-building averages and a straight-line VRAM projection at
    `project_counts` buildings -- soot and geometry are UNIQUE per building
    (measured: zero content-hash collisions across a 48-building corpus) so
    they scale linearly; BaseColor/Normal/ORM/other are the kit's own
    shared assets and do NOT grow with building count past the point every
    module type has appeared once, so they are reported as a floor, not
    scaled. This is a projection, not a re-measurement -- a city with
    materially different asset variety will move the shared floor."""
    recs = [r for r in records if r]
    if not recs:
        print("\n(no bakes to project from)")
        return
    print("\n==== PER-BUILDING PROJECTION ====")
    for kind in ("gac", "kit", "other"):
        ks = [r for r in recs if r["kind"] == kind]
        if not ks:
            continue
        soot = [r["soot_mb"] for r in ks]
        geo = [r["geo_mb"] for r in ks]
        print("  %-6s n=%-3d  soot avg %7.2f MB (sum %8.2f)   "
              "geo avg %6.2f MB (sum %7.2f)"
              % (kind, len(ks), sum(soot) / len(ks), sum(soot),
                 sum(geo) / len(ks), sum(geo)))
    soot_all = [r["soot_mb"] for r in recs]
    geo_all = [r["geo_mb"] for r in recs]
    n = len(recs)
    soot_avg = sum(soot_all) / n
    geo_avg = sum(geo_all) / n
    print("  %-6s n=%-3d  soot avg %7.2f MB (sum %8.2f)   "
          "geo avg %6.2f MB (sum %7.2f)"
          % ("ALL", n, soot_avg, sum(soot_all), geo_avg, sum(geo_all)))
    print("\n  projection = N * (soot_avg + geo_avg); soot also shown at "
          "the measured BC1 ratio (%.2fx)" % BC1_VS_RGBA8_MIPS)
    for N in project_counts:
        soot_mb = N * soot_avg
        geo_mb = N * geo_avg
        print("    N=%-4d  soot %8.1f MB  geo %7.1f MB  "
              "total(uncompressed) %8.1f MB (%.2f GB)   "
              "soot@BC1 %7.1f MB -> total(BC1 soot) %8.1f MB (%.2f GB)"
              % (N, soot_mb, geo_mb, soot_mb + geo_mb, (soot_mb + geo_mb) / 1024.0,
                 soot_mb / BC1_VS_RGBA8_MIPS,
                 soot_mb / BC1_VS_RGBA8_MIPS + geo_mb,
                 (soot_mb / BC1_VS_RGBA8_MIPS + geo_mb) / 1024.0))

def main():
    args = []
    for a in sys.argv[1:]:
        args.extend(sorted(glob.glob(a)) if "*" in a else [a])
    cache_idx = build_cache_index()
    fetched, global_tex = {}, {}
    records = []
    for path in args:
        records.append(report_bake(path, cache_idx, fetched, global_tex))
    report_totals(global_tex)
    report_projection(records)

if __name__ == "__main__":
    raise SystemExit(main())
