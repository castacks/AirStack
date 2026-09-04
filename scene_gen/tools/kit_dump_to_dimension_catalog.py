#!/usr/bin/env python3
"""Create the canonical offline building-size catalog from a real Kit dump.

The Kit dump records the effective, Z-up footprint used by its SizeResolver.
Unlike the old scrape, this catalog keys records by full USD URL, placement
scale and source up-axis. Repeated observations must agree.
"""
import argparse
import json
import os


def canonical_asset_id(usd):
    """Use one identity across /root/AirStack and /isaac-sim/AirStack."""
    value = str(usd).replace("\\", "/")
    if "://" in value:
        return value
    marker = "/scene_gen/"
    if marker in value:
        return "airstack://scene_gen/" + value.split(marker, 1)[1]
    if value.startswith("scene_gen/"):
        return "airstack://" + value
    return value


def build_catalog(docs, existing=None):
    found = {}
    sources = []
    if existing:
        sources.extend(existing.get("sources") or [])
        for r in existing.get("records") or ():
            key = (canonical_asset_id(r["usd"]),
                   round(float(r["scale"]), 12),
                   str(r.get("axis_up", "Z")).upper())
            found[key] = tuple(float(r[k]) for k in ("W", "D", "H"))
    for path, doc in docs:
        sources.append({"path": path, "preset": doc.get("preset"),
                        "seed": doc.get("seed")})
        for p in doc.get("placements") or ():
            if p.get("category") != "house":
                continue
            key = (canonical_asset_id(p.get("usd") or ""),
                   round(float(p.get("scale", 1.0)), 12),
                   str(p.get("axis_up", "Z")).upper())
            if not key[0] or any(p.get(k) is None for k in ("W", "D", "H")):
                continue
            w, d, h = (float(p[k]) for k in ("W", "D", "H"))
            # Current dumps explicitly store world-axis W/D. Older Kit dumps
            # (including the first L1 diagnostic) stored resolver-local W/D
            # and have no dimensions_space marker. Canonical records are
            # always resolver-local effective dimensions.
            if doc.get("dimensions_space") == "world_xy" and \
                    45.0 <= (float(p.get("yaw_deg", 0.0)) % 180.0) < 135.0:
                w, d = d, w
            dims = (w, d, h)
            old = found.get(key)
            if old and any(abs(old[i] - dims[i]) > 1e-4 for i in range(3)):
                raise ValueError("inconsistent Kit dimensions for %r: %r vs %r"
                                 % (key, old, dims))
            found[key] = dims
    records = [
        {"usd": k[0], "scale": k[1], "axis_up": k[2],
         "W": v[0], "D": v[1], "H": v[2]}
        for k, v in sorted(found.items())
    ]
    return {"schema": "scene_gen.urban_building_dimensions.v1",
            "sources": sources, "records": records}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("dumps", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--merge-existing", default="")
    args = ap.parse_args()
    docs = [(p, json.load(open(p))) for p in args.dumps]
    existing = None
    if args.merge_existing and os.path.exists(args.merge_existing):
        existing = json.load(open(args.merge_existing))
    out = build_catalog(docs, existing=existing)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as fh:
        json.dump(out, fh, indent=2)
        fh.write("\n")
    print("[dimensions] %d canonical model/scale/up-axis record(s) -> %s"
          % (len(out["records"]), args.out))


if __name__ == "__main__":
    main()
