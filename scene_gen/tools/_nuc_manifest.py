"""Summarise the Nucleus archetype library: files + manifest records."""
import os, json, collections
import omni.client as oc
root = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype/"
r, ents = oc.list(root)
files = [(e.relative_path, getattr(e, "size", 0)) for e in ents if not (e.flags & oc.ItemFlags.CAN_HAVE_CHILDREN)]
usds = [f for f in files if f[0].endswith(".usd")]
print("files:", len(files), "usd:", len(usds), "total MB: %.0f" % (sum(s for _, s in usds) / 1e6))
big = sorted(usds, key=lambda t: -t[1])[:8]
print("largest:", [(n, "%.1f MB" % (s / 1e6)) for n, s in big])
styles = collections.Counter(n.split("_DG")[0].split("_SETTLE")[0].split("_TILT")[0].split("_OV")[0] for n, _ in usds)
print("styles:", dict(styles))
r, _, content = oc.read_file(root + "archetypes.json")
if r == oc.Result.OK:
    m = json.loads(bytes(memoryview(content)).decode())
    print("manifest type:", type(m).__name__, "len", len(m))
    recs = m if isinstance(m, list) else m.get("records", m.get("archetypes", []))
    if isinstance(recs, dict):
        k0 = next(iter(recs)); print("first key:", k0); print("first rec:", json.dumps(recs[k0])[:600]); 
        keys = collections.Counter(); 
        for v in recs.values():
            if isinstance(v, dict): keys.update(v.keys())
        print("keys:", dict(keys))
    else:
        print("first rec:", json.dumps(recs[0])[:600])
        keys = collections.Counter()
        for v in recs: keys.update(v.keys())
        print("keys:", dict(keys))
        print("with fall_sides:", sum(1 for v in recs if "fall_sides" in v), "with baked_at/time:", sum(1 for v in recs if "baked" in json.dumps(v)[:200]))
else:
    print("no manifest:", r)
