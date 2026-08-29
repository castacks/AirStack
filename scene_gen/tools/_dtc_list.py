import omni.client
for ROOT in ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/downtowncity/",):
    res, ents = omni.client.list(ROOT)
    print(ROOT, res)
    if res != omni.client.Result.OK: continue
    for e in sorted(ents, key=lambda q: q.relative_path):
        kind = "D" if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN else "F"
        print("  %s %-52s %8.1f MB" % (kind, e.relative_path, getattr(e,"size",0)/1e6))
