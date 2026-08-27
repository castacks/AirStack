import sys, omni.client as c
c.initialize()
ROOT = "omniverse://airlab-nucleus.andrew.cmu.edu:443"
def walk(url, depth, maxd, prefix=""):
    r, entries = c.list(url)
    if str(r) != "Result.OK":
        print(f"{prefix}[ERR {r}] {url}")
        return
    ents = sorted(entries, key=lambda e: e.relative_path)
    for e in ents:
        isdir = bool(e.flags & c.ItemFlags.CAN_HAVE_CHILDREN)
        print(f"{prefix}{e.relative_path}{'/' if isdir else ''}\t{'' if isdir else e.size}")
        if isdir and depth < maxd:
            walk(url.rstrip('/') + '/' + e.relative_path, depth+1, maxd, prefix + "  ")
for arg in sys.argv[1:]:
    path, _, d = arg.partition('|')
    maxd = int(d) if d else 2
    url = path if path.startswith("omniverse://") else ROOT + path
    print(f"\n===== {url}  (depth {maxd}) =====")
    walk(url, 1, maxd)
c.shutdown()
