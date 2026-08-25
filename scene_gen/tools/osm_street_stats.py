#!/usr/bin/env python
"""
osm_street_stats.py — measure real cities to set the generator's constants.

Offline, one-off, and deliberately NOT importable by the generation path: it
runs on the host, prints a table, and the numbers get copied into a scene config
by hand. Generated cities have no runtime dependency on OSM.

    uv run --no-project --with osmnx --with shapely --with scipy \\
        scene_gen/tools/osm_street_stats.py --place manhattan --dist 500

WHY MEASURE RATHER THAN IMPORT
------------------------------
OSM is a database of real surveyed places, not a layout format or a rules
engine, so nothing in it can be "run" to produce a city. Importing its geometry
directly is the other option, and it is what Toptas's OSM-City-Engine does; the
follow-up PoliMi thesis exists because those reconstructions "lack the realism
and complexity needed to be considered close to a real-life city" and had to be
extended procedurally. We start from the procedural side and use OSM only to
calibrate it, which also sidesteps the fact that this generator is built on
axis-aligned rectangles while real streets curve.

WHAT IT REPORTS, AND WHICH KNOB EACH FIGURE SETS
------------------------------------------------
    block short / long side     layout.anisotropic.block_short_m / block_long_m
    street segment length       cross-check on the above
    intersection density        whether the subdivision splits too eagerly
    signalised share            traffic_lights.intersection_chance
    highway class mix           roads.main_road_chance
    lanes by class              roads.main_road_lanes / secondary_road_lanes
    building:levels             districts.rings height bands
    landuse area shares         districts.rings extent + mix
    furniture frontage spacing  city_detail.categories.*.spacing_m

ENDPOINT PITFALL, since it cost a day. `--endpoint` takes the *interpreter*
URL for readability but osmnx wants the base and appends the path itself.
Handing it the full URL sends the status probe to ".../interpreter/status",
which answers with an error page; osmnx cannot parse that and falls back to
sleeping 60 s before every request. That looked exactly like being rate-limited
and is why furniture was never measured. `main` normalises the URL and, when
the server advertises "Rate limit: 0", switches osmnx's pause off entirely —
its status poll recurses while the server is busy, and a public mirror always
is. See the comments in `main`.

LICENSING
---------
OSM data is ODbL. Share-alike attaches to redistributing a *Derivative
Database*; what leaves this script is a handful of aggregate statistics, which
are facts rather than a substantial extract. Copy medians into a config; do not
commit raw per-object measurements. Credit "(c) OpenStreetMap contributors" and
cite OSMnx — Boeing, G. (2025), *Geographical Analysis* 57(4), 567-577.
"""

import argparse
import math
import os
import statistics
import sys
import time

PLACES = {
    "manhattan": (40.7549, -73.9840),
    "chicago":   (41.8827, -87.6284),
    "sf":        (37.7936, -122.3994),
    "barcelona": (41.3915, 2.1650),
    "portland":  (45.5202, -122.6742),
}

# Categories city_detail places, and the OSM tag that records them.
# ORDER IS LOAD-BEARING: queried top-down, and the heavy ones can stall for
# minutes behind a mirror's 504-and-retry cycle. natural=tree is by far the
# worst (a NYC-scale tree census inside the radius) and streetlight is next,
# so both sit at the bottom — a run cut short still returns the categories
# that were actually in question.
FURNITURE_TAGS = {
    "bench":         {"amenity": "bench"},
    "trash_can":     {"amenity": "waste_basket"},
    "fire_hydrant":  {"emergency": "fire_hydrant"},
    "bike_rack":     {"amenity": "bicycle_parking"},
    "mailbox":       {"amenity": "post_box"},
    "streetlight":   {"highway": "street_lamp"},
    "street_tree":   {"natural": "tree"},
}


# Drivable highway classes, matching what osmnx calls network_type="drive".
# Used by the direct path to measure street centreline length.
_DRIVE = ("motorway|trunk|primary|secondary|tertiary|unclassified|residential|"
          "living_street|motorway_link|trunk_link|primary_link|secondary_link|"
          "tertiary_link")


def _bbox(center, dist):
    """(south, west, north, east) for a *dist*-metre half-width box."""
    lat, lon = center
    dlat = dist / 110_540.0
    dlon = dist / (111_320.0 * math.cos(math.radians(lat)))
    return (lat - dlat, lon - dlon, lat + dlat, lon + dlon)


def _local_xy(center):
    """lat/lon -> local metres. Equirectangular about *center*, which is good
    to well under a metre over the ~1.4 km box these queries cover."""
    lat0, lon0 = center
    kx = 111_320.0 * math.cos(math.radians(lat0))
    return lambda lat, lon: ((lon - lon0) * kx, (lat - lat0) * 110_540.0)


def _overpass(query, endpoint, timeout):
    """POST an Overpass QL query, return parsed JSON."""
    import json
    import urllib.error
    import urllib.parse
    import urllib.request

    base = endpoint.rstrip("/")
    if not base.endswith("/interpreter"):
        base += "/interpreter"
    data = urllib.parse.urlencode({"data": query}).encode()
    req = urllib.request.Request(
        base, data=data,
        headers={"User-Agent": "AirStack scene_gen osm_street_stats "
                               "(procedural-city calibration; aggregates only)"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8", "replace"))


def direct_measure(name, center, dist, endpoint, timeout, retries=3):
    """Furniture stats in TWO Overpass requests instead of osmnx's eight.

    WHY THIS EXISTS. osmnx's `features_from_point` asks for node+way+relation
    per tag, each with `(._;>;)` recursion inside a polygon filter, and it
    issues one such request per category. That is eight heavy queries per city;
    against a loaded mirror each one 504s and is retried a minute later, so a
    five-city run does not finish. The same facts come from one union query
    with `out center tags`, which the same mirror answers in seconds. Fewer,
    lighter queries is also simply the polite way to use a shared public API.

    Still aggregates only: coordinates are used to compute a nearest-neighbour
    distribution in memory and are never written out.
    """
    south, west, north, east = _bbox(center, dist)
    box = f"{south:.6f},{west:.6f},{north:.6f},{east:.6f}"
    km2 = (2 * dist / 1000.0) ** 2          # a box here, not osmnx's disc

    print(f"\n{'=' * 76}\n{name}   {dist} m half-width box   {km2:.2f} km2"
          f"\n{'=' * 76}", flush=True)

    def _fetch(q, what):
        for attempt in range(retries):
            try:
                return _overpass(q, endpoint, timeout)
            except Exception as exc:
                wait = 10 * (attempt + 1)
                print(f"  [{what} attempt {attempt+1}/{retries} "
                      f"{type(exc).__name__}; retrying in {wait}s]", flush=True)
                time.sleep(wait)
        return None

    # ---- street centreline --------------------------------------------------
    # Ways, not a directed graph, so each street is counted once and there is
    # nothing to halve. Segments are kept only when their midpoint is inside
    # the box, which trims the overhang of ways that merely clip the corner.
    street_m = float("nan")
    js = _fetch(f'[out:json][timeout:{timeout}];'
                f'way["highway"~"^({_DRIVE})$"]({box});out geom;', "streets")
    if js is not None:
        to_xy = _local_xy(center)
        total = 0.0
        for el in js.get("elements", ()):
            g = el.get("geometry") or ()
            for a, b in zip(g, g[1:]):
                mlat = (a["lat"] + b["lat"]) / 2.0
                mlon = (a["lon"] + b["lon"]) / 2.0
                if not (south <= mlat <= north and west <= mlon <= east):
                    continue
                ax, ay = to_xy(a["lat"], a["lon"])
                bx, by = to_xy(b["lat"], b["lon"])
                total += math.hypot(bx - ax, by - ay)
        street_m = total
        print(f"  street centreline         {street_m/1000.0:7.2f} km  "
              f"({street_m/km2/1000.0:.1f} km/km2)", flush=True)

    # ---- furniture, all categories in one union -----------------------------
    parts = []
    for tags in FURNITURE_TAGS.values():
        (k, v), = tags.items()
        parts.append(f'node["{k}"="{v}"]({box});')
        parts.append(f'way["{k}"="{v}"]({box});')
    js = _fetch(f'[out:json][timeout:{timeout}];({"".join(parts)});'
                f'out center tags;', "furniture")
    if js is None:
        print("  furniture query FAILED after retries", flush=True)
        return

    to_xy = _local_xy(center)
    found = {c: [] for c in FURNITURE_TAGS}
    for el in js.get("elements", ()):
        tags = el.get("tags") or {}
        lat = el.get("lat", (el.get("center") or {}).get("lat"))
        lon = el.get("lon", (el.get("center") or {}).get("lon"))
        if lat is None or lon is None:
            continue
        for cat, want in FURNITURE_TAGS.items():
            (k, v), = want.items()
            if tags.get(k) == v:
                found[cat].append(to_xy(lat, lon))
                break

    print(f"  {'-'*70}")
    print(f"  {'category':<16}{'n':>6}{'per km2':>9}{'per km st':>10}"
          f"{'frontage':>10}   nearest-neighbour")
    for cat, pts in found.items():
        n = len(pts)
        if not n:
            print(f"  {cat:<16}{0:>6}   not mapped here", flush=True)
            continue
        front = 2.0 * street_m / n
        print(f"  {cat:<16}{n:>6}{n/km2:>9.0f}{n/(street_m/1000.0):>10.1f}"
              f"{front:>9.0f}m   {_q(_nn_xy(pts), 'm')}", flush=True)


def _nn_xy(pts):
    """Nearest-neighbour distance per point, from local metric coordinates."""
    if len(pts) < 2:
        return []
    import numpy as np
    from scipy.spatial import cKDTree
    d, _ = cKDTree(np.asarray(pts)).query(np.asarray(pts), k=2)
    return [float(v) for v in d[:, 1] if math.isfinite(v)]


def _q(vals, unit=""):
    if not vals:
        return "n=0"
    v = sorted(vals)
    return (f"n={len(v):<5} median={statistics.median(v):7.1f}{unit}  "
            f"IQR {v[len(v)//4]:.1f}-{v[(3*len(v))//4]:.1f}")


def _nn(gdf):
    """Nearest-neighbour distance per point (m) — the closest proxy for
    "spacing along a street" that needs no street matching."""
    import numpy as np
    p = gdf.geometry.centroid
    xy = np.column_stack([p.x.values, p.y.values])
    if len(xy) < 2:
        return []
    from scipy.spatial import cKDTree
    d, _ = cKDTree(xy).query(xy, k=2)
    return [float(v) for v in d[:, 1] if math.isfinite(v)]


def measure(name, center, dist, ox, do_furniture=True, do_zoning=True,
            do_blocks=True):
    from shapely.ops import polygonize
    from shapely.geometry import MultiLineString

    print(f"\n{'=' * 76}\n{name}   {dist} m radius\n{'=' * 76}", flush=True)
    G = ox.graph_from_point(center, dist=dist, network_type="drive",
                            simplify=True)
    Gp = ox.project_graph(G)
    nodes, edges = ox.graph_to_gdfs(Gp)
    crs = Gp.graph["crs"]

    # Centreline length of the drive network. Taken from the UNDIRECTED graph:
    # OSMnx returns a MultiDiGraph, so a two-way street is two reciprocal edges
    # and a one-way street is one. Halving the directed sum is therefore only
    # right where nothing is one-way — measured downtown Chicago is 84% one-way
    # by length, where halving undercuts the true centreline by nearly 2x.
    street_m = sum(float(d.get("length") or 0.0)
                   for _, _, d in Gp.to_undirected().edges(data=True))
    # graph_from_point(dist=D) truncates to a BOX of half-width D, not a disc.
    km2 = (2.0 * dist / 1000.0) ** 2
    print(f"  street centreline         {street_m/1000.0:7.2f} km  "
          f"({street_m/km2/1000.0:.1f} km/km2 over {km2:.2f} km2)", flush=True)

    if not do_blocks:
        if do_furniture:
            furniture(name, center, dist, ox, crs, street_m, km2)
        return

    # ---- blocks -------------------------------------------------------------
    print(f"  segment length            "
          f"{_q([float(v) for v in edges['length'] if v], 'm')}")
    polys = list(polygonize(MultiLineString(list(edges.geometry))))
    shorts, longs, ratios, areas = [], [], [], []
    for poly in polys:
        if not (200 < poly.area < 500_000):
            continue
        areas.append(poly.area)
        xs, ys = poly.minimum_rotated_rectangle.exterior.coords.xy
        sides = [math.dist((xs[i], ys[i]), (xs[i + 1], ys[i + 1]))
                 for i in range(4)]
        s, l = min(sides[0], sides[1]), max(sides[0], sides[1])
        if s > 1:
            shorts.append(s)
            longs.append(l)
            ratios.append(l / s)
    print(f"  block area                {_q(areas, 'm2')}")
    print(f"  block SHORT side          {_q(shorts, 'm')}")
    print(f"  block LONG side           {_q(longs, 'm')}")
    print(f"  block long:short          {_q(ratios, 'x')}")

    inter = [n for n, d in Gp.degree() if d > 2]
    print(f"  blocks / km2              {len(areas)/km2:7.0f}")
    print(f"  intersections / km2       {len(inter)/km2:7.0f}")
    if "highway" in nodes.columns and inter:
        sig = int((nodes["highway"] == "traffic_signals").sum())
        print(f"  signalised intersections  {100.0*sig/len(inter):7.0f}%  "
              f"({sig}/{len(inter)})")

    # ---- road hierarchy -----------------------------------------------------
    if "highway" in edges.columns:
        cls = edges["highway"].astype(str).str.strip("[]'\"")
        counts = cls.value_counts()
        total = int(counts.sum())
        main_kinds = ("primary", "secondary", "trunk", "motorway")
        n_main = int(sum(v for k, v in counts.items()
                         if any(m in k for m in main_kinds)))
        print(f"  {'-'*70}")
        print(f"  main-road share           {100.0*n_main/total:7.0f}%  "
              f"-> roads.main_road_chance")
        for k, v in counts.head(6).items():
            lanes = None
            if "lanes" in edges.columns:
                sel = edges[cls == k]["lanes"].dropna()
                vals = []
                for x in sel:
                    try:
                        vals.append(float(x if not isinstance(x, list) else x[0]))
                    except (TypeError, ValueError):
                        pass
                lanes = statistics.median(vals) if vals else None
            print(f"    {k:<22} {100.0*v/total:5.1f}%"
                  + (f"   median {lanes:.0f} lanes" if lanes else ""))

    # ---- zoning: building heights and land use ------------------------------
    if not do_zoning:
        if do_furniture:
            furniture(name, center, dist, ox, crs, street_m, km2)
        return
    print(f"  {'-'*70}")
    try:
        b = ox.features_from_point(center, tags={"building": True}, dist=dist)
        if b is not None and not b.empty:
            lv = []
            for col in ("building:levels", "building:levels:aboveground"):
                if col in b.columns:
                    for x in b[col].dropna():
                        try:
                            lv.append(float(str(x).split(";")[0]))
                        except ValueError:
                            pass
            print(f"  buildings                 n={len(b)}")
            if lv:
                print(f"  building:levels           {_q(lv, ' floors')}")
                v = sorted(lv)
                print(f"    p10/p50/p90             "
                      f"{v[len(v)//10]:.0f} / {statistics.median(v):.0f} / "
                      f"{v[(9*len(v))//10]:.0f}  -> districts height bands")
            if "building" in b.columns:
                bt = b["building"].astype(str).value_counts()
                print("    top building types      "
                      + ", ".join(f"{k}={100.0*v/len(b):.0f}%"
                                  for k, v in bt.head(5).items()))
            bp = b.to_crs(crs)
            fp = [g.area for g in bp.geometry if g.area and g.area < 100_000]
            if fp:
                print(f"  building footprint        {_q(fp, 'm2')}")
    except Exception as exc:
        print(f"  buildings                 query failed: {type(exc).__name__}")

    try:
        lu = ox.features_from_point(center, tags={"landuse": True}, dist=dist)
        if lu is not None and not lu.empty:
            lup = lu.to_crs(crs)
            by = {}
            for kind, geom in zip(lu["landuse"].astype(str), lup.geometry):
                if geom is not None and geom.area:
                    by[kind] = by.get(kind, 0.0) + geom.area
            tot = sum(by.values()) or 1.0
            print("  landuse area share        "
                  + ", ".join(f"{k}={100.0*v/tot:.0f}%"
                              for k, v in sorted(by.items(),
                                                 key=lambda kv: -kv[1])[:6]))
    except Exception as exc:
        print(f"  landuse                   query failed: {type(exc).__name__}")

    if do_furniture:
        furniture(name, center, dist, ox, crs, street_m, km2)


def furniture(name, center, dist, ox, crs, street_m, km2):
    """Per-category count, coverage and the frontage spacing it implies.

    THE NUMBER THAT MATTERS IS `frontage`, NOT THE NEAREST-NEIGHBOUR MEDIAN.
    `city_detail` walks the block frontage ring and emits one prop every
    `spacing_m`, so its count is `frontage_length / spacing`. Nearest-neighbour
    distance does not convert into that: bins cluster in pairs at a crossing and
    a paired bin reports ~2 m however sparse the street is, so the NN median
    measures clumping, not density. Both are printed; only `frontage` is
    directly comparable with a config spacing.

        frontage = 2 x street centreline / n

    because every metre of street has a kerb on each side, and each kerb is the
    frontage of the block behind it.

    COVERAGE IS THE HEALTH CHECK ON ALL OF IT. OSM records street furniture
    only where somebody walked the street and mapped it, so a low count means
    "unmapped" at least as often as it means "not there". A raw spacing from a
    thinly mapped extract is an upper bound on true spacing (too sparse) and
    should not be copied into a config. Treat n/km2 as the weight the figure
    deserves, and prefer a category that several cities agree on.
    """
    print(f"  {'-'*70}")
    print(f"  {'category':<16}{'n':>6}{'per km2':>9}{'per km st':>10}"
          f"{'frontage':>10}   nearest-neighbour")
    for cat, tags in FURNITURE_TAGS.items():
        try:
            g = ox.features_from_point(center, tags=tags, dist=dist)
        except Exception as exc:
            print(f"  {cat:<16}  failed: {type(exc).__name__}", flush=True)
            continue
        if g is None or g.empty:
            print(f"  {cat:<16}{0:>6}   not mapped here", flush=True)
            continue
        n = len(g)
        front = 2.0 * street_m / n if n else float("nan")
        print(f"  {cat:<16}{n:>6}{n/km2:>9.0f}{n/(street_m/1000.0):>10.1f}"
              f"{front:>9.0f}m   {_q(_nn(g.to_crs(crs)), 'm')}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--place", action="append",
                    help=f"one of: {', '.join(PLACES)} (default manhattan)")
    ap.add_argument("--dist", type=int, default=500)
    ap.add_argument("--no-furniture", action="store_true",
                    help="skip the per-category queries (much faster)")
    # The buildings and landuse queries are by far the heaviest of the ten a
    # full run makes — every polygon in the radius, where a furniture query
    # returns nodes — and they are what earns the rate limit that then starves
    # the furniture queries. Sampling several cities for furniture only is what
    # this exists for; the zoning figures are already recorded.
    ap.add_argument("--furniture-only", action="store_true",
                    help="street length + furniture; skip blocks/buildings/"
                         "landuse (3 heavy queries fewer per city)")
    ap.add_argument("--no-zoning", action="store_true",
                    help="skip only the buildings/landuse queries")
    ap.add_argument("--sleep", type=float, default=0.0,
                    help="seconds to wait between places, to stay under the "
                         "Overpass rate limit on a multi-city run")
    ap.add_argument("--direct", action="store_true",
                    help="measure furniture with two hand-written Overpass "
                         "queries instead of osmnx's eight. Much lighter on "
                         "the server and the only path that finishes reliably")
    ap.add_argument("--rate-limit-pause", choices=("auto", "on", "off"),
                    default="auto",
                    help="osmnx's own pre-request pause. auto disables it when "
                         "the server advertises no rate limit, which is the "
                         "only case where its recursive status polling stalls")
    ap.add_argument("--timeout", type=int, default=90,
                    help="per-Overpass-request timeout, seconds")
    # The main instance rate-limits hard and then refuses connections outright
    # for a while. The mirrors carry the same data.
    ap.add_argument("--endpoint", default="https://overpass-api.de/api/interpreter",
                    help="Overpass endpoint; try "
                         "https://overpass.kumi.systems/api/interpreter "
                         "or https://overpass.osm.ch/api/interpreter if "
                         "the default refuses connections")
    args = ap.parse_args()

    try:
        import osmnx as ox
    except ImportError:
        print("osmnx missing. uv run --no-project --with osmnx --with shapely "
              "--with scipy scene_gen/tools/osm_street_stats.py",
              file=sys.stderr)
        return 1

    ox.settings.use_cache = True
    ox.settings.log_console = False
    # Raw Overpass responses ARE a substantial extract of the database, unlike
    # the aggregates this prints. Cache them outside the repo so they cannot be
    # committed by accident.
    ox.settings.cache_folder = os.path.expanduser("~/.cache/osmnx")
    # The earlier version of this script hung: the default Overpass timeout is
    # generous and 36 sequential requests behind a throttle never returned.
    ox.settings.requests_timeout = args.timeout
    ox.settings.overpass_settings = f"[out:json][timeout:{args.timeout}]"

    # osmnx wants the BASE url and appends "/interpreter" and "/status" itself.
    # Passing the full interpreter URL is what made every mirror look like it
    # was rate-limiting: the status probe went to ".../interpreter/status",
    # which is a query endpoint, so it answered with an error page, osmnx
    # failed to parse it and fell back to `default_pause` — 60 s of sleep in
    # front of every single request. Nothing was ever throttled.
    base = args.endpoint.rstrip("/")
    if base.endswith("/interpreter"):
        base = base[:-len("/interpreter")]
    for attr in ("overpass_url", "overpass_endpoint"):   # name varies by version
        if hasattr(ox.settings, attr):
            setattr(ox.settings, attr, base)
    print(f"[osm] endpoint {base}")

    # The second half of the same trap. osmnx reads line 4 of /status and, if
    # it starts with "Currently" (i.e. the server has any query in flight),
    # sleeps 5 s and RECURSES. A busy public mirror is never idle, so that
    # recursion does not terminate in any useful time. The mirrors we use
    # advertise "Rate limit: 0", meaning they do not rate-limit at all, so the
    # whole pause mechanism is both harmful and unnecessary against them.
    # Politeness is handled instead by --sleep between cities.
    limit = None
    if args.rate_limit_pause != "on":
        try:
            import urllib.request
            with urllib.request.urlopen(base + "/status", timeout=20) as r:
                for line in r.read().decode("utf-8", "replace").splitlines():
                    if line.lower().startswith("rate limit:"):
                        limit = int(line.split(":")[1].strip())
                        break
        except Exception as exc:
            print(f"[osm] status probe failed ({type(exc).__name__}); "
                  f"leaving osmnx rate-limit pause as-is")
    if args.rate_limit_pause == "off" or (args.rate_limit_pause == "auto"
                                          and limit == 0):
        ox.settings.overpass_rate_limit = False
        why = ("forced off" if args.rate_limit_pause == "off"
               else f"server advertises rate limit {limit}")
        print(f"[osm] osmnx pause disabled ({why})")
    elif limit is not None:
        print(f"[osm] server advertises rate limit {limit}; osmnx pause kept")
    else:
        # auto with a failed probe: the pause stays on, and against a busy
        # mirror that is the stalling path. --rate-limit-pause off is the fix.
        print("[osm] osmnx pause left ON and the probe failed — if this run "
              "hangs, re-run with --rate-limit-pause off")

    places = [k.lower() for k in (args.place or ["manhattan"])]
    for i, k in enumerate(places):
        if k not in PLACES:
            print(f"unknown place {k!r}; have {', '.join(PLACES)}")
            continue
        if i and args.sleep:
            print(f"[osm] backing off {args.sleep:.0f}s", flush=True)
            time.sleep(args.sleep)
        if args.direct:
            try:
                direct_measure(k, PLACES[k], args.dist, args.endpoint,
                               args.timeout)
            except Exception as exc:
                print(f"{k}: FAILED — {type(exc).__name__}: {exc}", flush=True)
            continue
        try:
            measure(k, PLACES[k], args.dist, ox, not args.no_furniture,
                    do_zoning=not (args.no_zoning or args.furniture_only),
                    do_blocks=not args.furniture_only)
        except Exception as exc:
            print(f"{k}: FAILED — {type(exc).__name__}: {exc}", flush=True)

    print("\n(c) OpenStreetMap contributors — ODbL. Aggregates only.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
