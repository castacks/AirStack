#!/usr/bin/env python3
"""Ground truth for a FROZEN dataset cell, from the files that shipped with it.

`scene_annotations.py` measures a GENERATED scene off the live stage, because a
scene that was just built is the only place its contents are written down. A
frozen cell is the opposite case: `freeze_dataset_launch_script.py` already
measured everything and wrote it beside the `.usd` as `GT_people.json` and
`GT_hints.json`. Re-measuring it off the stage would be slower, would need Kit,
and — worse — could DISAGREE with the labels the dataset ships, which are the
answer key a run is scored against. So this module does no measuring at all: it
RESHAPES the cell's own files into the three names the stack looks for.

    <scene>.json            people only  — what the GCS draws and the scorer reads
    <scene>_obstacles.json  houses / trees / cars — the 3D lawnmower's clearance
    <scene>_region.json     the disaster-affected polygon — the search area

Everything here is stdlib (`json`, `math`): it runs in the Isaac launcher, in
the offline mission planner (`scene_gen/tools/frozen_cell_plan.py`) and in the
unit tests, none of which should need pxr, numpy or a GPU.

THE AFFECTED AREA IS MEASURED FROM THE DAMAGE, NOT FROM THE DISASTER MODEL.
`scene_gen/disaster/region.py` derives a wildfire's area from the fire model —
the ellipse at `elapsed` — and that is right for a scene being built, where the
model is what authored the damage. A frozen cell is a finished artefact, and
two of the six suburban cells (the tornado levels) carry no `build_stats.json`
to re-derive a model from at all. What every cell DOES carry is the position of
every damaged object, so the affected area here is the convex hull of the
damaged objects' own footprints: it is what the file actually contains rather
than what the generator intended, and it is derived the same way for every
disaster. Measured on the six frozen suburban cells it comes out at 25/52/74%
of the plate for fire levels 1-3 and 12/16/27% for tornado levels 1-3 — the
intensity ladder, read straight off the geometry.

THE HULL IS CONVEX ON PURPOSE. The consumer is `sector.partition`, which cuts
the polygon into per-robot rectangles; a concave blob would hand a drone a
bounding rectangle full of ground its neighbour also owns. A fire scar is an
ellipse and a tornado track is a corridor, so a hull is a close fit to both.
"""

import json
import math
import os

# GT_hints class -> the lowercase class `search_baselines/clearance.py`
# knows (`OBSTACLE_CLASSES`). Anything not listed is NOT an obstacle to a drone
# at cruise height and is dropped: `Pool` and `Parking Lot` are flat ground,
# and people are what the search is FOR.
OBSTACLE_CLASS_MAP = {
    "Building": "house",
    "Damaged building": "house",
    "Tree": "tree",
    "Burnt Tree": "tree",
    "Fallen Tree": "tree",
    "Car": "car",
    "Van": "car",
    "Truck": "truck",
    "Toppled": "prop",
    "Debris": "prop",
}

# The classes that are EVIDENCE OF THE DISASTER. `Toppled` is an attitude that
# REPLACES the base class (a rolled car, a felled streetlight — gt_hints.py), so
# it belongs here rather than with the intact vehicles; `Debris` is one record
# per field, and a field is only authored where something came apart.
DAMAGE_CLASSES = ("Damaged building", "Burnt Tree", "Fallen Tree", "Debris",
                  "Toppled")

# A person, in metres — the same nominal box `scene_annotations.PERSON_SIZE_M`
# uses, and for the same reason: the rigs are posed, so their own AABB varies
# with the pose while the GT question ("is there a person here") does not.
PERSON_SIZE_M = (0.7, 0.7, 1.8)


# ---------------------------------------------------------------------------
# reading a cell
# ---------------------------------------------------------------------------

CONTAINER_ROOT = "/isaac-sim/final_disaster_dataset"

# `omniverse://`, `file://`, `https://` — anything with a scheme is a URL and
# must NOT be probed with `os.path`. On an OSMO pod the dataset is not on a
# local mount at all (`final_disaster_dataset/` is outside the repo and is not
# cloned), so the cells are addressed on Nucleus, where `os.path.isfile` is
# always False and would reject a perfectly good scene.
_URL_MARK = "://"


def is_url(path):
    return _URL_MARK in (path or "")


def join(base, *parts):
    """`os.path.join` for a local path, `/`-join for a URL."""
    if is_url(base):
        out = base.rstrip("/")
        for p in parts:
            out += "/" + str(p).strip("/")
        return out
    return os.path.join(base, *parts)


def _split(path):
    """Path components, for a local path or a URL, without the scheme."""
    if is_url(path):
        return [p for p in path.split(_URL_MARK, 1)[1].split("/") if p]
    return [p for p in os.path.abspath(path).split(os.sep) if p]


def usd_name_for_cell(cell_path):
    """`<disaster>_<locale>_lvl<n>_<k>.usd` from a `.../<Disaster>/<Locale>/
    level_<n>/<k>` path or URL, or None if it is not shaped like a cell.

    The contract is capitalised in the PATH and lowercase in the FILENAME
    (freeze-disaster-dataset), so the name is derivable and a caller can name
    the CELL rather than repeat the rule.
    """
    parts = _split(cell_path)
    if len(parts) < 4 or not parts[-2].startswith("level_"):
        return None
    return "{0}_{1}_lvl{2}_{3}.usd".format(
        parts[-4].lower(), parts[-3].lower(),
        parts[-2].split("_", 1)[1], parts[-1])


def resolve_cell(spec, root=CONTAINER_ROOT):
    """`FROZEN_SCENE` -> the `.usd` to reference. Raises `ValueError`.

    Four accepted spellings, because a mission author should not have to
    remember which one the launcher wants:

        /abs/path/to/fire_suburban_lvl1_1.usd
        Fire/Suburban/level_1/1/fire_suburban_lvl1_1.usd   (under `root`)
        Fire/Suburban/level_1/1                            (the cell)
        omniverse://host/Projects/.../Fire/Suburban/level_1/1   (either, on Nucleus)

    **A LOCAL path that does not exist RAISES** rather than being handed to
    `AddReference`, which composes an empty prim, reports success and gives
    you a black viewport with no error 20 minutes into a pod.

    **A URL is NOT probed.** `os.path` cannot answer for `omniverse://`, and
    the alternative — pulling `omni.client` into a module that is deliberately
    stdlib-only so it can run in the tests and in the offline mission planner —
    would cost more than it buys: a bad URL fails at `Sdf.Layer.FindOrOpen`
    with the server's own message, which is at least as good a diagnosis. When
    the URL names a CELL rather than a file, the filename is derived from the
    contract, so a mission's `FROZEN_SCENE` is the same string either way and
    only `FROZEN_DATASET_ROOT` changes between a local run and a pod.
    """
    spec = (spec or "").strip()
    if not spec:
        raise ValueError("FROZEN_SCENE is empty")
    if is_url(spec) or is_url(root):
        url = spec if is_url(spec) else join(root, spec)
        if url.lower().endswith(".usd") or url.lower().endswith(".usdc"):
            return url
        name = usd_name_for_cell(url)
        if not name:
            raise ValueError(
                "FROZEN_SCENE={0!r} resolves to {1}, which is neither a .usd "
                "nor a <Disaster>/<Locale>/level_<n>/<k> cell".format(spec, url))
        return join(url, name)
    cands = ([spec] if os.path.isabs(spec) else []) + [os.path.join(root, spec)]
    for c in cands:
        if os.path.isfile(c):
            return os.path.abspath(c)
        if os.path.isdir(c):
            name = usd_name_for_cell(c)
            if name and os.path.isfile(os.path.join(c, name)):
                return os.path.abspath(os.path.join(c, name))
            usds = sorted(f for f in os.listdir(c) if f.endswith(".usd"))
            if len(usds) == 1:
                return os.path.abspath(os.path.join(c, usds[0]))
            raise ValueError(
                "FROZEN_SCENE={0!r}: {1} holds {2} .usd files; name one"
                .format(spec, c, len(usds)))
    raise ValueError(
        "FROZEN_SCENE={0!r}: no such file or cell (looked in {1}). Is the "
        "dataset mounted at {2}, or should FROZEN_DATASET_ROOT point at "
        "Nucleus?".format(spec, ", ".join(cands), root))


def cell_dir(scene_usd):
    """The dataset cell directory a frozen `.usd` lives in."""
    if is_url(scene_usd):
        return scene_usd.rsplit("/", 1)[0]
    return os.path.dirname(os.path.abspath(scene_usd))


def load_cell(scene_usd, read_text=None):
    """`(people_doc, hints_doc)` for the cell holding `scene_usd`.

    `read_text(path) -> str | None` is the escape hatch for a cell that lives
    on Nucleus: this module stays stdlib-only, so the CALLER (the Isaac
    launcher, which has `omni.client`) supplies the reader when the cell is a
    URL. Everything else — the tests, the offline mission planner — gets the
    plain-`open` default.

    A missing file is returned as `None` rather than raised on: a cell whose
    export died before the hints were written still has usable people, and the
    launcher should say which of the three files it could not write instead of
    refusing to fly.
    """
    d = cell_dir(scene_usd)
    read = read_text or _read_local
    return (_load_json(join(d, "GT_people.json"), read),
            _load_json(join(d, "GT_hints.json"), read))


def _read_local(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def _load_json(path, read):
    try:
        text = read(path)
    except Exception:
        return None
    if not text:
        return None
    try:
        return json.loads(text)
    except ValueError:
        return None


def _read_json(path):
    return _load_json(path, _read_local)


def people_records(people_doc):
    """The survivor records out of a `GT_people.json` (== `humans.json`)."""
    if isinstance(people_doc, dict):
        return list(people_doc.get("people") or ())
    return list(people_doc or ())


def hint_records(hints_doc):
    """The object records out of a `GT_hints.json`."""
    if isinstance(hints_doc, dict):
        return list(hints_doc.get("hints") or ())
    return list(hints_doc or ())


def region_m(hints_doc, default=(-500.0, -500.0, 500.0, 500.0)):
    """The plate rectangle `(x0, y0, x1, y1)` from the hints' own meta."""
    meta = (hints_doc or {}).get("meta") if isinstance(hints_doc, dict) else None
    reg = (meta or {}).get("region_m")
    if reg and len(reg) == 4:
        return tuple(float(v) for v in reg)
    return tuple(float(v) for v in default)


# ---------------------------------------------------------------------------
# the three annotation files
# ---------------------------------------------------------------------------

def people_boxes(people_doc, size_m=PERSON_SIZE_M):
    """One `person` box per survivor, in the schema `annotation_viz_node` reads.

    `z` in a record is the ground the person stands on, so the box centre is
    half a body above it — the same convention `scene_annotations` uses, so a
    frozen cell and a generated one produce identical GT for the same people.
    """
    boxes = []
    for r in people_records(people_doc):
        try:
            x, y, z = float(r["x"]), float(r["y"]), float(r.get("z", 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        boxes.append({
            "class": "person",
            "bbox_world": {
                "center_xyz_m": [round(x, 3), round(y, 3),
                                 round(z + size_m[2] / 2.0, 3)],
                "size_xyz_m": [size_m[0], size_m[1], size_m[2]],
            },
        })
    return boxes


def obstacle_boxes(hints_doc, class_map=None):
    """`house` / `tree` / `car` / `truck` / `prop` boxes for the lawnmower.

    Straight off each hint's `bbox_min`/`bbox_max`, which `gt_hints.py`
    measured on the composed stage with a both-purposes `BBoxCache`. They are
    world AABBs, which is exactly what `clearance.load_boxes` wants — the
    `yaw_deg` a consumer would need for an oriented box is carried in the hint
    and deliberately not used here.
    """
    cmap = class_map or OBSTACLE_CLASS_MAP
    boxes = []
    for r in hint_records(hints_doc):
        cls = cmap.get(r.get("class"))
        if not cls:
            continue
        lo, hi = r.get("bbox_min"), r.get("bbox_max")
        if not (lo and hi and len(lo) >= 3 and len(hi) >= 3):
            continue
        try:
            lo = [float(v) for v in lo[:3]]
            hi = [float(v) for v in hi[:3]]
        except (TypeError, ValueError):
            continue
        size = [hi[i] - lo[i] for i in range(3)]
        if max(size) <= 0.0:
            continue
        boxes.append({
            "class": cls,
            "bbox_world": {
                "center_xyz_m": [round((lo[i] + hi[i]) / 2.0, 3)
                                 for i in range(3)],
                "size_xyz_m": [round(abs(v), 3) for v in size],
            },
        })
    return boxes


def damage_points(hints_doc, classes=DAMAGE_CLASSES):
    """Every damaged object's footprint CORNERS, `[(x, y), ...]`.

    Corners rather than centres: a `burned_out` house is 26 m across and a
    fallen tree lies 20 m along the ground, so a hull of centres is up to a
    building-width inside the damage it is meant to bound. The hull of the
    corners bounds the objects themselves.

    A record with no `bbox_min` (a handful of `Burnt Tree` rows whose archetype
    measured empty) falls back to its `centre`, and one with neither is skipped
    — silently, because the hull of 1,700 objects does not change for one.
    """
    want = set(classes)
    pts = []
    for r in hint_records(hints_doc):
        if r.get("class") not in want:
            continue
        lo, hi = r.get("bbox_min"), r.get("bbox_max")
        try:
            if lo and hi and len(lo) >= 2 and len(hi) >= 2:
                x0, y0 = float(lo[0]), float(lo[1])
                x1, y1 = float(hi[0]), float(hi[1])
                pts += [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                continue
            c = r.get("centre")
            if c and len(c) >= 2:
                pts.append((float(c[0]), float(c[1])))
        except (TypeError, ValueError):
            continue
    return pts


def convex_hull(points):
    """Counter-clockwise convex hull of `[(x, y), ...]` (monotone chain).

    Deterministic — sorted input, no tolerance, no randomness — because the
    polygon it produces is cut into per-robot sectors by `sector.partition`,
    and every robot has to compute the same cut from the same file.
    """
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def polygon_area(poly):
    """Absolute shoelace area, m^2."""
    if len(poly) < 3:
        return 0.0
    s = 0.0
    for i, (x, y) in enumerate(poly):
        x2, y2 = poly[(i + 1) % len(poly)]
        s += x * y2 - x2 * y
    return abs(s) / 2.0


def point_in_polygon(x, y, poly):
    """Even-odd ray cast. Matches `sector.points_in_polygon`'s convention
    closely enough for the reporting this module does (it is not what confines
    a drone — that is the planner's own copy)."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def clip_to_rect(poly, rect):
    """Sutherland-Hodgman clip of `poly` to `(x0, y0, x1, y1)`.

    The hull of the damage can run a metre or two past the plate edge (a house
    on the boundary has half its box outside), and a search polygon that leaves
    the ground is a drone told to fly where there is nothing.
    """
    x0, y0, x1, y1 = [float(v) for v in rect]
    edges = (
        (lambda p: p[0] >= x0, lambda a, b: _cut_x(a, b, x0)),
        (lambda p: p[0] <= x1, lambda a, b: _cut_x(a, b, x1)),
        (lambda p: p[1] >= y0, lambda a, b: _cut_y(a, b, y0)),
        (lambda p: p[1] <= y1, lambda a, b: _cut_y(a, b, y1)),
    )
    out = list(poly)
    for inside, cut in edges:
        if not out:
            return []
        src, out = out, []
        prev = src[-1]
        for cur in src:
            if inside(cur):
                if not inside(prev):
                    out.append(cut(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(cut(prev, cur))
            prev = cur
    return out


def _cut_x(a, b, x):
    t = (x - a[0]) / ((b[0] - a[0]) or 1e-15)
    return (x, a[1] + t * (b[1] - a[1]))


def _cut_y(a, b, y):
    t = (y - a[1]) / ((b[1] - a[1]) or 1e-15)
    return (a[0] + t * (b[0] - a[0]), y)


def pad_convex(poly, margin_m):
    """A CONVEX ring grown outward by `margin_m` metres on every side.

    `sector.pad_polygon` is the general (miter-limited, concavity-aware)
    version and is what the planner uses; this is the convex special case, in
    stdlib, because the only polygon this module pads is a convex hull. Each
    edge is pushed out along its outward normal and adjacent offset lines are
    intersected — exact for a convex ring, and `tests/test_frozen_annotations`
    checks it against `sector.pad_polygon` on the real cells.

    THE PAD IS APPLIED HERE, NOT BY THE PLANNER, and that is the point. The
    planner pads and THEN partitions, so on a 1 km plate the two end sectors of
    a `rect`/`principal` split are mostly the 50 m ring hanging over the edge
    of the ground sheet: measured on the six frozen cells, sector 8's bounding
    rectangle was 57,000-82,000 m2 of which only 7,600-12,300 m2 was actually
    on the plate. Padding first and CLIPPING TO THE PLATE gives every drone a
    sector made of ground, and the scene overlay then sets
    `search_area_pad_m: 0.0` so the pad is not applied twice.
    """
    m = float(margin_m)
    n = len(poly)
    if n < 3 or m == 0.0:
        return [(float(x), float(y)) for x, y in poly]
    ring = [(float(x), float(y)) for x, y in poly]
    if _signed_area(ring) < 0.0:      # normals below assume counter-clockwise
        ring = ring[::-1]
    lines = []
    for i in range(len(ring)):
        (x0, y0), (x1, y1) = ring[i], ring[(i + 1) % len(ring)]
        ex, ey = x1 - x0, y1 - y0
        ln = math.hypot(ex, ey)
        if ln < 1e-9:
            continue
        nx, ny = ey / ln, -ex / ln    # outward normal of a CCW ring
        lines.append((nx, ny, nx * (x0 + nx * m) + ny * (y0 + ny * m)))
    out = []
    for i in range(len(lines)):
        a1, b1, c1 = lines[i - 1]
        a2, b2, c2 = lines[i]
        det = a1 * b2 - a2 * b1
        if abs(det) < 1e-12:          # parallel edges: keep the offset vertex
            continue
        out.append(((c1 * b2 - c2 * b1) / det, (a1 * c2 - a2 * c1) / det))
    return out


def _signed_area(poly):
    s = 0.0
    for i, (x, y) in enumerate(poly):
        x2, y2 = poly[(i + 1) % len(poly)]
        s += x * y2 - x2 * y
    return s / 2.0


def affected_polygon(hints_doc, rect=None, classes=DAMAGE_CLASSES):
    """The disaster-affected area of a frozen cell, `[[x, y], ...]` world.

    Convex hull of every damaged object's footprint, clipped to the plate.
    Returns `[]` when a cell carries no damage at all, which the caller must
    treat as an error rather than as "search everywhere" — see
    `region_entries`.
    """
    pts = damage_points(hints_doc, classes)
    if len(pts) < 3:
        return []
    hull = convex_hull(pts)
    if rect is None:
        rect = region_m(hints_doc)
    hull = clip_to_rect(hull, rect)
    return [[round(x, 3), round(y, 3)] for x, y in hull]


SEARCH_PAD_M = 50.0


def search_polygon(hints_doc, rect=None, pad_m=SEARCH_PAD_M,
                   classes=DAMAGE_CLASSES):
    """The polygon a drone is actually allowed to fly: damage + pad, on plate.

    `pad_m` is the same slack `search_area_pad_m` buys on a generated scene —
    the visible scar fingers past the modelled front, a tornado's debris throw
    lands beyond the last damaged house, and a drone working an edge needs room
    to turn. Everything past the plate is then clipped away, because the ground
    sheet ends there and a search area over empty space is a drone told to look
    where there is nothing.
    """
    if rect is None:
        rect = region_m(hints_doc)
    hull = affected_polygon(hints_doc, rect, classes)
    if len(hull) < 3:
        return []
    poly = clip_to_rect(pad_convex(hull, pad_m), rect)
    return [[round(x, 3), round(y, 3)] for x, y in poly]


def region_entries(hints_doc, people_doc=None, scene_usd=None,
                   pad_m=SEARCH_PAD_M, classes=DAMAGE_CLASSES):
    """The `<scene>_region.json` payload for a frozen cell.

    Two polygons, and the overlay flies the SECOND:

      `damage`  the convex hull of the damaged objects — the disaster-affected
                area as the file itself records it, reported so a result table
                can say what fraction of the plate was hit;
      `search`  that hull grown by `pad_m` and clipped to the plate — what the
                scene overlay names in `search_area_scene_key`, with
                `search_area_pad_m: 0.0` so the pad is not applied twice.

    NEITHER IS CALLED `burn` OR `affected`. Those names belong to
    `scene_gen/disaster/region.py` and mean particular things about the FIRE
    MODEL — the ellipse at `elapsed`, and that ellipse grown until it covers
    the survivor staged furthest ahead of the front. Neither is what is
    computed here, and neither can be computed for a tornado cell at all, so
    reusing a name would make two different measurements indistinguishable in a
    results directory six weeks later.
    """
    rect = region_m(hints_doc)
    hull = affected_polygon(hints_doc, rect, classes)
    search = search_polygon(hints_doc, rect, pad_m, classes)
    meta = (hints_doc or {}).get("meta") or {} if isinstance(hints_doc, dict) \
        else {}
    people = people_records(people_doc)
    pts = [(float(r["x"]), float(r["y"])) for r in people
           if _num(r.get("x")) is not None and _num(r.get("y")) is not None]
    in_hull = sum(1 for x, y in pts if point_in_polygon(x, y, hull))
    in_search = sum(1 for x, y in pts if point_in_polygon(x, y, search))
    plate = max(1e-9, (rect[2] - rect[0]) * (rect[3] - rect[1]))
    return [
        {"class": "search", "polygon_xy": search,
         "area_m2": round(polygon_area(search), 1),
         "source": "damage hull + {0:.0f} m pad, clipped to the plate"
                   .format(pad_m),
         "pad_m": float(pad_m)},
        {"class": "damage", "polygon_xy": hull,
         "area_m2": round(polygon_area(hull), 1),
         "source": "convex hull of GT_hints damage classes",
         "damage_classes": list(classes)},
        {"class": "region",
         "polygon_xy": [[rect[0], rect[1]], [rect[2], rect[1]],
                        [rect[2], rect[3]], [rect[0], rect[3]]],
         "area_m2": round(plate, 1)},
        {"class": "meta",
         "scene_config": meta.get("scene_config"),
         "disaster": meta.get("disaster"),
         "seed": meta.get("seed"),
         "people_variant": meta.get("people_variant"),
         "frozen_usd": os.path.basename(scene_usd) if scene_usd else None,
         "people_total": len(people),
         "people_inside_damage": in_hull,
         "people_inside_search": in_search,
         "damage_frac_of_plate": round(polygon_area(hull) / plate, 4),
         "search_frac_of_plate": round(polygon_area(search) / plate, 4)},
    ]


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------------------
# writing
# ---------------------------------------------------------------------------

def write(path, payload):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=1)
    return path


def write_all(scene_name, dirs, people_doc, hints_doc, scene_usd=None,
              quiet=False):
    """Write all three files under `<scene_name>` into every existing dir.

    Returns `{suffix: [paths]}`. Both annotation directories get all three, for
    the reason `scene_annotations.annotation_dirs` gives: the GCS draws the GT
    and the scorer reads it, and a GT that differs between the picture and the
    number is worse than no GT.
    """
    people = people_boxes(people_doc)
    obstacles = obstacle_boxes(hints_doc)
    region = region_entries(hints_doc, people_doc, scene_usd)
    written = {"": [], "_obstacles": [], "_region": []}
    for d in dirs:
        if not os.path.isdir(d):
            continue
        for suffix, payload in (("", people), ("_obstacles", obstacles),
                                ("_region", region)):
            if not payload:
                continue
            written[suffix].append(
                write(os.path.join(d, f"{scene_name}{suffix}.json"), payload))
    if not quiet:
        srch = next((e for e in region if e.get("class") == "search"), {})
        m = region[-1] if region else {}
        print("[frozen-gt] {0}: {1} people, {2} obstacle boxes; damage "
              "{3:.0%} of the plate, search area {4:,.0f} m2 ({5:.0%}), "
              "people inside the search {6}/{7}".format(
                  scene_name, len(people), len(obstacles),
                  m.get("damage_frac_of_plate") or 0.0,
                  srch.get("area_m2") or 0.0,
                  m.get("search_frac_of_plate") or 0.0,
                  m.get("people_inside_search") or 0,
                  m.get("people_total") or 0))
        for suffix in ("", "_obstacles", "_region"):
            for p in written[suffix]:
                print(f"[frozen-gt]   -> {p}")
        if not any(written.values()):
            print("[frozen-gt]   WARNING: no annotation directory existed; "
                  "nothing was written")
    return written


def distance_to_farthest_vertex(x, y, poly):
    """Metres from `(x, y)` to the farthest vertex of `poly` — the half-extent
    a `frame_mode: local` grid centred there needs to contain it."""
    return max((math.hypot(px - x, py - y) for px, py in poly), default=0.0)
