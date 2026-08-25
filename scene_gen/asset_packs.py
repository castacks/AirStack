#!/usr/bin/env python3
"""asset_packs.py — split a multi-object USD "pack" into its separable props.

Several Objaverse entries in the asset sets are **packs**: one file holding a
whole tray of props (a dozen road signs laid out in a row, a barrier-and-cone
set, a bike rack with bikes already at it). Referencing one lands the entire
cluster, and ``target-size-m`` then sizes the *spread* rather than any single
item — a 4 m "sign" that is really eight 2 m signs side by side.

The sub-objects are individually addressable once the pack is on the stage, so
the fix is to reference a **sub-prim** instead of the pack's default prim.
This module finds those sub-prims and measures them:

    from asset_packs import split_pack, sub_objects

    split = split_pack("objaverse://7fac2b1b6c5746daa4ac773441c45180")
    for s in split.subs:
        print(s.prim_path, s.size_m, s.base_z)

    # the bare (path, size, base) triples, separable ones only
    sub_objects("objaverse://7fac2b1b6c5746daa4ac773441c45180")

How the split is taken
----------------------
Exporters wrap the interesting geometry in a chain of single-child bookkeeping
Xforms (``/root/Sketchfab_model/<name>_fbx/RootNode/...``). Walking straight
into the default prim's children would return one node every time, so the
splitter first **descends the spine**: while the current prim has exactly one
geometry-bearing child, it steps into it. The first prim with two or more such
children is the *branch node*, and its children are the pack's props — the
grouping the author actually made.

Each candidate is then measured (world-space bbox through ``BBoxCache``,
converted to metres) and tested for **footprint disjointness** against its
siblings. ``separable`` means "no sibling occupies my patch of ground", i.e.
this prim can be referenced on its own without slicing a neighbour in half.

Footprint, not full 3D box, and that is the load-bearing choice. A parking
meter's pole, head and lens are three boxes that never intersect in 3D, so a 3D
test happily "splits" one prop into three fragments; they all stand on one
patch of ground, so a footprint test keeps them together. Props in a real pack
are laid out side by side, which the footprint test still separates cleanly.
``--overlap-mode bbox3d`` is there when you want the old behaviour.

Deliberately only one level deep by default: ``--depth`` opens it up when a
pack really does nest (a pack of trays of props), and ``--min-size`` /
``--min-points`` drop the fragments that shakes loose.

Run it (plain pxr — no SimulationApp, no Nucleus needed for local packs)::

    docker run --rm -v <repo>:/isaac-sim/AirStack --entrypoint bash <image> -c '
      U=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)
      export PYTHONPATH="$U:$PYTHONPATH"; export LD_LIBRARY_PATH="$U/bin:$LD_LIBRARY_PATH"
      cd /isaac-sim && ./python.sh /isaac-sim/AirStack/scene_gen/asset_packs.py \
        objaverse://7fac2b1b6c5746daa4ac773441c45180 --yaml'

Columns printed by the CLI:
    sub-prim   path to reference, relative to the pack's default prim
    x/y/z      the sub-object's own bounding box, in METRES
    base       bbox minimum along the stage up axis — the ground offset
    pts        Mesh points in that subtree; the poly-budget share
    sep        "yes" when no sibling box overlaps this one

Note for whoever wires this into an asset set: ``scene_generator`` currently
authors ``prim.GetReferences().AddReference(usd)`` with no prim path, so it
targets the default prim. Referencing a sub-object needs the two-argument form,
``AddReference(usd, Sdf.Path(sub_prim_path))``. ``--yaml`` prints entries with a
``prim-path`` key on that assumption; it is a config shape to agree on, not one
the generator reads today.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, field, asdict

try:
    from pxr import Usd, UsdGeom
except ImportError as exc:                                  # pragma: no cover
    raise SystemExit(
        "asset_packs needs pxr. Run it inside the isaac-sim container with\n"
        '  U=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)\n'
        '  PYTHONPATH="$U" LD_LIBRARY_PATH="$U/bin" ./python.sh ...\n'
        f"(import error: {exc})"
    )


# ---------------------------------------------------------------------------
# asset URL resolution
#
# Mirrors scene_generator.LOCAL_ASSET_ROOTS rather than importing it: this
# module has to run under bare pxr, and importing scene_generator drags in the
# whole generator. The two must agree, so keep them in step.
# ---------------------------------------------------------------------------

SCENE_GEN_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(SCENE_GEN_DIR)

LOCAL_ASSET_ROOTS = {
    "airstack": _REPO_ROOT,
    "objaverse": os.path.join(SCENE_GEN_DIR, "assets", "objaverse"),
}

_UID_RE = re.compile(r"^[0-9a-fA-F]{32}$")

# Prim types that never carry geometry — skipped when hunting for sub-objects
# so a Material or Shader sibling can't be mistaken for a prop. Scope is NOT
# here: exporters do park real geometry under one.
_NON_GEOM_TYPES = {"Material", "Shader", "NodeGraph", "GeomSubset",
                   "Camera", "SphereLight", "DistantLight", "DomeLight",
                   "RectLight", "DiskLight", "CylinderLight"}

_GEOM_TYPES = {"Mesh", "PointInstancer", "BasisCurves", "NurbsCurves",
               "Points", "Capsule", "Cone", "Cube", "Cylinder", "Sphere"}


def resolve_asset(path: str) -> str:
    """Expand ``objaverse://``/``airstack://`` (and a bare 32-hex Objaverse
    uid) to an absolute path. Real URLs and plain paths pass through.
    """
    path = str(path or "").strip()
    if _UID_RE.match(path):
        path = "objaverse://" + path
    scheme, sep, rest = path.partition("://")
    if not sep or scheme not in LOCAL_ASSET_ROOTS:
        return os.path.expanduser(path)
    rest = rest.lstrip("/")
    if scheme == "objaverse" and _UID_RE.match(rest):
        rest = os.path.join(rest, rest + ".usdc")
    return os.path.join(LOCAL_ASSET_ROOTS[scheme], rest)


# ---------------------------------------------------------------------------
# results
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SubObject:
    """One separable prop inside a pack."""

    prim_path: str                       #: absolute prim path on the pack stage
    name: str                            #: leaf prim name
    type_name: str                       #: prim type ("Xform", "Mesh", …)
    size_m: tuple                        #: (x, y, z) bbox size in metres
    base_z: float                        #: bbox min along the stage up axis, metres
    center_m: tuple                      #: bbox centre in metres, pack-relative
    points: int                          #: Mesh points in this subtree
    separable: bool                      #: no sibling bbox meaningfully overlaps
    overlaps: tuple = ()                 #: names of siblings whose boxes it shares

    @property
    def footprint_m(self) -> float:
        """Largest horizontal dimension — what ``fit: footprint`` would size."""
        return max(self.size_m[0], self.size_m[1])

    @property
    def height_m(self) -> float:
        return self.size_m[2]


@dataclass
class PackSplit:
    """Everything found in one pack USD."""

    usd: str                             #: path as given
    resolved: str                        #: path actually opened
    default_prim: str
    branch_path: str                     #: prim the split was taken at
    meters_per_unit: float
    up_axis: str
    whole_size_m: tuple                  #: the pack's own bbox, metres
    whole_points: int
    subs: list = field(default_factory=list)   #: list[SubObject]

    @property
    def separable(self) -> list:
        return [s for s in self.subs if s.separable]

    def is_pack(self, min_subs: int = 2) -> bool:
        """True when the file holds several separable props rather than one."""
        return len(self.separable) >= min_subs

    def to_dict(self) -> dict:
        d = asdict(self)
        d["subs"] = [asdict(s) for s in self.subs]
        return d


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

def _count_points(prim) -> int:
    """Mesh points in *prim*'s subtree, instance proxies included.

    Instanced prims are the trap here: a plain ``Usd.PrimRange(prim)`` walks
    straight past an instance's prototype and reports zero, which reads as "no
    geometry" and makes the whole subtree invisible to the splitter.
    """
    total = 0
    rng = Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
    for p in rng:
        if p.GetTypeName() == "Mesh":
            attr = p.GetAttribute("points")
            if attr:
                val = attr.Get()
                if val:
                    total += len(val)
    return total


def _has_geometry(prim) -> bool:
    if prim.GetTypeName() in _GEOM_TYPES:
        return True
    rng = Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
    for p in rng:
        if p.GetTypeName() in _GEOM_TYPES:
            return True
    return False


def _geom_children(prim) -> list:
    """Children that could be props: not materials/shaders, and non-empty."""
    out = []
    for child in prim.GetChildren():
        if child.GetTypeName() in _NON_GEOM_TYPES:
            continue
        if not _has_geometry(child):
            continue
        out.append(child)
    return out


def _up_index(up_axis: str) -> int:
    return 1 if str(up_axis).upper().startswith("Y") else 2


def _bbox(cache, prim, mpu: float):
    """(size, min, max) in metres, or None when the prim bounds nothing."""
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    mn, mx = rng.GetMin(), rng.GetMax()
    size = tuple((mx[i] - mn[i]) * mpu for i in range(3))
    return size, tuple(mn[i] * mpu for i in range(3)), tuple(mx[i] * mpu for i in range(3))


#: How separability is judged. ``footprint`` compares the XY footprints only;
#: ``bbox3d`` compares the full boxes.
OVERLAP_MODES = {"footprint": (0, 1), "bbox3d": (0, 1, 2)}


def _overlap_fraction(a, b, axes=(0, 1), eps: float = 1e-4) -> float:
    """How much two axis-aligned boxes share, as a fraction of the smaller.

    Measured over *axes* only. The default is the **XY footprint**, and that
    choice is the whole heuristic: the parts of a single prop stack vertically
    and sit on top of each other's footprint (a parking meter's pole, head and
    lens are three boxes that never intersect in 3D but occupy one patch of
    ground), whereas the props in a pack are laid out side by side. Comparing
    full 3D boxes shreds every multi-part prop into pieces; comparing
    footprints keeps them whole and still separates a row of signs.

    Extent ratio where both boxes have extent. Packs are full of flat props —
    a sign plate is millimetres thick and has ~0 area — so when either box is
    degenerate the score falls back to the smallest per-axis ratio.
    """
    (amn, amx), (bmn, bmx) = a, b
    ov = [min(amx[i], bmx[i]) - max(amn[i], bmn[i]) for i in axes]
    if any(o <= eps for o in ov):
        return 0.0                              # a gap on any axis ⇒ disjoint

    ext_a = [amx[i] - amn[i] for i in axes]
    ext_b = [bmx[i] - bmn[i] for i in axes]

    def _prod(vals):
        out = 1.0
        for v in vals:
            out *= v
        return out

    va, vb = _prod(ext_a), _prod(ext_b)
    if va > eps and vb > eps:
        return _prod(ov) / min(va, vb)

    fracs = []
    for k in range(len(axes)):
        for ext in (ext_a[k], ext_b[k]):
            if ext > eps:
                fracs.append(min(1.0, ov[k] / ext))
    return min(fracs) if fracs else 1.0


# ---------------------------------------------------------------------------
# the split
# ---------------------------------------------------------------------------

def open_pack(usd: str):
    """Open a pack USD. Returns (stage, root_prim, meters_per_unit, up_axis)."""
    resolved = resolve_asset(usd)
    stage = Usd.Stage.Open(resolved)
    if stage is None:
        raise IOError(f"failed to open {resolved}")
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    up = str(UsdGeom.GetStageUpAxis(stage) or "Z")
    dp = stage.GetDefaultPrim()
    root = dp if (dp and dp.IsValid()) else stage.GetPseudoRoot()
    return stage, root, mpu, up


def find_branch(root):
    """Walk past single-child wrapper Xforms to the first real branch.

    Returns the prim whose children are the pack's props (or *root* itself when
    the file holds a single object and never branches).
    """
    prim = root
    seen = 0
    while seen < 32:                              # cycle guard, not a real limit
        kids = _geom_children(prim)
        if len(kids) != 1:
            return prim
        prim = kids[0]
        seen += 1
    return prim


def split_pack(usd: str,
               *,
               min_size_m: float = 0.05,
               min_points: int = 8,
               depth: int = 1,
               overlap_frac: float = 0.05,
               overlap_mode: str = "footprint") -> PackSplit:
    """Enumerate the separable sub-objects of a pack USD.

    :param usd:          path, ``objaverse://<uid>``, ``airstack://<path>``,
                         a bare Objaverse uid, or any URL pxr can open.
    :param min_size_m:   drop candidates whose largest dimension is under this
                         — screws, decals and stray verts.
    :param min_points:   drop candidates with fewer Mesh points than this.
    :param depth:        how many levels below the branch node to expand. 1 is
                         the author's own grouping and is almost always right;
                         raise it only for packs of packs.
    :param overlap_frac: two boxes count as overlapping once they share more
                         than this fraction of the smaller one. 0.05 tolerates
                         props that just touch (a cone's base on a barrier's
                         foot) without calling them one object.
    :param overlap_mode: ``footprint`` (default) compares XY footprints, which
                         keeps a multi-part prop whole; ``bbox3d`` compares the
                         full boxes and will split one prop into its parts.
    """
    if overlap_mode not in OVERLAP_MODES:
        raise ValueError(f"overlap_mode must be one of {sorted(OVERLAP_MODES)}")
    axes = OVERLAP_MODES[overlap_mode]
    stage, root, mpu, up = open_pack(usd)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    ui = _up_index(up)

    whole = _bbox(cache, root, mpu)
    whole_size = whole[0] if whole else (0.0, 0.0, 0.0)

    branch = find_branch(root)

    # Expand `depth` levels, keeping a node whenever it refuses to branch.
    level = _geom_children(branch)
    for _ in range(max(0, depth - 1)):
        nxt = []
        for prim in level:
            kids = _geom_children(prim)
            nxt.extend(kids if len(kids) > 1 else [prim])
        if nxt == level:
            break
        level = nxt

    # Measure, then filter.
    raw = []
    for prim in level:
        bb = _bbox(cache, prim, mpu)
        if bb is None:
            continue
        size, mn, mx = bb
        pts = _count_points(prim)
        if max(size) < min_size_m or pts < min_points:
            continue
        raw.append((prim, size, mn, mx, pts))

    # Pairwise disjointness over the survivors.
    subs = []
    for i, (prim, size, mn, mx, pts) in enumerate(raw):
        clash = []
        for j, (other, _s, omn, omx, _p) in enumerate(raw):
            if i == j:
                continue
            if _overlap_fraction((mn, mx), (omn, omx), axes) > overlap_frac:
                clash.append(other.GetName())
        subs.append(SubObject(
            prim_path=str(prim.GetPath()),
            name=prim.GetName(),
            type_name=str(prim.GetTypeName()),
            size_m=tuple(round(v, 4) for v in size),
            base_z=round(mn[ui], 4),
            center_m=tuple(round((mn[k] + mx[k]) / 2.0, 4) for k in range(3)),
            points=pts,
            separable=not clash,
            overlaps=tuple(sorted(clash)),
        ))

    subs.sort(key=lambda s: -max(s.size_m))
    return PackSplit(
        usd=usd,
        resolved=resolve_asset(usd),
        default_prim=str(root.GetPath()),
        branch_path=str(branch.GetPath()),
        meters_per_unit=mpu,
        up_axis=up,
        whole_size_m=tuple(round(v, 4) for v in whole_size),
        whole_points=_count_points(root),
        subs=subs,
    )


def sub_objects(usd: str, *, separable_only: bool = True, **kw) -> list:
    """The plain answer: ``[(sub_prim_path, bbox_size_m, base_z), …]``.

    ``bbox_size_m`` is the sub-object's own size in metres, so a caller sizing
    it with ``target-size-m`` finally sizes *the prop* rather than the pack's
    spread. ``base_z`` is its bbox minimum along the stage up axis — add it to
    the ground height the way the generator already does for whole assets.
    """
    split = split_pack(usd, **kw)
    chosen = split.separable if separable_only else split.subs
    return [(s.prim_path, s.size_m, s.base_z) for s in chosen]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _rel(path: str, root: str) -> str:
    return path[len(root):] if root != "/" and path.startswith(root) else path


def _print_split(split: PackSplit, show_all: bool) -> None:
    name = os.path.basename(split.resolved) or split.resolved
    wx, wy, wz = split.whole_size_m
    print(f"\n=== {split.usd}")
    print(f"    file        {name}")
    print(f"    default     {split.default_prim}   "
          f"(mpu {split.meters_per_unit:g}, up {split.up_axis})")
    print(f"    branch      {split.branch_path}")
    print(f"    whole pack  {wx:.2f} x {wy:.2f} x {wz:.2f} m, "
          f"{split.whole_points:,} pts")
    rows = split.subs if show_all else split.separable
    print(f"    sub-objects {len(split.separable)} separable "
          f"of {len(split.subs)} candidates"
          + ("" if split.is_pack() else "   <- NOT A PACK: single object"))
    if not rows:
        return
    print()
    print(f"    {'sub-prim':<50} {'x_m':>7} {'y_m':>7} {'z_m':>7} "
          f"{'base':>7} {'pts':>8}  sep")
    print("    " + "-" * 96)
    for s in rows:
        rel = _rel(s.prim_path, split.branch_path) or "/"
        x, y, z = s.size_m
        print(f"    {rel[-50:]:<50} {x:>7.3f} {y:>7.3f} {z:>7.3f} "
              f"{s.base_z:>7.3f} {s.points:>8,}  "
              f"{'yes' if s.separable else 'no (' + ','.join(s.overlaps)[:24] + ')'}")


def _print_yaml(split: PackSplit, show_all: bool) -> None:
    """Paste-ready asset-set entries, one per separable sub-object.

    ``target-size-m`` is the sub-object's own measured size, so the entry
    reproduces the prop at its authored scale instead of stretching one item to
    the whole pack's footprint.
    """
    rows = split.subs if show_all else split.separable
    if not rows:
        return
    print(f"\n# --- {split.usd}  ({len(rows)} sub-objects) ---")
    for s in rows:
        fit = "height" if s.height_m > s.footprint_m else "footprint"
        target = s.height_m if fit == "height" else s.footprint_m
        print(f'    - {{usd: "{split.usd}", prim-path: "{s.prim_path}", '
              f'target-size-m: {target:.2f}, fit: {fit}}}'
              f'  # {s.name}, {s.points:,} pts')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Enumerate the separable sub-objects of a multi-object USD pack.",
        epilog="Assets may be given as a path, objaverse://<uid>, "
               "airstack://<path>, or a bare 32-hex Objaverse uid.")
    ap.add_argument("usds", nargs="+", help="pack USDs to inspect")
    ap.add_argument("--min-size", type=float, default=0.05, metavar="M",
                    help="drop candidates smaller than this in every axis (default 0.05)")
    ap.add_argument("--min-points", type=int, default=8, metavar="N",
                    help="drop candidates with fewer Mesh points (default 8)")
    ap.add_argument("--depth", type=int, default=1, metavar="N",
                    help="levels to expand below the branch node (default 1)")
    ap.add_argument("--overlap", type=float, default=0.05, metavar="F",
                    help="bbox share above which two props count as one (default 0.05)")
    ap.add_argument("--overlap-mode", choices=sorted(OVERLAP_MODES),
                    default="footprint",
                    help="compare XY footprints (default) or full 3D boxes")
    ap.add_argument("--all", action="store_true",
                    help="list overlapping candidates too, not just separable ones")
    ap.add_argument("--yaml", action="store_true",
                    help="also print paste-ready asset-set entries")
    ap.add_argument("--json", action="store_true",
                    help="emit machine-readable JSON instead of a table")
    args = ap.parse_args(argv)

    splits, failed = [], 0
    for usd in args.usds:
        try:
            splits.append(split_pack(usd,
                                     min_size_m=args.min_size,
                                     min_points=args.min_points,
                                     depth=args.depth,
                                     overlap_frac=args.overlap,
                                     overlap_mode=args.overlap_mode))
        except Exception as exc:
            print(f"  !! {usd}: {type(exc).__name__}: {exc}", file=sys.stderr)
            failed += 1

    if args.json:
        print(json.dumps([s.to_dict() for s in splits], indent=2))
        return 1 if failed and not splits else 0

    for split in splits:
        _print_split(split, args.all)
    if args.yaml:
        for split in splits:
            _print_yaml(split, args.all)

    packs = [s for s in splits if s.is_pack()]
    print(f"\n{len(splits)} file(s) inspected, {len(packs)} hold several props.")
    return 1 if failed and not splits else 0


if __name__ == "__main__":
    sys.exit(main())
