#!/usr/bin/env python3
"""tag_asset_materials.py — click through an asset set's buildings and record each one's material.

    AirStack/.venv/bin/python scene_gen/tools/tag_asset_materials.py \
        scene_gen/config/asset_sets/urban.yaml

A small Tk window shows one asset at a time — its path, the categories it is
pooled under, its tags and its per-entry overrides — and four buttons put it in
`wood`, `concrete`, `brick` or `steel`. You can skip an asset, step back, or
jump to any index. Keys work too: 1-4 pick a material, `s` skips, arrows step,
`q` quits.

BUILDINGS ONLY, BY DEFAULT
--------------------------
Only assets pooled under `buildings` are listed — `buildings.intact`,
`.damaged`, `.destroyed`, `.midrise`, `.rowhouse`, `.tower`. Street furniture,
trees, vehicles and ground tiles are skipped: a material tag means something
for a structure and very little for a traffic cone.

`--category` retargets that (`--category debris`, or `--category ''` for every
asset in the set). It matches on the category path, so `buildings` takes
`buildings.*` but never a `bus_stops` that merely shares a prefix.

Buildings live in the *_nucleus sets and in urban.yaml — `shared.yaml` has none,
and the tool says so rather than opening an empty window.

`--list` prints the assets it would show and exits, which needs no display and
is the way to check what a set contains before starting.

WHAT IT WRITES, AND WHAT IT DOES NOT TOUCH
------------------------------------------
The input asset set is opened read-only and never written. Choices go to a
separate sidecar YAML (`--out`, default `<input>.materials.yaml`), so nothing
here can disturb a live scene config: the sidecar is a record of decisions for
you to act on deliberately, not something the generator reads.

The sidecar is rewritten after every click — via a temp file and `os.replace`,
so an interrupted write cannot truncate it — which makes the session
resumable. Re-running picks up at the first asset with no decision recorded.

    assets:
      "CityPark/Props/SM_Stump.prop.usd":
        material: wood                 # null means skipped: seen, no material
        categories: [trees]
        tags: [stump]

An asset absent from `assets:` was never reached, which is why skipping records
`null` rather than dropping the entry — "I looked and none apply" and "I never
got there" are different answers and the counts keep them apart.

ONE ROW PER ASSET, NOT PER YAML ENTRY
-------------------------------------
The same USD is often pooled under several categories (a stump is both `trees`
and park scatter). Material is a property of the asset, not of the pool it is
drawn from, so entries are de-duplicated by path and the categories are merged
into one card — you answer once, and `categories:` records everywhere it came
from.

`extends:` is NOT followed. Only the assets written in the file you pass are
listed, so running it on a set that extends another covers just that set's own
additions; run it on the parent separately. This is deliberate — following the
chain would silently re-ask about inherited assets under a different filename.

NO ASSET PREVIEW, AND WHY
-------------------------
These are props, and unlike the material libraries they carry no Nucleus
`.thumbs/` (checked: `ERROR_NOT_FOUND` on the `Props/` thumb directories), and
their USD payloads reference siblings by relative path that cannot be resolved
off-server. Rendering one would mean Isaac Sim or Blender per asset, which is
not something to put behind a click. So the card is textual — the filenames are
descriptive enough to judge from in practice.

If you do render a preview set later, `--previews DIR` will show `DIR/<stem>.png`
beside the card with no further changes here.
"""

from __future__ import annotations

import argparse
import datetime
import os
import sys
import tempfile

import yaml

MATERIALS = ("wood", "concrete", "brick", "steel")

#: Per-entry keys that are not metadata worth showing on the card.
_SKIP_KEYS = {"usd", "tags"}


# ---------------------------------------------------------------------------
# reading the asset set
# ---------------------------------------------------------------------------

def walk_usds(node, prefix=()):
    """Yield ``(category_path, entry_dict)`` for every asset under `usds:`.

    Handles all three shapes the sets use: a bare `"path.usd"` string, the
    `{usd: …, scale: …, tags: […]}` dict, and a named map of either
    (`tiles: {concrete: "…"}`), nested to any depth.
    """
    if isinstance(node, str):
        yield prefix, {"usd": node}
    elif isinstance(node, dict):
        if "usd" in node:                       # an entry, not a category map
            yield prefix, node
        else:
            for key, val in node.items():
                yield from walk_usds(val, prefix + (str(key),))
    elif isinstance(node, list):
        for item in node:
            yield from walk_usds(item, prefix)


def in_category(rec: dict, prefix: str) -> bool:
    """True when the asset is pooled under *prefix* or a child of it.

    Matched on the category path, so `buildings` takes `buildings.intact`,
    `buildings.damaged`, … but never a `bus_stops` that merely starts with the
    same letters.
    """
    return any(c == prefix or c.startswith(prefix + ".")
               for c in rec["categories"])


def load_assets(path: str, category: str | None = None) -> tuple:
    """``(assets, doc)`` — one record per distinct USD, in file order.

    *category* keeps only assets pooled under that category path. Filtering
    happens after de-duplication, so an asset used both as a building and as
    something else is still matched on its full set of categories.
    """
    with open(path) as fh:
        doc = yaml.safe_load(fh) or {}

    by_path, order = {}, []
    for cat_path, entry in walk_usds(doc.get("usds") or {}):
        usd = entry.get("usd")
        if not isinstance(usd, str) or not usd:
            continue
        if usd not in by_path:
            by_path[usd] = {"usd": usd, "categories": [], "tags": [], "extra": {}}
            order.append(usd)
        rec = by_path[usd]
        cat = ".".join(cat_path)
        if cat and cat not in rec["categories"]:
            rec["categories"].append(cat)
        for tag in entry.get("tags") or []:
            if tag not in rec["tags"]:
                rec["tags"].append(tag)
        for key, val in entry.items():
            if key not in _SKIP_KEYS:
                rec["extra"][key] = val
    records = [by_path[p] for p in order]
    if category:
        records = [r for r in records if in_category(r, category)]
    return records, doc


# ---------------------------------------------------------------------------
# the sidecar
# ---------------------------------------------------------------------------

def load_choices(out: str) -> dict:
    """``{usd: material_or_None}`` for assets already decided. Missing = unseen."""
    if not os.path.exists(out):
        return {}
    with open(out) as fh:
        doc = yaml.safe_load(fh) or {}
    return {k: (v or {}).get("material")
            for k, v in (doc.get("assets") or {}).items()}


def save_choices(out: str, source: str, assets: list, choices: dict,
                 category: str | None = None) -> None:
    """Rewrite the sidecar. Temp file + replace, so a crash cannot truncate it."""
    # Anything already in the file that is outside the current filter is carried
    # through untouched. Without this, tagging with --category buildings would
    # silently drop decisions made in an earlier run over the whole set.
    mine = {rec["usd"] for rec in assets}
    body = {}
    if os.path.exists(out):
        with open(out) as fh:
            prior = (yaml.safe_load(fh) or {}).get("assets") or {}
        body = {k: v for k, v in prior.items() if k not in mine}
    carried = len(body)

    for rec in assets:
        if rec["usd"] not in choices:
            continue
        body[rec["usd"]] = {
            "material": choices[rec["usd"]],
            "categories": rec["categories"],
            "tags": rec["tags"],
        }

    # Tally only what is in scope now. `choices` is seeded from the whole file,
    # so entries outside the current category would otherwise be counted as
    # progress and leave `undecided` short.
    ours = {k: v for k, v in choices.items() if k in mine}
    counts = {m: 0 for m in MATERIALS}
    counts["skipped"] = 0
    for mat in ours.values():
        key = "skipped" if mat is None else mat
        counts[key] = counts.get(key, 0) + 1
    counts["undecided"] = len(assets) - len(ours)
    if carried:
        counts["carried_over"] = carried

    header = (
        "# Asset material tags — written by scene_gen/tools/tag_asset_materials.py\n"
        "#\n"
        "# A record of decisions, NOT something the generator reads. Nothing here\n"
        "# has been applied to the asset set; do that deliberately.\n"
        "#\n"
        "# material: null means the asset was skipped — seen, no material chosen.\n"
        "# An asset missing from `assets:` was never reached.\n"
    )
    doc = {
        "source": source,
        "category": category or "(all)",
        "updated": datetime.datetime.now().isoformat(timespec="seconds"),
        "materials": list(MATERIALS),
        "counts": counts,
        "assets": body,
    }
    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(out)),
                               suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as fh:
            fh.write(header)
            yaml.safe_dump(doc, fh, sort_keys=False, default_flow_style=False)
        os.replace(tmp, out)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


# ---------------------------------------------------------------------------
# session state — everything the buttons do, with no Tk in it
# ---------------------------------------------------------------------------

class Session:
    """Where we are and what has been decided. The GUI is a view over this.

    Kept free of Tk so the part that can actually lose your work — recording a
    choice and writing the sidecar — is testable without a display.
    """

    def __init__(self, assets: list, out: str, source: str,
                 category: str | None = None):
        self.assets, self.out, self.source = assets, out, source
        self.category = category
        self.choices = load_choices(out)
        self.i = next((n for n, a in enumerate(assets)
                       if a["usd"] not in self.choices), 0)

    @property
    def current(self) -> dict:
        return self.assets[self.i]

    @property
    def decided(self) -> int:
        return sum(1 for a in self.assets if a["usd"] in self.choices)

    def recorded(self):
        """The choice for the current asset: a material, None (skipped), or
        the string "undecided" — which `None` alone could not express."""
        usd = self.current["usd"]
        return self.choices[usd] if usd in self.choices else "undecided"

    def choose(self, material) -> None:
        """Record *material* (None = skipped), persist, and move on."""
        self.choices[self.current["usd"]] = material
        self.save()
        self.step(1)

    def step(self, delta: int) -> None:
        self.i = max(0, min(len(self.assets) - 1, self.i + delta))

    def jump(self, n: int) -> str | None:
        """Go to 1-based index *n*. Returns an error message, or None on success."""
        if not 1 <= n <= len(self.assets):
            return f"jump: out of range (1..{len(self.assets)})"
        self.i = n - 1
        return None

    def save(self) -> None:
        save_choices(self.out, self.source, self.assets, self.choices,
                     self.category)


# ---------------------------------------------------------------------------
# the GUI
# ---------------------------------------------------------------------------

def run_gui(assets: list, out: str, source: str, previews: str | None,
            category: str | None = None) -> int:
    import tkinter as tk
    from tkinter import ttk

    session = Session(assets, out, source, category)

    root = tk.Tk()
    root.title(f"Asset materials — {os.path.basename(source)}")
    root.geometry("760x560")
    root.minsize(620, 480)

    state = {"photo": None}

    outer = ttk.Frame(root, padding=14)
    outer.pack(fill="both", expand=True)

    progress = ttk.Label(outer, font=("TkDefaultFont", 10))
    progress.pack(anchor="w")

    name = ttk.Label(outer, font=("TkDefaultFont", 15, "bold"), wraplength=700)
    name.pack(anchor="w", pady=(8, 0))

    path_lbl = ttk.Label(outer, foreground="#666", wraplength=700)
    path_lbl.pack(anchor="w")

    image_lbl = ttk.Label(outer)
    image_lbl.pack(anchor="w", pady=4)

    detail = ttk.Label(outer, wraplength=700, justify="left")
    detail.pack(anchor="w", pady=(10, 0))

    current = ttk.Label(outer, font=("TkDefaultFont", 11, "bold"))
    current.pack(anchor="w", pady=(10, 0))

    ttk.Separator(outer).pack(fill="x", pady=12)

    buttons = ttk.Frame(outer)
    buttons.pack(anchor="w")

    nav = ttk.Frame(outer)
    nav.pack(anchor="w", pady=(10, 0))

    status = ttk.Label(outer, foreground="#666", wraplength=700)
    status.pack(anchor="w", side="bottom")

    def show() -> None:
        rec = session.current
        left = len(assets) - session.decided
        progress.config(
            text=f"asset {session.i + 1} of {len(assets)}     "
                 f"{session.decided} decided, {left} to go")
        name.config(text=os.path.basename(rec["usd"]))
        path_lbl.config(text=rec["usd"])

        bits = [f"categories:  {', '.join(rec['categories']) or '—'}",
                f"tags:        {', '.join(rec['tags']) or '—'}"]
        for key, val in rec["extra"].items():
            bits.append(f"{key + ':':13s}{val}")
        detail.config(text="\n".join(bits))

        mat = session.recorded()
        if mat == "undecided":
            current.config(text="not yet decided", foreground="#888")
        else:
            current.config(text=f"recorded: {mat or 'skipped'}",
                           foreground="#1a7f37" if mat else "#996600")

        state["photo"] = None
        image_lbl.config(image="")
        if previews:
            # Through PIL, not tk.PhotoImage: that reads no JPEG and cannot
            # scale, so a full-size render would either raise or shove the
            # buttons off-screen.
            stem = os.path.splitext(os.path.basename(rec["usd"]))[0]
            for ext in (".png", ".jpg", ".jpeg"):
                full = os.path.join(previews, stem + ext)
                if not os.path.exists(full):
                    continue
                try:
                    from PIL import Image, ImageTk
                    img = Image.open(full)
                    img.thumbnail((260, 260), Image.LANCZOS)
                    state["photo"] = ImageTk.PhotoImage(img)
                    image_lbl.config(image=state["photo"])
                except Exception:                                # noqa: BLE001
                    pass                       # a bad preview is not fatal
                break

    def choose(material) -> None:
        session.choose(material)
        status.config(text=f"saved -> {out}")
        show()

    def step(delta: int) -> None:
        session.step(delta)
        show()

    def jump() -> None:
        try:
            idx = int(jump_var.get())
        except ValueError:
            status.config(text="jump: type an asset number")
            return
        err = session.jump(idx)
        if err:
            status.config(text=err)
            return
        root.focus_set()            # give the keyboard shortcuts back
        show()

    for n, material in enumerate(MATERIALS, start=1):
        ttk.Button(buttons, text=f"{material}  ({n})", width=14,
                   command=lambda m=material: choose(m)).pack(side="left", padx=3)

    ttk.Button(nav, text="skip  (s)", width=12,
               command=lambda: choose(None)).pack(side="left", padx=3)
    ttk.Button(nav, text="back  (←)", width=10,
               command=lambda: step(-1)).pack(side="left", padx=3)
    ttk.Button(nav, text="next  (→)", width=10,
               command=lambda: step(1)).pack(side="left", padx=3)

    ttk.Label(nav, text="jump to #").pack(side="left", padx=(18, 0))
    jump_var = tk.StringVar()
    jump_entry = ttk.Entry(nav, textvariable=jump_var, width=7)
    jump_entry.pack(side="left", padx=4)
    ttk.Button(nav, text="go", width=5, command=jump).pack(side="left")
    jump_entry.bind("<Return>", lambda _e: jump())

    def key(event) -> None:
        # Digits belong to the jump box while it has focus.
        if root.focus_get() is jump_entry:
            return
        if event.keysym in ("1", "2", "3", "4"):
            choose(MATERIALS[int(event.keysym) - 1])
        elif event.keysym.lower() == "s":
            choose(None)
        elif event.keysym == "Left":
            step(-1)
        elif event.keysym == "Right":
            step(1)
        elif event.keysym.lower() == "q":
            root.destroy()

    root.bind("<Key>", key)
    status.config(text=f"writing to {out}")
    show()
    root.mainloop()

    session.save()
    print(f"{session.decided} of {len(assets)} assets decided -> {out}")
    return 0


# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("asset_set", help="asset set YAML to read (never written)")
    ap.add_argument("-o", "--out",
                    help="sidecar to write (default: <asset_set>.materials.yaml)")
    ap.add_argument("--category", default="buildings", metavar="PATH",
                    help="only assets pooled under this category "
                         "(default: buildings; '' for every asset in the set)")
    ap.add_argument("--previews", metavar="DIR",
                    help="show DIR/<asset stem>.png beside each card, if present")
    ap.add_argument("--list", action="store_true",
                    help="print the assets that would be shown, then exit")
    args = ap.parse_args()

    assets, doc = load_assets(args.asset_set, args.category)
    if not assets:
        parent = doc.get("extends")
        what = f"`{args.category}` assets" if args.category else "assets"
        hint = ""
        if args.category:
            # Distinguish "this set has none of that category" from "this set
            # has nothing at all" — the fix differs.
            everything, _ = load_assets(args.asset_set)
            if everything:
                cats = sorted({c.split(".")[0] for r in everything
                               for c in r["categories"]})
                hint = (f" It has {len(everything)} other assets, under: "
                        f"{', '.join(cats)}.")
        if not hint and parent:
            hint = (f" It only overrides '{parent}'. Run this on "
                    f"{parent}.yaml instead.")
        print(f"no {what} in {args.asset_set}.{hint}", file=sys.stderr)
        return 1
    if doc.get("extends"):
        print(f"note: {os.path.basename(args.asset_set)} extends "
              f"'{doc['extends']}', which is NOT followed — only this file's "
              f"own {len(assets)} "
              f"{args.category or 'asset'}{'' if args.category else 's'} "
              f"are listed.", file=sys.stderr)

    if args.list:
        for n, rec in enumerate(assets, start=1):
            print(f"{n:4d}  {rec['usd']}"
                  f"   [{', '.join(rec['categories'])}]"
                  f"{'  tags=' + ','.join(rec['tags']) if rec['tags'] else ''}")
        return 0

    # The category goes in the name so tagging buildings and tagging
    # everything cannot land in the same file by default.
    slug = (args.category or "all").replace(".", "_")
    out = args.out or f"{os.path.splitext(args.asset_set)[0]}.{slug}.materials.yaml"
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
            or sys.platform in ("darwin", "win32")):
        print("no display — this is a GUI tool. Use --list to see the assets.",
              file=sys.stderr)
        return 2
    return run_gui(assets, out, args.asset_set, args.previews, args.category)


if __name__ == "__main__":
    raise SystemExit(main())
