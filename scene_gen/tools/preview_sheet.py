#!/usr/bin/env python3
"""Contact-sheet the previews a bake already took. One row per asset, one
column per rung.

    ┌──────────────┬──────────┬─────────┬─────────────┬──────────────────┬──────────┐
    │              │ pristine │ cracked │ soft_storey │ partial_collapse │ pancaked │
    ├──────────────┼──────────┼─────────┼─────────────┼──────────────────┼──────────┤
    │ house_01 ·25 │          │         │             │                  │          │
    │ block_04     │          │         │             │                  │          │
    └──────────────┴──────────┴─────────┴─────────────┴──────────────────┴──────────┘

RENDERS NOTHING. `archetypes/preview.py` photographed each cell during the
bake, while its geometry was on a lit stage with the debris settled around it;
this only composites those PNGs. That makes it seconds rather than minutes,
and — the real point — it works against a library a bake is still filling, so
a long bake can be watched rather than waited on.

`tools/render_archetypes.py` answers the same question with Cycles, off the
exported USDs. Prefer that for a look check on a finished library: it is
lit properly and framed per row. Prefer this while a bake is running, or when
the question is "did the pipeline do the right thing to the geometry" rather
than "does the art read".

    scene_gen/tools/preview_sheet.py scene_gen/assets/archetypes/earthquake

WHAT THE ROW LABEL CARRIES
--------------------------
The type, and how many of it the census saw the scene place (`·25`). Rows are
ordered by that count, so the assets a scene leans on are at the top of the
sheet — which is also the order they were baked in. An asset with no count was
not seen by any census: stock, not necessarily unused.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from archetypes import library as lib                          # noqa: E402
from archetypes import version as V                            # noqa: E402
from disaster import levels as L                               # noqa: E402


def _font(size, bold=True):
    from PIL import ImageFont
    for name in (f"DejaVuSans{'-Bold' if bold else ''}.ttf",
                 "LiberationSans-Regular.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except Exception:                                        # noqa: BLE001
            continue
    return ImageFont.load_default()


def _rungs(doc: dict) -> list:
    """The ladder, in INTENSITY order — not the order records happen to be in.

    A sheet whose columns are sorted alphabetically puts `cracked` after
    `pancaked` and destroys the one thing it exists to show.
    """
    seen = {str(r.get("level")) for r in doc.get("archetypes", ())}
    order = []
    try:
        order = list(L.bake_levels(str(doc.get("disaster") or "none"),
                                   L.STRUCTURE))
    except Exception:                                            # noqa: BLE001
        pass
    out = [lv for lv in order if lv in seen]
    return out + sorted(seen - set(out))


def build(lib_dir: str, out_path: str = "", view: str = "obl",
          tile_w: int = 340, limit: int = 0,
          kind: str = "structure") -> str:
    from PIL import Image, ImageDraw

    manifest = os.path.join(lib_dir, lib.MANIFEST_NAME)
    doc = lib.read_manifest(manifest)
    recs = doc.get("archetypes") or []
    if not recs:
        raise SystemExit(f"no archetypes in {manifest}")

    # STRUCTURES ONLY, by default. The earthquake ladder for vegetation is a
    # single `pristine` rung -- shaking does not crack or pancake a tree -- so
    # every tree row came out as one tile and four blanks, which reads as a
    # hole in the library rather than as "nothing to damage here".
    # `gallery_picker` has always filtered this way; the sheet now agrees.
    if kind and kind != "all":
        recs = [r for r in recs if str(r.get("kind")) == kind]
        if not recs:
            raise SystemExit(f"no {kind!r} archetypes in {manifest}")

    cols = _rungs(doc)
    by_type: dict = {}
    for r in recs:
        by_type.setdefault(str(r.get("type")), {})[str(r.get("level"))] = r

    def used(t):
        recs_t = by_type[t]
        for r in recs_t.values():
            if r.get("used_by"):
                return sum(int(v) for v in r["used_by"].values())
        return 0

    types = sorted(by_type, key=lambda t: (-used(t), t))
    if limit:
        types = types[:limit]

    # Tile size from the first preview that exists, scaled to `tile_w`.
    first = None
    for t in types:
        for lv in cols:
            p = (by_type[t].get(lv) or {}).get("preview", {}).get(view)
            if p and os.path.isfile(os.path.join(lib_dir, p)):
                first = os.path.join(lib_dir, p)
                break
        if first:
            break
    if not first:
        raise SystemExit(
            f"no '{view}' previews in {lib_dir} — was the library baked with "
            f"SCENE_ARCH_PREVIEW enabled?")
    w0, h0 = Image.open(first).size
    tw = int(tile_w)
    th = max(1, int(round(h0 * tw / float(w0))))

    pad, head, top = 6, 30, 62
    # THE LABEL COLUMN IS SIZED TO THE LONGEST NAME, not fixed. At a flat 210px
    # with the name cut to 26 characters, `Reference_Brownstone11Row` and
    # `SM_MERGED_BP_MBuilding01` ran off the edge -- and a row you cannot name
    # is a row you cannot act on, since every other tool here takes the type.
    _lf = _font(14)
    # Measured on a scratch canvas: the real one cannot exist yet, because its
    # width is what this is computing.
    _probe = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    _wide = 0
    for _t in types:
        try:
            _b = _probe.textbbox((0, 0), _t, font=_lf)
            _wide = max(_wide, _b[2] - _b[0])
        except Exception:                                        # noqa: BLE001
            _wide = max(_wide, 8 * len(_t))
    gutter = max(210, _wide + 3 * pad + 12)
    W = gutter + len(cols) * (tw + pad) + pad
    H = top + head + len(types) * (th + pad) + pad
    sheet = Image.new("RGB", (W, H), (17, 17, 16))
    draw = ImageDraw.Draw(sheet)

    audit = V.audit(doc)
    stale = len(audit["stale"])
    draw.text((pad + 6, 12),
              f"{doc.get('asset_pack', '?')} — {doc.get('disaster', '?')} "
              f"archetypes ({view})",
              fill=(238, 238, 232), font=_font(24))
    draw.text((pad + 6, 40),
              f"{len(recs)} archetypes / {len(by_type)} types · pipeline "
              f"v{doc.get('pipeline_version', '?')} "
              f"{doc.get('pipeline_fingerprint', '')}"
              + (f" · {stale} STALE" if stale else "")
              + f" · baked {doc.get('last_bake_at', '?')}",
              fill=(200, 120, 100) if stale else (140, 140, 134),
              font=_font(13, False))

    for c, col in enumerate(cols):
        x = gutter + c * (tw + pad)
        accent = (235, 235, 228) if col == "pristine" else (232, 168, 96)
        draw.text((x + 4, top + 6), col.upper(), fill=accent, font=_font(16))

    for r, t in enumerate(types):
        y = top + head + r * (th + pad)
        n = used(t)
        draw.text((pad + 6, y + 6), t, fill=(226, 226, 220), font=_lf)
        if n:
            draw.text((pad + 6, y + 24), f"placed {n}x",
                      fill=(150, 200, 150), font=_font(12, False))
        else:
            draw.text((pad + 6, y + 24), "not in census",
                      fill=(120, 120, 116), font=_font(12, False))
        for c, col in enumerate(cols):
            x = gutter + c * (tw + pad)
            rec = by_type[t].get(col)
            if not rec:
                # A GAP IS INFORMATION. A rung that was planned and rejected
                # (settle did not converge, textures resolved to nothing) is
                # exactly what a reviewer is looking for, and a sheet that
                # silently closes the gap hides it.
                draw.rectangle([x, y, x + tw, y + th], outline=(60, 40, 40))
                draw.text((x + 8, y + th // 2), "not baked",
                          fill=(150, 90, 90), font=_font(13, False))
                continue
            p = (rec.get("preview") or {}).get(view)
            ap = os.path.join(lib_dir, p) if p else ""
            if ap and os.path.isfile(ap):
                sheet.paste(Image.open(ap).convert("RGB").resize((tw, th)),
                            (x, y))
            else:
                draw.rectangle([x, y, x + tw, y + th], outline=(50, 50, 50))
                draw.text((x + 8, y + th // 2), "no preview",
                          fill=(110, 110, 110), font=_font(13, False))
            st = rec.get("settle") or {}
            note = (f"{rec.get('usd_mb', '?')} MB · {st.get('bodies', 0)} "
                    f"bodies · {rec.get('seconds', '?')}s")
            if V.is_stale(rec):
                note += "  STALE"
            draw.text((x + 6, y + th - 17), note, fill=(206, 206, 198),
                      font=_font(12, False))

    out = out_path or os.path.join(lib_dir, f"_sheet_{view}.png")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    sheet.save(out)
    print(f"[sheet] {len(types)} types x {len(cols)} rungs -> {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("library", help="a disaster directory in the library "
                                    "(the one holding manifest.json)")
    ap.add_argument("--out", default="")
    ap.add_argument("--view", default="obl", choices=["obl", "top"],
                    help="obl reads as a building; top shows the debris ring")
    ap.add_argument("--tile", type=int, default=340)
    ap.add_argument("--kind", default="structure",
                    help="archetype kind to show; 'all' includes the "
                         "single-rung vegetation ladder")
    ap.add_argument("--limit", type=int, default=0,
                    help="first N types only, most-placed first")
    args = ap.parse_args()
    build(args.library, args.out, args.view, args.tile, args.limit,
          args.kind)


if __name__ == "__main__":
    main()
