#!/usr/bin/env python
"""
objaverse_previews.py — download candidate thumbnails so a human can pick.

Objaverse metadata gets you a shortlist but cannot tell you whether an asset
actually looks right: texture *count* only proves images are in the archive,
not that the materials sample them, and plenty of well-liked models are
dioramas, scans with baked ground, or stylised in a way that clashes. That call
needs eyes.

Sketchfab ships rendered thumbnails for every object and Objaverse's annotations
carry the URLs, so this builds a browsable folder per category **without
downloading a single mesh** — no glb fetch, no Blender conversion, nothing
cached that then has to be cleaned up.

    uv run --no-project --with objaverse --with pandas --with pyyaml \\
        --with trimesh --with requests \\
        scene_gen/tools/objaverse_previews.py --out ~/objaverse_picks --limit 100

Output per category::

    <out>/<category>/003_142L_4772f_1ccd0c76....jpg    rank, likes, faces, uid
    <out>/<category>/index.csv                          uid,name,likes,faces,...

The uid is in the filename on purpose: pick the images you want and the uid can
be read straight off them, with no cross-referencing.
"""

import argparse
import csv
import os
import re
import sys

# Categories the generated city is missing art for. Queries are regexes over
# name+tags+categories; excludes kill the recurring false positives each one
# attracts (searching "bollard" is clean, "sign" is not).
CATEGORIES = {
    "bollard":          ("bollard|parking post", ""),
    "parking_meter":    ("parking meter", ""),
    "bike_rack":        ("bike rack|bicycle rack|bicycle parking", ""),
    "stop_sign":        ("stop sign", "bus stop"),
    "street_name_sign": ("street sign|street name sign|road sign",
                         "neon|medieval|fantasy|tavern"),
    "traffic_sign":     ("traffic sign|road sign|warning sign|speed limit",
                         "neon|medieval|fantasy"),
    "traffic_light":    ("traffic light|traffic signal|stoplight", ""),
    "utility_pole":     ("utility pole|telephone pole|power pole|electric pole",
                         "scifi|sci-fi|rooftop"),
    "dumpster":         ("dumpster|garbage container|waste container|skip bin",
                         "cyberpunk|scifi|sci-fi"),
    "billboard":        ("billboard|advertising board|advert sign",
                         "neon|cyberpunk"),
    "traffic_cone":     ("traffic cone|road cone|safety cone", ""),
    "newspaper_box":    ("newspaper box|vending machine|newspaper stand", ""),
    "manhole":          ("manhole|storm drain|sewer cover|drain grate", ""),
    "ac_unit":          ("air conditioner|hvac|ac unit|rooftop unit|ventilation",
                         "interior"),
    "bench_street":     ("bench", "park bench workout|weight bench|sofa"),
    "phone_booth":      ("phone booth|telephone booth|payphone", ""),

    # ---- suburban park / recreation --------------------------------------
    # A neighbourhood park is mostly SURFACES and FENCES, not props: the courts
    # and the chain-link around them are what make it read as a park rather
    # than as mown grass with equipment dropped on it. Excludes matter more
    # here than for street furniture — every sports query pulls in stadiums,
    # arenas and player characters, none of which belong in a back-of-suburb
    # park, and "pool" without an exclude is almost entirely billiards.
    "basketball_court": ("basketball court|basketball hoop|basketball net",
                         "arena|stadium|nba|player|character|shoe|ball only"),
    "tennis_court":     ("tennis court|tennis net",
                         "arena|stadium|racket only|player|character"),
    "volleyball_court": ("volleyball court|volleyball net|beach volleyball",
                         "arena|stadium|player|character"),
    "baseball_field":   ("baseball field|baseball diamond|backstop|dugout|"
                         "batting cage",
                         "stadium|arena|player|character|glove|bat only"),
    "soccer_goal":      ("soccer goal|football goal|goal post|soccer net",
                         "stadium|arena|player|character"),
    "running_track":    ("running track|athletics track|sports track",
                         "stadium|arena|race car|racetrack"),
    "bleachers":        ("bleachers|grandstand|spectator seating|stadium seat",
                         "arena interior|cinema"),
    "chain_link_fence": ("chain link fence|chainlink|wire mesh fence|"
                         "metal fence|steel fence",
                         "medieval|fantasy|barbed only"),
    "fence_generic":    ("fence|fencing|railing",
                         "medieval|fantasy|castle|electric fence sci"),
    "clubhouse":        ("clubhouse|pavilion|park building|community center|"
                         "recreation center|gazebo",
                         "medieval|fantasy|golf cart"),
    "swimming_pool":    ("swimming pool|pool deck|diving board|pool ladder",
                         "billiard|snooker|pool table|8 ball|cue"),
    "playground":       ("playground|play structure|jungle gym|monkey bars|"
                         "swing set|slide|seesaw|sandbox",
                         "indoor|water park|character"),
    "picnic_table":     ("picnic table|park table|picnic bench", "indoor"),
    "park_shelter":     ("picnic shelter|park pavilion|pergola|shade structure",
                         "medieval|fantasy"),
    "bbq_grill":        ("bbq grill|barbecue|park grill|charcoal grill",
                         "indoor kitchen|gas stove"),
    "drinking_fountain": ("drinking fountain|water fountain|bubbler",
                          "decorative fountain|garden fountain statue"),
    "skate_park":       ("skate park|skate ramp|half pipe|quarter pipe|"
                         "skateboard ramp", "character|skateboard only"),
    "scoreboard":       ("scoreboard|score board", "arena jumbotron"),
    "park_sign":        ("park sign|trail sign|wooden sign post|park entrance",
                         "neon|medieval|tavern|shop sign"),
    "dog_park":         ("dog park|agility equipment|dog agility", "character"),
    "tennis_practice":  ("practice wall|handball court|pickleball court",
                         "arena|stadium"),
}

# Big enough to judge an asset, small enough that a few thousand download fast.
PREFERRED_WIDTH = 720


def _thumb_url(ann, want_w=PREFERRED_WIDTH):
    """Best thumbnail URL at or above *want_w*, else the largest available."""
    imgs = ((ann or {}).get("thumbnails") or {}).get("images") or []
    if not imgs:
        return None
    ok = [i for i in imgs if (i.get("width") or 0) >= want_w]
    pick = min(ok, key=lambda i: i["width"]) if ok else max(
        imgs, key=lambda i: i.get("width") or 0)
    return pick.get("url")


def _safe(s, n=40):
    return re.sub(r"[^A-Za-z0-9._-]+", "-", str(s or ""))[:n].strip("-")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--limit", type=int, default=100,
                    help="candidates per category (default 100)")
    ap.add_argument("--max-faces", type=int, default=80_000,
                    help="drop anything heavier; these are city props")
    ap.add_argument("--min-texture-res", type=int, default=512)
    ap.add_argument("--only", action="append",
                    help="limit to these categories (repeatable)")
    args = ap.parse_args()

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import objaverse
    import requests
    from objaverse_assets import load_catalog, search

    cats = CATEGORIES
    if args.only:
        want = {c.lower() for c in args.only}
        cats = {k: v for k, v in CATEGORIES.items() if k.lower() in want}
        if not cats:
            print(f"no category matched {args.only}; "
                  f"available: {', '.join(sorted(CATEGORIES))}")
            return 1

    catalog = load_catalog()
    os.makedirs(args.out, exist_ok=True)
    session = requests.Session()
    grand = 0

    for cat, (query, exclude) in sorted(cats.items()):
        df = search(catalog, query, exclude=exclude,
                    min_textures=1, min_texture_res=args.min_texture_res,
                    max_faces=args.max_faces, sort="likes", limit=args.limit)
        cdir = os.path.join(args.out, cat)
        os.makedirs(cdir, exist_ok=True)
        rows = list(df.itertuples())
        if not rows:
            print(f"{cat:<18} 0 matches — widen the query")
            continue

        # One annotation fetch for the whole category; that is where the
        # thumbnail URLs live (the catalog parquet does not carry them).
        uids = [r.uid for r in rows]
        anns = objaverse.load_annotations(uids)

        n_ok = n_fail = 0
        with open(os.path.join(cdir, "index.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["rank", "uid", "name", "likes", "views", "faces",
                        "textures", "texture_res", "license", "file"])
            for i, r in enumerate(rows, 1):
                url = _thumb_url(anns.get(r.uid))
                fname = (f"{i:03d}_{int(r.likes or 0)}L_"
                         f"{int(r.faces or 0)}f_{r.uid}.jpg")
                if url:
                    try:
                        resp = session.get(url, timeout=30)
                        resp.raise_for_status()
                        with open(os.path.join(cdir, fname), "wb") as img:
                            img.write(resp.content)
                        n_ok += 1
                    except Exception:
                        n_fail += 1
                        fname = ""
                else:
                    n_fail += 1
                    fname = ""
                w.writerow([i, r.uid, r.name, int(r.likes or 0),
                            int(r.views or 0), int(r.faces or 0),
                            int(r.textures or 0), int(r.texture_res or 0),
                            r.license, fname])
        grand += n_ok
        print(f"{cat:<18} {n_ok:>4} images"
              + (f"  ({n_fail} had no thumbnail)" if n_fail else ""))

    readme = os.path.join(args.out, "README.txt")
    with open(readme, "w") as fh:
        fh.write(
            "Objaverse candidates, one folder per asset type.\n\n"
            "Filenames are: <rank>_<likes>L_<faces>f_<uid>.jpg\n"
            "  rank   position when sorted by likes (1 = most liked)\n"
            "  likes  Sketchfab likes, the best available popularity proxy\n"
            "  faces  polygon count; city props want this low\n"
            "  uid    the Objaverse id -- this is what gets put in the config\n\n"
            "To choose: delete or note the ones you want, then hand back the\n"
            "filenames (or just the uids). index.csv in each folder has the\n"
            "same data plus name, views and licence.\n\n"
            "Sorted by likes, filtered to textured, non-animated assets under\n"
            f"{args.max_faces:,} faces.\n")

    print(f"\n{grand} images across {len(cats)} categories -> {args.out}")
    print(f"see {readme}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
