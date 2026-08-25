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
    <out>/index.html                                    one grid page, all categories

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
# Fantasy / antiquity look that swamps any "ruins" or "destroyed" query.
HIST = ("medieval|fantasy|castle|temple|ancient|roman|greek|gothic|dungeon|elven|"
        "egypt|aztec|maya|mayan|inca|viking|church|cathedral|abbey|monastery|"
        "shrine|tomb|pyramid|sci-fi|scifi|space|alien|steampunk|halloween|spooky|"
        "haunted|zombie")

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
    "tennis_court":     ("tennis",
                         "racket|racquet|ball$|player|character|shoe|arena|"
                         "stadium|table tennis|ping pong"),
    "volleyball_court": ("volleyball",
                         "ball$|player|character|shoe|arena|stadium"),
    "baseball_field":   ("baseball",
                         "ball$|bat$|glove|helmet|cap|player|character|shoe|"
                         "stadium|arena|card"),
    "soccer_goal":      ("soccer|football goal|goal post|goal net",
                         "ball$|player|character|shoe|jersey|stadium|arena|"
                         "american football|helmet"),
    # NOT IN OBJAVERSE either: 0 matches for every phrasing tried. A track is
    # a painted surface, so it is better generated than sourced.
    "running_track":    ("running track|athletics track", "race car|racetrack"),
    # NOT IN OBJAVERSE. "bleacher" -> 1 match, "grandstand|stadium seating|
    # spectator stand" -> 0. Kept so the gap is recorded rather than retried.
    "bleachers":        ("bleacher|grandstand|spectator seating", ""),
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
    # The ORNAMENTAL kind — a basin or tiered centrepiece, which is a different
    # object from the drinking fountain above and gets its own folder rather
    # than muddying that one. "fountain" alone returns 387, so the excludes are
    # doing most of the work: bare "fountain" is full of drinking fountains,
    # soda fountains, taps and pen nibs (fountain pen).
    "fountain_ornamental": ("fountain",
                            "drinking|bubbler|soda|water cooler|pen|nib|"
                            "faucet|tap$|sink|particle|vfx|fx|sprite"),
    "skate_park":       ("skate park|skate ramp|half pipe|quarter pipe|"
                         "skateboard ramp", "character|skateboard only"),
    "scoreboard":       ("scoreboard|score board", "arena jumbotron"),
    "park_sign":        ("park sign|trail sign|wooden sign post|park entrance",
                         "neon|medieval|tavern|shop sign"),
    "dog_park":         ("dog park|agility equipment|dog agility", "character"),
    "tennis_practice":  ("practice wall|handball court|pickleball court",
                         "arena|stadium"),

    # ---- disaster / damage ---------------------------------------------------
    # Art for the *aftermath* — rubble, burnt and collapsed structures, wrecks.
    # Bare "burnt", "ash", "flood", "ruins", "battlefield" are traps: they pull
    # in food, Pokémon Ash, Halo's Flood enemies, medieval castles and FPS maps
    # respectively, so the excludes below are doing real work. HIST strips the
    # fantasy/antiquity look that dominates "ruins" and "destroyed".
    "rubble_pile":      ("rubble|rubble pile|pile of rubble|debris pile|debris heap|"
                         "pile of debris|heap of .*debris",
                         "barney|flintstones|figurine|pebble|cobble|beach|spaceship|"
                         "books|figure|laundry|labyrinth|megaman|marine|sack|container"),
    "brick_pile":       ("brick pile|bricks pile|pile of bricks|brick rubble|"
                         "bricks rubble|brick heap|heap of .*bricks|broken brick|"
                         "brick debris|bricks debris", ""),
    "concrete_debris":  ("concrete chunk|concrete debris|concrete rubble|"
                         "broken concrete|concrete slab|concrete pile|concrete block|"
                         "cinder ?block|rebar|concrete piece|concrete fragment",
                         "sci-fi|scifi|crossbow|weapon|stairs|forge random|fence"),
    "wood_debris":      ("wood debris|wooden debris|plank debris|planks debris|"
                         "broken plank|broken planks|broken wood|broken board|"
                         "broken boards|plank pile|wood pile|pile of wood|scrap wood|"
                         "broken beam|wooden beam|timber pile|lumber pile",
                         "firewood|fireplace|campfire|bbq|grill|medieval|fantasy|"
                         "tmnt|turtle|ninja|goggles|mercedes|skate|platformer|"
                         "platform game|toon|stylized"),
    "burnt_wood":       ("burnt wood|burned wood|charred|charcoal|burnt log|"
                         "burned log|burnt plank|burned plank|burnt timber|"
                         "burned timber|burnt beam|scorched wood|burn pile|"
                         "burnt stump|burned stump|burnt trunk|burned trunk",
                         "bbq|grill|croissant|food|pencil|drawing|sketch|steak|meat|"
                         "lamp|drawers|coaster|coasters|chair|wall unit|velvet|brass|"
                         "furniture|iron$|ironing|dead frontier|hellhound|wraith|boss|"
                         "spider|titan|figure|figurine|statue|gameboy|nintendo|"
                         "minecraft|generator|kiln|kilns|paving|hardscape|slane|"
                         "tamales|corn"),
    "burnt_tree":       ("burnt tree|burned tree|charred tree|scorched tree|"
                         "burnt forest|burned forest|wildfire|forest fire|bushfire|"
                         "burnt palm|burned palm|burnt pine|burned pine|dead tree|"
                         "dead trees",
                         "halloween|spooky|cartoon|christmas|stylized|haunted|"
                         "fire truck|firefighter|freefire|free fire|csgo|"
                         "counter-strike|p2000|bolt|water gun|firefight|skull|toy|"
                         "kid|citiesskylines|cities skylines|netflix"),
    # NEARLY EMPTY in Objaverse: one 1M-face "Fire Damaged Roof Section" scan
    # and an untextured "Burnt Building". Burnt structures have to be built
    # from burnt_wood / damaged_wall pieces, or textured in-house.
    "burnt_building":   (r"(?=.*\b(?:burnt|burned|charred|scorched|fire damaged|"
                         r"fire damage|house fire|after fire)\b)"
                         r".*\b(?:house|building|wall|roof|beam|timber|cabin|barn|"
                         r"shed|structure|ruins?|home|apartment|facade)\b",
                         "teeth|forensic|monster|car|cars|vehicle|truck|van|bus|"
                         "suv|sedan|police|bulb|knife|tree|fireplace"),
    "burnt_vehicle":    ("burnt car|burned car|burnt out car|burned out car|"
                         "burned-out car|burnt vehicle|burned vehicle|car wreck|"
                         "wrecked car|wrecked truck|wrecked bus|wrecked van|"
                         "destroyed car|destroyed truck|destroyed vehicle|"
                         "destroyed bus|crashed car|car crash|abandoned car|"
                         "abandoned truck|abandoned bus|derelict car|shelled car|"
                         "burned police",
                         "sci-fi|scifi|toy|hotwheels|hot wheels|diecast|lego|train|"
                         "rail|railway|railroad"),
    "destroyed_building": ("destroyed building|destroyed house|destroyed home|"
                         "ruined building|ruined house|collapsed building|"
                         "collapsed house|demolished building|demolished house|"
                         "demolished|damaged building|damaged house|building ruins|"
                         "house ruins|bombed building|bombed house|"
                         "war torn building|abandoned house|abandoned building|"
                         "derelict house|derelict building|dilapidated|"
                         "destroyed apartment|ruined apartment|damaged apartment|"
                         "destroyed shop|destroyed store|destroyed town|"
                         "destroyed city|destroyed village|ruined city|ruined town|"
                         "ruined village|earthquake building|earthquake housing",
                         HIST + "|bathroom|kitchen|interior|cobweb|asset pack|"
                         "assets|clutter|train|rail|locomotive|tile|tileable|"
                         "tilable|material|cyberpunk|treehouse|window$|bridge"),
    "damaged_wall":     ("broken wall|damaged wall|cracked wall|collapsed wall|"
                         "destroyed wall|ruined wall|wall ruins|wall debris|"
                         "wall rubble|broken pillar|damaged pillar|broken column|"
                         "wall piece|wall chunk|wall fragment|hole in wall|"
                         "blown wall|wall with hole|shelled wall|shelled balcony|"
                         "shelled garage",
                         HIST + "|painted wall fragment|vase|ceramic|themet|"
                         "dwarven|decorated|prehistory"),
    # 3 hits total ("Roof Rubble Pile", "Damaged roof section", an untextured
    # "House with broken roof"). Kept so the gap is recorded.
    "collapsed_roof":   ("collapsed roof|damaged roof|broken roof|roof debris|"
                         "roof rubble|roof tiles broken|broken roof tiles|"
                         "destroyed roof|roof collapse|fallen roof", HIST),
    # The real find here is a cluster of Ukraine photogrammetry scans by one
    # author ("Shelled wall 01-06", "Shelled car", "Burned car 01/03") — heavy
    # (100-350k faces, one texture) but exactly the look. Everything else
    # matching "war" is Call of Duty / Battlefield fan art.
    "war_damage":       ("war torn|wartorn|war-torn|bombed|shelled|bomb crater|"
                         "shell crater|artillery crater|bomb damage|blast damage|"
                         "explosion damage|war damage|war damaged|shell damage",
                         "moon|lunar|mars|meteor|asteroid|volcano|sci-fi|scifi|"
                         "space|planet|character|soldier|weapon|gun|rifle|smg|"
                         "helmet|armor|armour|mech|warhammer|40k|spaceship|ship|"
                         "battlefield|warzone|halo|call of duty|pubg|gameboy|"
                         "godzilla|kong|punisher|costume|jaeger|fighter|jet|"
                         "geology|impact"),
    "modern_ruins":     ("ruins|ruined|ruin|wreckage",
                         HIST + "|tree|forest|rock|mountain|statue|column|pillar|"
                         "arch$|shipwreck|ship|boat|plane|aircraft|character|"
                         "terrain|landscape|receiver|radio|speaker|arcade|armchair|"
                         "sofa|chair|undertale|undyne|steps|platform|mosaic|"
                         "vending|tank|car|cars|sedan|coupe|van|wagon|vehicle|"
                         "wreck$|bunker|mine"),
    "post_apocalyptic": ("post-apocalyptic|post apocalyptic|postapocalyptic|"
                         "apocalypse|apocalyptic|wasteland",
                         "character|zombie|weapon|gun|rifle|pistol|shotgun|knife|"
                         "sword|axe|mask|helmet|suit|armor|armour|sci-fi|scifi|"
                         "creature|monster|mutant|robot|mech|pip-boy|pipboy|drone|"
                         "spaceship|outfit|clothing|jacket|backpack|bag|bottle|"
                         "can$|food|psylocke|xmen|x-men|marvel|comics|sexy|pose|"
                         "beauty|girl|woman|fantasy town|toon|cartoon|welding|"
                         "arcade|receiver|radio|speaker|gundam|car|cars|vehicle|"
                         "sedan|truck|van|bus|wagon|train|subway|metro|coupe"),
    "storm_flood_damage": ("flooded|flood damage|flood damaged|floodwater|"
                         "flood water|water damage|water damaged|tornado|"
                         "hurricane damage|storm damage|storm damaged|disaster|"
                         "tsunami debris|high water",
                         "halo|mars|nasa|planet|geology|eurofighter|jet|aircraft|"
                         "kamaz|ural|truck|bottle|can$|drink|totem|inflatable|"
                         "inflable|drone|yuneec|tablet|cuneiform|medieval|galway|"
                         "mask|fan|ventilation|satellite|weather|character|"
                         "freefire|dorothy|movie|beach sign|signboard|typhoon"),
    "earthquake_ground": ("earthquake|landslide|sinkhole|mudslide|cracked ground|"
                         "cracked road|cracked earth|cracked asphalt|pothole|"
                         "potholes|broken road|damaged road|road damage|"
                         "broken asphalt|broken pavement|broken sidewalk|"
                         "damaged pavement|damaged concrete|ground crack|"
                         "train derailment",
                         "sci-fi|scifi|space|lava|magma|character|mars|nasa|"
                         "hirise|geology|geolog|fault|seismite|carbonate|"
                         "limestone|canyon|island|science|fossil|shell$|"
                         "splitpoint|dem$|relief|library"),
    "fallen_tree":      ("fallen tree|fallen log|fallen trunk|uprooted|"
                         "broken tree|snapped tree|fallen branch|tornado tree|"
                         "storm damaged tree|snapped trunk|broken trunk|"
                         "windthrow|lying dead tree|lying trunk",
                         "halloween|spooky|cartoon|christmas|stylized|haunted|"
                         "mushroom|carved|stump"),
    "debris_general":   ("debris|wreckage|scrap metal|scrap pile|scrap heap|"
                         "junk pile|trash pile|garbage pile|junkyard|scrapyard|"
                         "scrap yard|pile of junk|pile of trash|pile of garbage",
                         "lab|books|figure|laundry|spaceship|space|medieval|"
                         "fantasy|dungeon|megaman|marine|labyrinth|psi|oil barrel|"
                         "ship|spacecraft|cars craft|fortnite|csgo|crate|"
                         "stone floor|mossy|boulder|quarry|anthropocene|stein|tv$|"
                         "character|mech|mecha|robot|bot$|gunner|undercarriage|"
                         "beachcombing|bike|bicycle|drone|drones|course|fallout|"
                         "coupe|k car|reliant|impala"),
    "dirt_pile":        ("dirt pile|dirt mound|mud pile|earth pile|soil pile|"
                         "gravel pile|sand pile|pile of dirt|pile of sand|"
                         "pile of gravel|mound of dirt|dirt heap|sand heap|"
                         "gravel heap|rock pile|pile of rocks|stone pile|"
                         "pile of stones|boulder pile|pile of earth",
                         "sci-fi|scifi|medieval|fantasy|cartoon|stylized|character|"
                         "cake|grave|yosemite|sculptural"),
    "emergency_barriers": ("sandbag|sandbags|jersey barrier|concrete barrier|"
                         "road block|roadblock|barricade|caution tape|police tape|"
                         "barrier tape|emergency tent|relief tent|rescue tent|"
                         "medical tent|triage|emergency shelter|field hospital|"
                         "disaster relief|hesco",
                         "medieval|fantasy|castle|sci-fi|scifi|spike|cheval|"
                         "hedgehog|czech|tank trap|character|soldier|cone|cones|"
                         "traffic sign|signs|sign package|transit|guard rail|"
                         "guardrail|metal fence|wood fence|wooden fence|"
                         "simple fence"),
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

    _write_sheet(args.out)

    print(f"\n{grand} images across {len(cats)} categories -> {args.out}")
    print(f"see {readme} and index.html")
    return 0


def _write_sheet(out):
    """One browsable page: a thumbnail grid per category, uid under every
    image so a pick can be copied straight off the page. Built from the
    index.csv in every category folder under *out*, so a ``--only`` rerun
    refreshes one folder without dropping the others from the page."""
    from html import escape as e
    sheet = {}
    for cat in sorted(os.listdir(out)):
        idx = os.path.join(out, cat, "index.csv")
        if not os.path.isfile(idx):
            continue
        with open(idx, newline="") as fh:
            for row in csv.DictReader(fh):
                if row["file"]:
                    sheet.setdefault(cat, []).append(
                        (row["file"], row["uid"], row["name"], int(row["likes"]),
                         int(row["faces"]), int(row["textures"]),
                         int(row["texture_res"]), row["license"]))
    cats = sorted(sheet)
    parts = [
        "<!doctype html><meta charset=utf-8><title>Objaverse candidates</title>",
        "<style>body{font:14px system-ui;margin:16px;background:#111;color:#ddd}"
        "nav a{margin-right:12px}h2{margin-top:40px}"
        ".g{display:grid;grid-template-columns:repeat(auto-fill,minmax(240px,1fr));gap:10px}"
        ".c{background:#1c1c1c;padding:6px;border-radius:6px}"
        ".c img{width:100%;aspect-ratio:4/3;object-fit:cover;border-radius:4px}"
        ".n{font-weight:600;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}"
        ".m{color:#999;font-size:12px}code{font-size:11px;color:#8bc;user-select:all}"
        "</style>",
        "<nav>" + " ".join(
            f'<a href="#{e(c)}">{e(c)} ({len(sheet[c])})</a>' for c in cats)
        + "</nav>",
    ]
    for c in cats:
        parts.append(f'<h2 id="{e(c)}">{e(c)} <small>{len(sheet[c])}</small></h2><div class=g>')
        for fname, uid, name, likes, faces, tex, res, lic in sheet[c]:
            src = f"{c}/{fname}"
            parts.append(
                f'<div class=c><a href="{e(src)}"><img loading=lazy src="{e(src)}"></a>'
                f'<div class=n title="{e(name)}">{e(name)}</div>'
                f'<div class=m>{likes} likes &middot; {faces:,} faces &middot; '
                f'{tex} tex @ {res}px &middot; {e(str(lic))}</div>'
                f'<code>{e(uid)}</code></div>')
        parts.append("</div>")
    with open(os.path.join(out, "index.html"), "w") as fh:
        fh.write("\n".join(parts))


if __name__ == "__main__":
    sys.exit(main())
