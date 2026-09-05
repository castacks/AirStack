#!/usr/bin/env python3
"""250 m suburban earthquake review; see scene_gen/_plans/suburban_earthquake_plan.md."""
import json
import os
import sys
from isaacsim import SimulationApp

app = SimulationApp({"headless": False, "extra_args": [
    "--/rtx/raytracing/fractionalCutoutOpacity=true",
    "--/rtx/pathtracing/fractionalCutoutOpacity=true"]})
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.window.script_editor")

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, REPO+"/scene_gen")
sys.path.insert(0, REPO+"/simulation/isaac-sim/utils")
import snapshots

PARENT = "/World/stage/generated"
OUT = os.environ.get("SNAP_DIR") or "/isaac-sim/.nvidia-omniverse/logs/suburban_earthquake_250"
os.makedirs(OUT, exist_ok=True)


def main():
    review_input = os.environ.get('REVIEW_INPUT')
    if review_input:
        omni.usd.get_context().open_stage(review_input+'/review_scene.usda')
        with open(review_input+'/scene_report.json') as f:
            report = json.load(f)
        review(omni.usd.get_context().get_stage(), report)
        return
    from tools.suburban_quake_build import build
    report = build(OUT, os.environ.get('SCENE_CONFIG') or 'suburb_earthquake_250',
                   cache=os.environ.get('QUAKE_HOUSE_CACHE'))
    omni.usd.get_context().open_stage(OUT+'/review_scene.usda')
    # Static review: do not start whole-neighbourhood physics.
    review(omni.usd.get_context().get_stage(), report)


def review(stage, report):
    houses, people = report['houses'], report['people']
    try:
        snapshots.place_camera(stage,(0,0,430),(0,0,0))
        snapshots.snapshot(OUT+"/overview.png")
        seen = set()
        for rec in houses:
            if rec["mode"] == "pristine" or rec["mode"] in seen:
                continue
            seen.add(rec["mode"])
            snapshots.views_around(stage,{rec["mode"]:(rec["x"],rec["y"])},OUT,
                                   top_h=55,obl_dist=36,obl_h=24)
        for rec in people:
            # Use the SAME damage-facing views that passed the sightline
            # gate, not a generic camera looking through an intact wall.
            for i,eye in enumerate(rec['review_eyes']):
                snapshots.place_camera(stage,eye,rec['review_target'],focal_mm=28.)
                snapshots.snapshot(OUT+'/'+rec['id']+'_view%d.png' % (i+1))
        # Named regression views from the user's review: unsupported floor
        # and foundation failures whose stored tilt used to be ignored.
        for rec in houses:
            if rec['id'] in ('sqh_006','sqh_033','sqh_036'):
                snapshots.views_around(stage,{rec['id']:(rec['x'],rec['y'])},OUT,
                                       top_h=48,obl_dist=32,obl_h=18)
        snapshots.place_camera(stage,(200,-230,220),(0,0,0))
        snapshots.snapshot(OUT+"/review.png")
        print("[suburban_quake] CAPTURES_COMPLETE "+OUT,flush=True)
    except Exception as exc:
        print("[suburban_quake] capture FAILED: "+repr(exc),flush=True)
    while app.is_running():
        app.update()


try:
    main()
finally:
    app.close()
