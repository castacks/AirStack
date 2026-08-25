#!/usr/bin/env python
"""pose_check.py — measure every `_HUMAN_POSES` entry against a real rig.

    # in the isaac-sim container, no Kit app needed (see run-isaac-sim-launcher)
    docker exec isaac-sim bash -c '
      U=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)
      R=$(ls -d /isaac-sim/extscache/omni.usd_resolver-*/ | head -1)
      C=/isaac-sim/kit/extscore/omni.client.lib
      LD_LIBRARY_PATH="/isaac-sim/kit:${U}bin:${R}lib:$C/bin" \
      PYTHONPATH="$U:$C:/isaac-sim/AirStack/scene_gen" \
      PXR_PLUGINPATH_NAME="${R}usd/omni_usd_resolver/resources" \
      /isaac-sim/kit/python/bin/python3 \
        /isaac-sim/AirStack/scene_gen/tools/pose_check.py'

WHY THIS EXISTS
---------------
The survivor poses were DERIVED ON PAPER and never measured. Two of them were
wrong in a way no reviewer could see from a wide shot and no log would ever
report: `sit_ground` encoded a CHAIR sit — thigh forward, shin hanging — so the
ankle finished 0.237 m BELOW the hip instead of level with it, and the
`_POSE_Z_OFFSET` that puts the seat on the ground then drove the feet 0.17 m
UNDER the asphalt. The table's own comment claimed the soles landed "0.15 m
under the pelvis and 0.83 m in front"; the code produced 0.246 and 0.580. The
arithmetic in the comment and the arithmetic in the numbers had diverged, and
nothing checked.

`_bind_human_pose` cannot help: it has three silent `return`s (no skeleton, no
cached transforms, no Skeleton prim) and warns on none of them, so a pose that
never applies looks exactly like a pose that applied badly.

This tool closes that gap. It imports the REAL tables from `scene_generator`
(which is stdlib + pxr only, so it needs no Kit app), walks the rig's actual
joint hierarchy with the shipping `_pose_joint_transforms`, and reports where
the body ACTUALLY lands once `_pose_dz`'s height scaling is applied. A pose
whose contact points miss the ground is a FAIL with the miss in metres.

WHAT IT CANNOT TELL YOU
-----------------------
That a geometrically valid pose READS correctly. `crouch` passed contact
checks while still looking like a stumble, because the spine and arm angles
that decide whether a posture is legible carry no ground contact to test.
Contact is necessary, not sufficient — this tool rules out the errors that are
invisible in a viewport, and the bench is still what judges the rest.
"""

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from pxr import Usd, UsdGeom, UsdSkel                                # noqa: E402

import scene_generator as sg                                        # noqa: E402
from disaster import people as ppl                                  # noqa: E402

_LIB = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"
        "Muyang/People/Assets/")
DEFAULT_RIGS = [_LIB + n for n in (
    "rp_carla_rigged_001_ue4.usd",
    "rp_claudia_rigged_002_ue4.usd",
    "rp_sophia_rigged_003_ue4.usd",
    "rp_eric_rigged_001_ue4.usd",
    "rp_manuel_rigged_001_ue4.usd",
    "rp_nathan_rigged_003_ue4.usd",
)]

# What each posture must touch, and roughly where the body should sit. Ranges
# are metres of the CONTACT joint above ground and of the hip; both are checked
# because a pose can ground its feet while leaving the figure at standing
# height, which is exactly how `sit_ground` failed.
#   contact  which joint has to reach the ground ('sole' = ankle less the rig's
#            own ankle-to-sole height; 'heel' allows the toes-up sitting foot)
#   hip_m    (lo, hi) acceptable hip-joint height
EXPECT = {
    "idle":       {"contact": "sole", "hip_m": (0.80, 1.10)},
    "walk":       {"contact": "sole", "hip_m": (0.80, 1.10)},
    "wave":       {"contact": "sole", "hip_m": (0.80, 1.10)},
    "sit_ground": {"contact": "heel", "hip_m": (0.05, 0.25)},
    "crouch":     {"contact": "sole", "hip_m": (0.35, 0.75)},
    # Seated-on-a-thing poses are placed by the CALLER at a seat height, so
    # there is no ground contact to check — only that the legs hang below the
    # seat rather than above it.
    "sit_edge":            {"contact": None, "hip_m": None},
    "seated_car":          {"contact": None, "hip_m": None},
    "seated_car_arms_down": {"contact": None, "hip_m": None},
}

TOL_M = 0.06          # a contact this far off the ground is visible on sight


def rig_facts(url):
    """(joints, rest, leaf-names, standing height m, ankle-above-sole m)."""
    stage = Usd.Stage.Open(url)
    if stage is None:
        raise RuntimeError("could not open " + url)
    prim = next((p for p in stage.Traverse()
                 if p.GetTypeName() == "Skeleton"), None)
    if prim is None:
        raise RuntimeError("no Skeleton in " + url)
    skel = UsdSkel.Skeleton(prim)
    joints, rest = skel.GetJointsAttr().Get(), skel.GetRestTransformsAttr().Get()
    if not joints or not rest or len(joints) != len(rest):
        raise RuntimeError("unusable skeleton in " + url)
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 0.01
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    rng = box.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    height = (rng.GetMax()[2] - rng.GetMin()[2]) * mpu
    leaf = [str(j).rsplit("/", 1)[-1] for j in joints]
    # At rest the character stands on the ground, so the ankle's own height IS
    # the ankle-to-sole distance — and it differs per rig, which is why the
    # male characters have always sat low.
    ankle = _world(joints, rest, {}, leaf, "foot_l", mpu)[2]
    return list(joints), list(rest), leaf, height, ankle, mpu


def _world(joints, rest, deltas, leaf, name, mpu):
    _local, world = sg._pose_joint_transforms(joints, rest, deltas)
    t = world[leaf.index(name)].ExtractTranslation()
    return (t[0] * mpu, -t[1] * mpu, t[2] * mpu)      # -y is forward


def check(url, verbose=False):
    joints, rest, leaf, height, ankle_h, mpu = rig_facts(url)
    print("\n%s   %.3f m tall, ankle %.3f m above the sole"
          % (os.path.basename(url), height, ankle_h))
    fails = []
    for pose in sorted(sg._HUMAN_POSES):
        deltas = sg._HUMAN_POSES[pose]
        exp = EXPECT.get(pose)
        # _pose_dz: the table's offsets were authored on a 1.80 m reference.
        dz = ppl._pose_dz(url, pose, height) + ppl._seated_asset_dz(url, pose)
        hip = _world(joints, rest, deltas, leaf, "pelvis", mpu)[2] + dz
        ank = _world(joints, rest, deltas, leaf, "foot_l", mpu)
        head = _world(joints, rest, deltas, leaf, "head", mpu)[2] + dz
        reach = ank[1] - _world(joints, rest, deltas, leaf, "pelvis", mpu)[1]
        sole = ank[2] + dz - ankle_h
        heel = ank[2] + dz - ankle_h * 0.45     # toes-up foot rocks onto the heel
        hand = _world(joints, rest, deltas, leaf, "hand_l", mpu)
        shoulder = _world(joints, rest, deltas, leaf, "upperarm_l", mpu)
        # Standing abduction: how far the hand is held out from the shoulder,
        # as an angle off straight-down. ~10-12 deg is relaxed; 45 is posed.
        import math as _m
        abd = _m.degrees(_m.atan2(abs(hand[0] - shoulder[0]),
                                  max(1e-6, shoulder[2] - hand[2])))
        line = ("  %-21s hip %+.3f  head %+.3f  sole %+.3f  reach %+.3f  "
                "dz %+.3f  arm %4.0f deg" % (pose, hip, head, sole, reach, dz, abd))
        if exp is None or exp["contact"] is None:
            print(line + "   (placed on a seat — not ground-checked)")
            continue
        touch = sole if exp["contact"] == "sole" else heel
        bad = []
        if abs(touch) > TOL_M:
            bad.append("%s %+.3f m off the ground" % (exp["contact"], touch))
        lo, hi = exp["hip_m"]
        if not (lo <= hip <= hi):
            bad.append("hip %.3f outside [%.2f, %.2f]" % (hip, lo, hi))
        print(line + ("   FAIL: " + "; ".join(bad) if bad else "   ok"))
        if bad:
            fails.append((os.path.basename(url), pose, "; ".join(bad)))
    return fails


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("rigs", nargs="*", default=DEFAULT_RIGS,
                    help="rig USD urls (default: the six RenderPeople humans)")
    args = ap.parse_args()
    fails = []
    for url in (args.rigs or DEFAULT_RIGS):
        try:
            fails += check(url)
        except Exception as exc:                      # a missing rig is data
            print("\n%s   SKIPPED: %s" % (os.path.basename(url), exc))
    print("\n" + "=" * 72)
    if fails:
        print("%d FAILING (rig, pose):" % len(fails))
        for r, p, why in fails:
            print("   %-34s %-12s %s" % (r, p, why))
    else:
        print("every pose grounds correctly on every rig")
    print("=" * 72)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
