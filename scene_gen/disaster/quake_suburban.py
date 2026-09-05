"""Wood-frame suburban earthquake review; no wind/fire damage routing.

The empirical support for the mechanisms, and the deliberately synthetic
sampling parameters, are in _plans/suburban_earthquake_plan.md. USD imports
are local so the damage plan can be checked without starting Isaac.
"""
import math
import random
import zlib

MODES = ("pristine", "racked", "foundation", "partial_collapse",
         "soft_storey", "collapse")
GRADE = dict(zip(MODES, ("DG0", "DG2", "DG3", "DG3", "DG4", "DG5")))


def local_xy(x, y, house):
    a = math.radians(-float(house.get("yaw", 0)))
    dx, dy = x - house["x"], y - house["y"]
    return math.cos(a)*dx-math.sin(a)*dy, math.sin(a)*dx+math.cos(a)*dy


def soft_weight(x, y, soil):
    if not soil:
        return 0.0
    cx, cy = soil["center"]
    a = math.radians(-float(soil.get("angle_deg", 0)))
    dx, dy = x-cx, y-cy
    u = (math.cos(a)*dx-math.sin(a)*dy)/float(soil["rx_m"])
    v = (math.sin(a)*dx+math.cos(a)*dy)/float(soil["ry_m"])
    return max(0.0, 1.0-u*u-v*v)


def plan_houses(houses, config, field, seed=11):
    from detail import modular_house as mh
    cfg = config.get("earthquake_suburban") or {}
    dis = config.get("disaster") or {}
    soil = dis.get("soft_soil")
    result = []
    for i, h in enumerate(houses):
        # Independent per-house stream: changing layout ordering cannot
        # reroll damage to every following dwelling.
        key = "{}:{:.3f}:{:.3f}".format(h["style"], h["x"], h["y"])
        rng = random.Random(int(seed) + zlib.crc32(key.encode()))
        spec = mh.STYLES[h["style"]]
        cells = mh.footprint(spec)
        w = (max(x for x, _ in cells)-min(x for x, _ in cells)+1)*mh.CELL_M
        d = (max(y for _, y in cells)-min(y for _, y in cells)+1)*mh.CELL_M
        demand = (float(field(h["x"], h["y"])) * float(dis.get("grade_scale", 1))
                  * float(cfg.get("demand_scale", 1)))
        score = demand*rng.uniform(0.02, 1.20)
        mode = ("pristine" if score < .18 else "racked" if score < .48
                else "foundation" if score < .68 else "partial_collapse"
                if score < .96 else "soft_storey" if score < 1.10 else "collapse")
        if mode == "soft_storey" and not (spec["storeys"] > 1 and spec.get("garage")):
            mode = "partial_collapse"
        sw = soft_weight(h["x"], h["y"], soil)
        settlement = bool(demand > 0 and sw and
                          rng.random() < min(1., demand)*sw*float(soil.get("rate", .65)))
        if settlement and mode not in ("collapse", "soft_storey"):
            mode = "foundation"
        # Keep the original per-house draw stream so the same seed remains
        # comparable across the visual revision. No added attachments.
        mild_damage = rng.random() < float(cfg.get("mild_damage_share", .65))
        if mode == "racked" and not mild_damage:
            mode = "pristine"
        result.append(dict(h, id="sqh_%03d" % i, mode=mode, grade=GRADE[mode],
                           W=w, D=d, H=spec["storeys"]*mh.STOREY_M+3,
                           storeys=spec["storeys"], intensity=demand,
                           soil_weight=sw, settlement=settlement,
                           side=rng.choice("SENW"),
                           yaw_deg=h["yaw"], type="wood_frame",
                           failure_sides=[], seed=rng.randrange(2**30)))
    return result


def _world(lx, ly, rec):
    a = math.radians(rec["yaw"])
    return rec["x"]+math.cos(a)*lx-math.sin(a)*ly, rec["y"]+math.sin(a)*lx+math.cos(a)*ly


def _shift(stage, path, dx=0., dy=0., dz=0.):
    from pxr import Gf, UsdGeom
    prim = stage.GetPrimAtPath(path)
    xf = UsdGeom.Xformable(prim)
    m = xf.GetLocalTransformation()
    # All module holders live below an identity parent.
    t = m.ExtractTranslation()
    m.SetTranslateOnly(t+Gf.Vec3d(dx, dy, dz))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)


def author_house(stage, rec, parent, ssf=1.0):
    """Static, authored timber-collapse recipe; suitable for per-house baking."""
    from .quake_suburban_bake import author_house as build
    return build(stage, rec, parent, ssf)


def rubble_body_support(stage, person, house, records, parent, ssf=1.):
    """Require actual support under soles, torso and head, not one shard.

    The urban ground helper assumes a different ground-root location and can
    fall back to z=0. Here we query the suburban ground explicitly and never
    accept that fallback as measured support.
    """
    from . import quake_people as qp
    samples = qp._prone_support_points(person['x'], person['y'],
                                        person['pose'], person['yaw_deg'])
    roots = [house, {'prim': parent+'/ground'}, {'prim': parent+'/quake_ground'}]
    heights = []
    for x,y in samples:
        if any(qp.point_in_footprint(x,y,r,margin=.1) for r in records):
            return None
        values = [qp._support_z(stage,r,x,y,float(person['z'])+.4,ssf,
                               body_footprint=False) for r in roots]
        values = [v for v in values if v is not None]
        if not values:
            return None
        heights.append(max(values))
    if max(heights)-min(heights) > .18:
        return None
    return max(heights), len(samples)


def author_people(stage, records, parent, blockers=(), total=12, seed=11, ssf=1., preferred=()):
    """Reuse earthquake support/sightlines and the established RP pose/cover.

    People are a visible rescue-target sample; status is deliberately unknown.
    The shared urban module's independent fatality coin is not retained.
    """
    import scene_generator as sg
    from . import quake_people as qp, fire_people as fp, planks, tornado_people as tp
    qp._GEOMETRY_CACHE.clear()
    rng = random.Random(seed+991)
    made, covers = [], []
    import copy
    by_prim={r['prim']:r for r in records}
    for original in preferred:
        person=copy.deepcopy(original)
        r=by_prim.get(person.get('building_prim'))
        if r is None or person['state']!='rubble_casualty':
            continue
        support=rubble_body_support(stage,person,r,records,parent,ssf)
        if support is None:
            continue
        z,n=support
        dz=z-person['z']
        person.update(z=round(z,3),support_samples=n,retained_from_previous=True)
        person['review_target'][2]+=dz
        for eye in person['review_eyes']:
            eye[2]+=dz
        if not all(qp._clear_segment(stage,r,person['review_target'],eye,ssf) for eye in person['review_eyes']):
            continue
        ux,uy=tp._body_axis(person['pose'],person['yaw_deg'],fp.LYING_ROLL[person['pose']])
        if any(math.hypot(person['x']+ux*t-bx,person['y']+uy*t-by)<rad+.4
               for t in (0,.9,1.8) for bx,by,rad in blockers):
            continue
        person['previous_id']=person['id']
        made.append(person)
        specs=qp._cover_specs(person,rng)
        for spec in specs:
            spec['class']='sheathing' if rng.random()<.65 else 'stud'
        covers.extend(specs)
        if len(made)>=total:
            break
    eligible = [r for r in records if r["failure_sides"]]
    rng.shuffle(eligible)
    for r in eligible:
        for attempt in range(8):
            if len(made) >= total:
                break
            person = (qp._interior_person(stage, r, records, rng, made,
                                          (-125,-125,125,125), ssf)
                      if attempt == 0 and r["storeys"] > 1 and r["mode"] == "partial_collapse"
                      else qp._rubble_person(stage, r, records, rng, made,
                                             (-125,-125,125,125), ssf))
            if not person:
                continue
            if person['state'] == 'rubble_casualty':
                support = rubble_body_support(stage,person,r,records,parent,ssf)
                if support is None:
                    continue
                z,n = support
                dz = z-person['z']
                person.update(z=round(z,3), support_samples=n,
                              full_body_support_verified=True,
                              z_mode='measured_full_body_surface')
                person['review_target'][2] += dz
                for eye in person['review_eyes']:
                    eye[2] += dz
                if not all(qp._clear_segment(stage,r,person['review_target'],eye,ssf)
                           for eye in person['review_eyes']):
                    continue
            ux, uy = tp._body_axis(person["pose"], person["yaw_deg"], fp.LYING_ROLL[person["pose"]])
            stations = [(person["x"]+ux*t, person["y"]+uy*t) for t in (0, .9, 1.8)]
            if any(math.hypot(x-bx,y-by) < rad+.4 for x,y in stations for bx,by,rad in blockers):
                continue
            person.update(id="sqp_%03d" % len(made), status="unknown", alive=None,
                          disaster="earthquake", sampling="visible_rescue_target")
            if person["state"] == "rubble_casualty":
                specs = qp._cover_specs(person, rng)
                for spec in specs:
                    spec["class"] = "sheathing" if rng.random() < .65 else "stud"
                covers.extend(specs)
            made.append(person)
            break
    # Reassign the contiguous scene IDs after retained + replacement records
    # are combined. previous_id preserves identity across the review revision.
    for i,person in enumerate(made):
        person['id']='sqp_%03d'%i
    placements, skipped = fp.to_placements(made, tag_ids=True)
    if skipped:
        raise RuntimeError("casualty conversion failed: "+repr(skipped))
    sg.apply_placements(stage, placements, parent+"/people", ssf, instance_categories=set())
    for rec, p in zip(made, placements):
        rec["prim"] = p["prim_path"]
    planks.build(stage, parent+"/people_cover", covers, planks.materials(stage,parent), ssf)
    return made
