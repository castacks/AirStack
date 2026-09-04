#!/usr/bin/env python3
"""fire_corridor.py — a WIND-DRIVEN CORRIDOR of fire, contiguous by
construction.

THE DEFECT THIS REPLACES
------------------------
The shipped selection lit N independent ignition seeds and let
`urban_fire_spread` grow each one. Two things went wrong with that on a 1 km
plate, and both are visible in a review sheet:

1. **Scattered spots, not a fire.** Six ignitions each reaching three or four
   buildings reads as six unrelated incidents, not one event. Raising the
   count raises the spot count, not the coverage.

2. **Holes inside the burn.** The solver can only touch an asset that has a
   bake, so it routes around every unbaked building and leaves it pristine in
   the middle of a burnt block. MEASURED on the L3 manifest: of the intact
   buildings within 60 m of a burning one, **27 were burnable and 27 had no
   bake at all** — half the checkerboard was a bake gap, not fire behaviour.

That second point is also, in hindsight, why the fire presets carried
burnable-only pools: cutting `lowrise` 26 -> 7 assets guaranteed contiguity by
making everything fuel. It bought that at the cost of six identical department
stores in a row, and it was never written down as the trade it was.

THE MODEL
---------
One fire, one origin, one front, running downwind. Everything else follows.

A building is in the burn if it lies inside the corridor: within
``half_width_m`` of the wind axis (fanning out by ``spread_deg``, as a real
fire does) and between the origin and the front. The corridor's LENGTH is
what the level ladder rides — a fire that has been running six hours has
covered more ground than one running ninety minutes — so the same
``target_frac`` knob sets both how much of the city is involved and,
through the front speed, how far along each building is in its own fire.

Level comes from the compartment-fire clock, not from a post-hoc count
shuffle. A building at along-distance ``s`` ignites when the front reaches
it, ``t_ignite = s / v``; at scene time ``T`` it has been burning
``tau = T - t_ignite``; its level is the highest whose `soot_plume.
DURATION_S` threshold ``tau`` has passed. So the burnt-out tail sits at the
origin and the fresh head sits at the front, with a real gradient between —
which is what an urban conflagration looks like from the air, and what the
old "same 5.6 h epoch at every level" ladder could never produce.

EVERY BUILDING IN THE CORRIDOR BURNS. That is the contiguity guarantee, and
it makes the bake list a geometric consequence rather than a guess: bake what
is in the corridor and there are no holes. `unbakeable()` reports the handful
that genuinely cannot be baked at any cost (a pack marked unburnable, a
blacklisted name, anything over the fire height cap) so the corridor
placement search can steer around them.

WHAT THIS DOES NOT DO
---------------------
It does not model firebreaks, wind shifts, or spotting across a gap. The
corridor is a swathe, and a wide arterial crossing it does not stop it. That
is a deliberate simplification: the shipped fire needs to look right from
150 m up and be reproducible offline, and a front that respects the street
grid is a much bigger model than this. `spread_deg` and the placement search
are the only nods to real geometry.
"""
import math
import os

DEFAULT_SPREAD_DEG = 5.0
DEFAULT_HALF_WIDTH_M = 110.0

#: Reasons `urban_fire_city.burnable` gives that a BAKE would fix. Anything
#: else is a permanent refusal and the corridor should avoid it.
_BAKEABLE_HINTS = ("no bake", "fire_b", "bake")

#: Checked BEFORE `_BAKEABLE_HINTS`, not after: `bake_kind()`'s own
#: `('slice', None)` refusal message for an unregistered pack
#: (`standalone/buildings/...` and friends) reads "... fire_bake.KINDS=(...)
#: has no bake kind for it ... must be REFUSED, not silently dropped" —
#: its own SUBSTRING "no bake" false-positive-matched `_BAKEABLE_HINTS`
#: above, so `unbakeable()` returned `False` for a refusal that names
#: itself, in its own text, as a permanent one. Measured 2026-09-02: this
#: put every `standalone/buildings/intact/*` and unregistered
#: `Library/Stages/.../FactoryDistrict` placement into the corridor as
#: "needs_bake" with `kind: null` — 44 of 61 records on one L1 run — instead
#: of excluding them via `permanent_bad` before the corridor placement
#: search ever ran, silently shipping most of a "burning" corridor
#: undamaged. Exact-vocabulary override, not another substring guess: this
#: matches `bake_kind()`'s own words for its own permanent case.
_PERMANENT_HINTS = ("must be refused", "no bake kind for it")


def _levels_by_duration():
    """`[(level, threshold_s), ...]` longest threshold first.

    Read from `soot_plume.DURATION_S` rather than copied, so a new rung added
    there is picked up here — the memory note "new levels must be in
    soot_plume.DURATION_S" cuts both ways.
    """
    from disaster import soot_plume
    items = [(k, float(v)) for k, v in soot_plume.DURATION_S.items()]
    # F6 (3600) is a ROOF outcome, not a rung above F5 (3900); ordering by
    # threshold alone would slot it between F4 and F5 and hand it to every
    # building that happened to land there. Collapse rungs are assigned
    # separately by `_apply_collapse`, so drop them from the clock ladder.
    items = [(k, v) for k, v in items if k not in ("F6", "F5c")]
    items.sort(key=lambda kv: -kv[1])
    return items


def level_for_age(age_s):
    """The compartment-fire rung a building that has been alight *age_s*
    seconds has reached. `None` when the front has not reached it yet."""
    if age_s is None or age_s < 0:
        return None
    for name, thresh in _levels_by_duration():
        if age_s >= thresh:
            return name
    return "F1" if age_s > 0 else None


def unbakeable(reason):
    """True when `burnable()`'s refusal is PERMANENT — no bake would fix it."""
    r = str(reason or "").lower()
    if any(h in r for h in _PERMANENT_HINTS):
        return True
    return not any(h in r for h in _BAKEABLE_HINTS)


def corridor_frame(heading_deg):
    """Unit vectors `(along, cross)` for a wind blowing towards *heading_deg*.

    Bearing convention matches the presets' `heading_deg`: 0 is +y, 90 is +x,
    i.e. compass-style clockwise from north, which is what `hurricane.
    wind_bearing_at` and `tornado.wind_at` already use.
    """
    th = math.radians(float(heading_deg))
    along = (math.sin(th), math.cos(th))
    cross = (math.cos(th), -math.sin(th))
    return along, cross


def project(x, y, origin, along, cross):
    dx, dy = x - origin[0], y - origin[1]
    return dx * along[0] + dy * along[1], dx * cross[0] + dy * cross[1]


def in_corridor(s, c, length_m, half_width_m, spread_deg):
    if s < 0.0 or s > length_m:
        return False
    half = half_width_m + max(0.0, s) * math.tan(math.radians(spread_deg))
    return abs(c) <= half


def select(placements, houses_idx, *, heading_deg, origin, length_m,
           half_width_m=DEFAULT_HALF_WIDTH_M, spread_deg=DEFAULT_SPREAD_DEG):
    """`[(i, placement, s, c), ...]` for every house inside the corridor."""
    along, cross = corridor_frame(heading_deg)
    out = []
    for i in houses_idx:
        p = placements[i]
        s, c = project(float(p["x_m"]), float(p["y_m"]), origin, along, cross)
        if in_corridor(s, c, length_m, half_width_m, spread_deg):
            out.append((i, p, s, c))
    out.sort(key=lambda t: t[2])
    return out


def fit_length(placements, houses_idx, *, heading_deg, origin, target_n,
               half_width_m=DEFAULT_HALF_WIDTH_M,
               spread_deg=DEFAULT_SPREAD_DEG, max_length_m=1600.0):
    """Shortest corridor length that holds at least *target_n* houses.

    Bisection on length rather than on width: the level ladder is meant to
    read as "the fire has run further", so growing the corridor along the
    wind is the behaviour we want when a level asks for more coverage.
    Widening is the fallback the caller applies when the plate runs out.
    """
    lo, hi = 0.0, float(max_length_m)
    for _ in range(40):
        mid = (lo + hi) / 2.0
        n = len(select(placements, houses_idx, heading_deg=heading_deg,
                       origin=origin, length_m=mid,
                       half_width_m=half_width_m, spread_deg=spread_deg))
        if n >= target_n:
            hi = mid
        else:
            lo = mid
    return hi


def place_corridor(placements, houses_idx, region_m, *, heading_deg,
                   target_n, half_width_m=DEFAULT_HALF_WIDTH_M,
                   spread_deg=DEFAULT_SPREAD_DEG, permanent_bad=frozenset(),
                   n_offsets=13):
    """Choose WHERE the corridor runs.

    The origin slides across the wind (the corridor always enters from the
    upwind edge, which is what makes it read as a fire that came from
    somewhere) and each offset is scored on how many PERMANENTLY unbakeable
    buildings it swallows — those are the only ones that must stay pristine
    inside the burn, and every one of them is a hole a reviewer will see.
    Ties break towards the shorter corridor, i.e. the denser part of the city.

    Returns `(origin, length_m, members, n_permanent_bad)`.
    """
    along, cross = corridor_frame(heading_deg)
    half_x = float(region_m[0]) / 2.0
    half_y = float(region_m[1]) / 2.0
    # Start the corridor outside the upwind corner so `s` is positive across
    # the whole plate: back off along the wind by the plate's half-diagonal.
    back = math.hypot(half_x, half_y)
    best = None
    span = max(half_x, half_y)
    for k in range(n_offsets):
        frac = (k / (n_offsets - 1.0)) * 2.0 - 1.0 if n_offsets > 1 else 0.0
        off = frac * span * 0.75
        origin = (-along[0] * back + cross[0] * off,
                  -along[1] * back + cross[1] * off)
        length = fit_length(placements, houses_idx, heading_deg=heading_deg,
                            origin=origin, target_n=target_n,
                            half_width_m=half_width_m, spread_deg=spread_deg)
        members = select(placements, houses_idx, heading_deg=heading_deg,
                         origin=origin, length_m=length,
                         half_width_m=half_width_m, spread_deg=spread_deg)
        if len(members) < target_n:
            continue
        bad = sum(1 for i, _p, _s, _c in members if i in permanent_bad)
        key = (bad, length)
        if best is None or key < best[0]:
            best = (key, origin, length, members, bad)
    if best is None:
        return None, None, [], 0
    _key, origin, length, members, bad = best
    return origin, length, members, bad


def assign_levels(members, *, epoch_s, length_m, front_speed_mps=None):
    """`{i: (level, t_ignite_s, age_s)}` along the corridor.

    The front is at the corridor's far end at scene time, so
    ``v = length / epoch`` unless the caller pins a speed. A building's age
    is the time since the front passed it, which makes the origin the oldest
    and the head the youngest — the gradient the whole model exists for.
    """
    v = (float(front_speed_mps) if front_speed_mps
         else (float(length_m) / float(epoch_s) if epoch_s > 0 else 0.0))
    out = {}
    for i, _p, s, _c in members:
        t_ig = (max(0.0, s) / v) if v > 0 else 0.0
        age = float(epoch_s) - t_ig
        lvl = level_for_age(age)
        if lvl is None:
            continue
        out[i] = (lvl, t_ig, age)
    return out, v


def apply_collapse(levels, members, *, n_collapse=0, n_f6=0, seed=0,
                   exclude=frozenset()):
    """Promote the OLDEST F5 records to F5c / F6.

    Collapse is a consequence of how long a compartment has been alight, so
    it belongs at the burnt-out tail rather than being sprinkled. Deterministic
    given the same corridor.
    """
    # `levels` is assigned to every geometric corridor member, including
    # permanent firebreaks that the manifest later omits.  Promoting one of
    # those omitted records silently consumes the small F6/F5c budget and can
    # leave a level-3 scene with no visible collapse at all.  Exclude them at
    # selection time so the requested promotions survive into the manifest.
    f5 = [i for i, (lvl, _t, _a) in levels.items()
          if lvl == "F5" and i not in exclude]
    f5.sort(key=lambda i: -levels[i][2])          # oldest first
    for i in f5[:int(n_f6)]:
        levels[i] = ("F6",) + levels[i][1:]
    for i in f5[int(n_f6):int(n_f6) + int(n_collapse)]:
        levels[i] = ("F5c",) + levels[i][1:]
    return levels
