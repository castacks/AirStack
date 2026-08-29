#!/usr/bin/env python3
"""The search area can come from the SCENE, and transit has its own gear.

Two additions to planner_node, pinned without ROS:

  1. `search_area_source: scene` reads `<scene>_region.json` — the polygon
     the Isaac launcher writes for a generated wildfire (its fire-front
     ellipse plus the evacuation band). The lookup is the same one the
     known-obstacle loader uses (source tree first), the entry is picked by
     class, and a missing file or entry RAISES: a planner that quietly fell
     back to the whole plat would look like one that searched the affected
     area, only slower.
  2. `transit_speed_mps` applies only while the robot is OUTSIDE its sector
     and not on a target; inside it is `explore_speed_mps`, on a target
     `target_speed_mps`, and 0 disables the third gear entirely.

Pure logic — the methods are sliced out of planner_node.py and exec'd on a
stub, in the manner of test_frame_consistency.py.
"""
import json
import os
import re
import sys
import tempfile
import textwrap

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PN_PATH = os.path.normpath(os.path.join(HERE, '..', 'search_baselines', 'planner_node.py'))
SRC = open(PN_PATH).read()
FAILS = []


def check(c, m):
    print(('  PASS  ' if c else '  FAIL  ') + m)
    if not c:
        FAILS.append(m)


def _method_src(name):
    """Source of `    def name(` including any decorator line above it."""
    i = SRC.index(f'    def {name}(')
    j = SRC.index('\n    def ', i + 10)
    # decorators sit on the lines immediately above
    k = i
    while True:
        prev = SRC.rfind('\n', 0, k - 1)
        line = SRC[prev + 1:k]
        if line.strip().startswith('@'):
            k = prev + 1
        else:
            break
    return SRC[k:j]


def _build(names, ns_extra=None):
    ns = {'np': np, 'os': os, 'json': json, '__file__': PN_PATH}
    ns.update(ns_extra or {})
    body = '\n'.join(textwrap.dedent(_method_src(n)) for n in names)
    exec('class T:\n' + textwrap.indent(body, '    '), ns)
    return ns['T']


class Log:
    def __init__(self):
        self.lines = []

    def warn(self, m, **kw):
        self.lines.append(('W', m))

    def info(self, m, **kw):
        self.lines.append(('I', m))


def main():
    print('=' * 66)
    print('SEARCH AREA SOURCE + TRANSIT SPEED')
    print('=' * 66)

    # ── [1] annotation lookup ─────────────────────────────────────────────
    print('\n[1] the region file is found the way the obstacle file is')
    T = _build(['_annotation_dirs', '_annotation_file', '_load_scene_region'])
    dirs = T._annotation_dirs()
    check(dirs and dirs[0].endswith(os.path.join('src', 'global', 'planners',
                                                 'raven_nav', 'annotations')),
          f'source-tree annotations dir first: {dirs[0] if dirs else dirs}')
    check(os.path.isdir(dirs[0]), 'and it exists in this checkout')

    with tempfile.TemporaryDirectory() as td:
        class S(T):
            _obst_scene = 'TestScene'

            @staticmethod
            def _annotation_dirs():
                return [td]
        s = S()
        # no file -> raise, and the message says what to do
        try:
            s._load_scene_region('affected')
            check(False, 'missing region file raises')
        except RuntimeError as exc:
            check('TestScene_region.json' in str(exc) and 'search_area_source' in str(exc),
                  f'missing region file raises: {str(exc)[:70]}...')
        # a file with the entries the launcher writes
        poly = [[-400.0, -400.0], [200.0, -400.0], [200.0, 200.0], [-400.0, 200.0]]
        doc = [{'class': 'burn', 'polygon_xy': [[0, 0], [1, 0], [1, 1]], 't_s': 400.0},
               {'class': 'affected', 'polygon_xy': poly, 't_s': 486.0},
               {'class': 'region', 'polygon_xy': [[-500, -500], [500, -500], [500, 500], [-500, 500]]},
               {'class': 'meta', 'elapsed_s': 400.0}]
        with open(os.path.join(td, 'TestScene_region.json'), 'w') as fh:
            json.dump(doc, fh)
        path, got = s._load_scene_region('affected')
        check(got == poly, f'affected polygon read back verbatim ({len(got)} pts)')
        check(path.endswith('TestScene_region.json'), f'from {os.path.basename(path)}')
        _, burn = s._load_scene_region('burn')
        check(len(burn) == 3, 'entries are picked by class')
        try:
            s._load_scene_region('nope')
            check(False, 'unknown key raises')
        except RuntimeError as exc:
            check("'nope'" in str(exc) and 'affected' in str(exc),
                  'unknown key raises and lists what the file has')
        # the obstacle loader's own lookup order is unchanged: _obstacles then bare
        open(os.path.join(td, 'TestScene.json'), 'w').write('[]')
        check(S._annotation_file('TestScene', ('_obstacles', '')).endswith('TestScene.json'),
              'obstacle lookup falls through _obstacles -> <scene>.json')
        open(os.path.join(td, 'TestScene_obstacles.json'), 'w').write('[]')
        check(S._annotation_file('TestScene', ('_obstacles', '')).endswith('_obstacles.json'),
              'and prefers <scene>_obstacles.json when both exist')

        class NoScene(T):
            _obst_scene = ''
        env = os.environ.pop('RESULTS_SCENE', None)
        try:
            NoScene()._load_scene_region('affected')
            check(False, 'no scene name raises')
        except RuntimeError as exc:
            check('RESULTS_SCENE' in str(exc), 'no scene name raises, naming RESULTS_SCENE')
        finally:
            if env is not None:
                os.environ['RESULTS_SCENE'] = env

    # ── [2] the __init__ wiring ───────────────────────────────────────────
    print('\n[2] __init__ wires the scene polygon in before the sector split')
    i_src = SRC.index("self._search_area_source = str(")
    i_sector = SRC.index("self._sector_mode = str(self._p('sector_partition'")
    i_conv = SRC.index("self._search_poly_converted = (self._search_area_frame != 'world')")
    check(i_src < i_conv < i_sector,
          'source is read, the frame forced to world, THEN the polygon is padded and cut')
    check("self._search_area_frame = 'world'" in SRC[i_src:i_conv],
          'a scene polygon forces search_area_frame=world (it is authored in world)')
    check(re.search(r"raise ValueError\(\s*f\"search_area_source must be 'config' or 'scene'", SRC)
          is not None, 'an unknown source is rejected at declare time')

    # ── [3] the intent speed ──────────────────────────────────────────────
    print('\n[3] transit gear only OUTSIDE the sector, and only when enabled')
    T2 = _build(['_intent_speed', '_to_map', '_to_map_arr', '_points_in_polygon'])
    sector = np.array([[-450.0, -470.0], [-260.0, -470.0], [-260.0, 490.0], [-450.0, 490.0]])

    class Stub(T2):
        _explore_speed = 3.0
        _target_speed = 1.5
        _transit_speed = 6.0
        _marker_offset = None
        _search_poly = sector
        _search_poly_converted = True
        _on = False
        _xy = (0.0, 0.0)

        def _on_target(self, agent):
            return self._on

        def _agent_xy(self, agent):
            return self._xy

    s = Stub()
    s._xy = (0.0, 0.0)                      # the spawn, 260 m from the strip
    check(s._intent_speed(None) == 6.0, 'outside the sector -> transit 6.0 m/s')
    s._xy = (-350.0, 10.0)                  # inside
    check(s._intent_speed(None) == 3.0, 'inside the sector -> explore 3.0 m/s')
    s._xy = (0.0, 0.0)
    s._on = True
    check(s._intent_speed(None) == 1.5, 'on a target -> target 1.5 m/s even outside')
    s._on = False
    s._transit_speed = 0.0
    check(s._intent_speed(None) == 3.0, 'transit_speed_mps 0 -> explore everywhere (old behaviour)')
    s._transit_speed = 6.0
    s._search_poly_converted = False
    check(s._intent_speed(None) == 3.0,
          'polygon not yet placed in the map (no GPS fix) -> explore, never transit')
    s._search_poly_converted = True
    s._search_poly = None
    check(s._intent_speed(None) == 3.0, 'no search area at all -> explore')
    # the map-frame test: with a marker offset the grid point is shifted first
    s._search_poly = sector
    s._marker_offset = np.array([-355.0, 10.0, 0.0])   # grid = map + offset
    s._xy = (0.0, 0.0)                                   # grid origin = map (355, -10)
    check(s._intent_speed(None) == 6.0, 'grid->map offset is applied before the polygon test')
    s._xy = (-355.0 * 2, 20.0)                            # map (-355, 10): inside
    check(s._intent_speed(None) == 3.0, '... and a grid point that maps inside is inside')

    # the log label follows the gear
    T3 = _build(['_speed_label'])

    class L(T3):
        _transit_speed, _explore_speed, _target_speed = 7.0, 1.5, 1.5
    lab = L()
    check(lab._speed_label(7.0) == 'transit to sector' and lab._speed_label(1.5) == 'search',
          'speed label: 7.0 -> transit to sector, 1.5 (explore == target) -> search')

    class L2(T3):
        _transit_speed, _explore_speed, _target_speed = 7.0, 3.0, 1.5
    lab = L2()
    check(lab._speed_label(1.5) == 'target approach' and lab._speed_label(3.0) == 'explore',
          'speed label with distinct explore/target: target approach / explore')

    # ── [4] the cap rides on the NavigateTask goal ────────────────────────
    print('\n[4] the cap is carried on NavigateTask.Goal.max_speed_mps, not a parameter')
    setter = _method_src('_set_local_speed')
    check('SetParameters' not in SRC and '_speed_clients' not in SRC,
          'no droan parameter-service client left in planner_node')
    check('self._send_activator(i' in setter and "self._nav_mode == 'gpt'" in setter,
          '_set_local_speed re-sends the activator with the new cap; gpt arm exempt')
    check('self._stamp_speed(goal, i)' in _method_src('_send_activator')
          and 'self._stamp_speed(goal, i)' in _method_src('_command'),
          'both the activator and the goal_per_round goal are stamped with the cap')

    class G:
        max_speed_mps = -1.0

    class NoField:
        pass
    T4 = _build(['_stamp_speed'])

    class Sp(T4):
        _speed_set = {0: 7.0}
        warned = []

        def get_logger(self):
            outer = self

            class Lg:
                def warn(self, m, **kw):
                    outer.warned.append(m)
            return Lg()
    sp = Sp()
    g = G()
    check(sp._stamp_speed(g, 0) == 7.0 and g.max_speed_mps == 7.0,
          'stamp writes the robot cap onto the goal')
    sp._stamp_speed(NoField(), 0)
    check(any('max_speed_mps' in w and 'bws' in w for w in sp.warned),
          'a task_msgs without the field is warned about loudly, not ignored')
    check("ground speed" in _method_src('_log_ground_speed')
          and 'self._log_ground_speed(i, agent)' in SRC,
          'measured ground speed is logged from the tick with the cap and the gear')

    # ── [5] detector gate bookkeeping ─────────────────────────────────────
    print('\n[5] every gate decision is logged, and the summary carries the run max')
    T5 = _build(['_track_detector', '_log_detector_summary'])
    i_bins = SRC.index('    _DET_BINS = (')
    bins_line = SRC[i_bins:SRC.index('\n', i_bins)].strip()

    class Clock:
        t = 100.0

        def now(self):
            outer = self

            class N:
                nanoseconds = int(outer.t * 1e9)
            return N()

    class Det(T5):
        _sem_threshold = 0.65
        _goal_name = 'person'
        _robots = ['robot_1']
        _detector_log_period = 30.0
        _detector_log_conf = 0.5
        _det_stats = {}
        _clock = Clock()
        _log = Log()

        def get_clock(self):
            return self._clock

        def get_logger(self):
            return self._log
    exec(bins_line.replace('_DET_BINS', 'Det._DET_BINS'), {'Det': Det})

    class Ag:
        def __init__(self, hits, gmax, boxes=None):
            self.last_detection = {'n': hits, 'top': [('person', gmax), ('tree', 0.4)],
                                   'goal_hits': hits, 'goal_max': gmax,
                                   'goal_boxes': boxes or [(gmax, [10, 20, 30, 60])]}
    d = Det()
    d._track_detector(0, Ag(0, 0.0))          # nothing proposed
    d._track_detector(0, Ag(2, 0.58, [(0.58, [312, 201, 330, 240]), (0.51, [5, 5, 15, 30]),
                                      (0.31, [0, 0, 4, 9])]))   # seen, under the gate
    d._track_detector(0, Ag(1, 0.71))         # pass
    d._track_detector(0, Ag(3, 0.745))        # pass, run max
    d._track_detector(0, Ag(1, 0.42))         # proposed under the logging floor
    lines = [m for _, m in d._log.lines]
    check(sum('detector PASS' in m for m in lines) == 2
          and any('0.745 > gate 0.65' in m for m in lines),
          'two passes logged as "detector PASS: person <conf> > gate 0.65"')
    seen = [m for m in lines if 'detector SEEN' in m]
    check(len(seen) == 1 and '0.580 (below gate 0.65, not mapped)' in seen[0]
          and '2 box(es) >= 0.50' in seen[0] and '0.58 @ px[312, 201, 330, 240] 18x39' in seen[0]
          and '0.31' not in seen[0] and "('tree', 0.4)" in seen[0],
          f'0.58 is logged as SEEN with its >=0.5 boxes and pixel rects: {seen[0][:120] if seen else seen}')
    check(any('detector below gate: person best 0.420 < 0.50' in m for m in lines),
          'a proposal under the logging floor goes to the throttled line')
    st = d._det_stats[0]
    check(st['ticks'] == 5 and st['proposed'] == 4 and st['passed'] == 2
          and st['max'] == 0.745 and st['bins'] == [0, 1, 1, 2, 0],
          f'tally: ticks 5, proposed 4, passed 2, max 0.745, bins {st["bins"]}')
    d._clock.t += 31.0
    d._track_detector(0, Ag(0, 0.0))
    summ = [m for m in [m for _, m in d._log.lines] if 'detector summary' in m]
    check(len(summ) == 1 and '6 ticks' in summ[0] and 'proposed on 4' in summ[0]
          and 'passed gate 0.65 on 2' in summ[0] and 'run max 0.745' in summ[0]
          and '0.65-0.8:2' in summ[0],
          f'periodic summary after 30 s sim: {summ[0][:110] if summ else summ}...')
    d._log_detector_summary(0, final=True)
    check(any('detector summary (FINAL)' in m for _, m in d._log.lines),
          'final summary at budget end is marked FINAL')
    check('detector gate:' in SRC and '_log_detector_summary(i, final=True)' in SRC,
          'gate is announced at startup and the final summary is wired into _finish_run')
    check("< self._detector_log_conf" in _method_src('_publish_detection_images'),
          'the evidence JPEG follows the logging floor, not the gate')
    AG = open(os.path.join(HERE, '..', 'search_baselines', 'airstack_agent.py')).read()
    check("'goal_boxes': goal_boxes" in AG and 'xyxy' in AG,
          'the detect wrapper records every goal-class box with its pixel rect')

    # ── [6] nav_activation follows LOCAL_PLANNER; MIGHTY's bridge takes the cap ─
    print('\n[6] LOCAL_PLANNER=mighty -> follower activation; the bridge caps waypoint speed')
    i_na = SRC.index("self._nav_activation = str(self._p('nav_activation'")
    blk = SRC[i_na:SRC.index('\n\n', i_na)]
    check("os.environ.get('LOCAL_PLANNER'" in blk and "'follower' if lp == 'mighty' else 'activator'" in blk,
          "nav_activation 'auto' resolves from LOCAL_PLANNER (mighty -> follower)")
    cmd = _method_src('_command')
    check("if self._nav_activation == 'follower':" in cmd
          and "self._nav_activation in ('activator', 'follower')" in _method_src('_set_local_speed'),
          'follower mode sends no keep-alive goal but still sends the speed-only goal on a gear change')
    BR = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..', 'modules',
                                       'asm_mighty', 'mighty_bridge', 'mighty_bridge', 'bridge_node.py'))
    if os.path.exists(BR):
        bsrc = open(BR).read()
        ns = {}
        i = bsrc.index('def capped_speed('); j = bsrc.index('\n\n\n', i)
        exec(bsrc[i:j], ns)
        cs = ns['capped_speed']
        check(cs(6.9, 1.5) == 1.5 and cs(1.2, 1.5) == 1.2 and cs(6.9, 0.0) == 6.9,
              'bridge: waypoint speed is min(v, cap), and cap 0 means uncapped')
        check('if not goal_request.global_plan.poses:\n            return GoalResponse.ACCEPT' in bsrc
              and "speed-only NavigateTask" in bsrc and 'wp.velocity = capped_speed(' not in bsrc,
              'bridge: an empty-plan goal is accepted as a speed-only activator; the cap is NOT applied to waypoints (MIGHTY anchoring)')
        check("speed cap requested {cap:.1f} m/s" in bsrc
              and 'self._set_mode(TrajectoryMode.Request.TRACK)' in bsrc,
              'bridge logs the requested cap and switches the controller to TRACK on engage')
    else:
        check(False, f'asm_mighty bridge not found at {BR}')
    LL = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..', 'local', 'local_bringup',
                                       'launch', 'local.launch.xml'))
    lsrc = open(LL).read()
    check('$(env LOCAL_PLANNER droan)' in lsrc and 'mighty_module.launch.xml' in lsrc
          and 'stereo_image_proc/point_cloud' in lsrc and 'camera_left' in lsrc,
          'local.launch.xml: LOCAL_PLANNER switch, MIGHTY fed the stereo cloud in camera_left')

    print('\n' + ('ALL PASS' if not FAILS else f'{len(FAILS)} FAILED:'))
    for f in FAILS:
        print('   - ' + f)
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
