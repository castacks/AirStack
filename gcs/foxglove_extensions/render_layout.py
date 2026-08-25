#!/usr/bin/env python3
"""Render airstack_layout_custom.json with NUM_ROBOTS tabs from a single-robot template.

Foxglove layout JSON has no native templating, so we generate it at GCS startup
based on the NUM_ROBOTS env var. Tab[0] of the input file is treated as the
canonical robot_1 template; we replicate it for robots 1..NUM_ROBOTS, mint
unique panel IDs per tab, and patch the 3D panel's per-robot transforms /
topics / namespaces to cover the same range.

Also injects display settings for the /rayfronts_debug/<robot>/voxels_sim/all
clouds (VOXEL_SCORE_THRESHOLD env var, see inject_rayfronts_debug). The
per-query clouds need no layout settings — their topic names embed the mission
query, so foxglove_visualizer_node republishes them as pre-colored scene-entity
cubes instead.
"""

import argparse
import colorsys
import copy
import json
import os
import re

PANEL_ID_RE = re.compile(r'^([A-Za-z0-9_.\- ]+)!(\w+)$')
ROBOT_KEY_RE = re.compile(r'^(.*?)robot_(\d+)(.*)$')
# Strip every trailing `_r<digits>` suffix this script previously appended so
# re-running over its own output doesn't stack (e.g. `_r1_r1_r1...`).
PANEL_SUFFIX_RE = re.compile(r'(_r\d+)+$')

# Mirror of gcs_visualizer.gcs_utils.ROBOT_COLORS — not importable here (the
# ROS workspace isn't sourced when this runs at container start).
ROBOT_COLORS = [
    (0.90, 0.10, 0.10),
    (0.10, 0.70, 0.20),
    (0.20, 0.40, 1.00),
    (1.00, 0.55, 0.00),
    (0.70, 0.30, 0.90),
    (0.00, 0.80, 0.85),
    (1.00, 0.85, 0.10),
    (1.00, 0.40, 0.70),
    (0.40, 0.80, 0.40),
    (0.55, 0.27, 0.07),
    (0.30, 0.30, 0.30),
    (0.95, 0.95, 0.95),
]
# Robot 3's wheel blue fluoresces to a dark #0040ff; override with a brighter
# blue. Keyed by robot number; keep in sync with foxglove_visualizer_node.
VOXEL_HIGH_OVERRIDE = {3: '#3387ffff'}  # bright blue

VOXEL_CUBE_SIZE = 0.5
VOXEL_GRADIENT_LOW = '#54005eff'  # dark purple; sync with gcs_utils VOXEL_GRADIENT_LOW
DEFAULT_VOXEL_THRESHOLD = 0.65  # raven_nav voxel_score_threshold default


def _robot_color_hex(n: int) -> str:
    """High end of robot n's voxel gradient: a fluorescent take on its wheel
    color (full value, boosted saturation), with a per-robot override (see
    VOXEL_HIGH_OVERRIDE). Keep in sync with foxglove_visualizer_node."""
    if n in VOXEL_HIGH_OVERRIDE:
        return VOXEL_HIGH_OVERRIDE[n]
    r, g, b = ROBOT_COLORS[(n - 1) % len(ROBOT_COLORS)]
    h, s, _ = colorsys.rgb_to_hsv(r, g, b)
    r, g, b = colorsys.hsv_to_rgb(h, min(1.0, s * 1.25), 1.0)
    return f'#{round(r * 255):02x}{round(g * 255):02x}{round(b * 255):02x}ff'


def inject_conavgpt2(layout: dict) -> None:
    """Add the conavgpt2 layers to every 3D panel, as an OVERLAY.

    The occupancy grid is the same flat plane as the sim ground, so it is only
    readable as a layer if it is translucent and the ground under it is not —
    hence alpha 0.55 here and `ground_alpha` 1.0 in foxglove_visualizer_node.
    (The node also lifts the grid by occupancy_z_offset_m so the two planes do
    not z-fight.)

    colorMode `custom` and not the default: `costmap` maps 0..100 through a
    nav-stack cost palette that says nothing here, because this grid only ever
    holds three values — 0 free, 100 occupied, -1 unknown. Free is drawn pale
    and occupied dark, and unknown is fully transparent so the ground shows
    through and the grid reads as COVERAGE rather than as a sheet over the
    scene.

    Frontier markers are opaque: they are the decision being visualised.
    """
    for pid, cfg in layout.get('configById', {}).items():
        if not (pid.startswith('3D!') and isinstance(cfg, dict)):
            continue
        topics = cfg.setdefault('topics', {})
        topics['/conavgpt2/occupancy'] = {
            'visible': True,
            'colorMode': 'custom',
            'alpha': 0.25,
            # FOUR classes out of a message type that defines three. ROS
            # OccupancyGrid is -1 unknown plus 0..100 cost, so the node stamps
            # detected target instances with 101 — outside that range, which a
            # viewer paints with its INVALID colour. That is the fourth slot.
            'minColor': '#ffffff',        #   0  free space      -> white
            'maxColor': '#000000',        # 100  obstacle        -> black
            'unknownColor': '#00000000',  #  -1  never observed  -> transparent
            'invalidColor': _robot_color_hex(1),   # 101 target  -> robot colour
            'frameLocked': False,
        }
        topics['/conavgpt2/frontiers'] = {'visible': True}
        # GT from the layout generator (scene_annotations.py). Namespaced per
        # class, so house/car/tree/person can be toggled independently.
        topics['/gcs/annotations/bboxes'] = {'visible': True}


def inject_rayfronts_debug(layout: dict, num_robots: int,
                           voxel_threshold: float) -> None:
    """Add per-robot settings for /rayfronts_debug/<robot>/voxels_sim/all to
    every 3D panel: 0.5 m outlined cubes (cubeOutline renders thin
    theme-colored edges — white in dark mode) on a dark-purple → per-robot-color
    gradient spanning [voxel_threshold, 1.0]. colorField pins sim_0 (the cloud
    has one sim_<q> per query — switch fields in the panel, the gradient
    stays); it must be pinned, since with no colorField in the layout Foxglove
    auto-selects one and force-resets colorMode to colormap/turbo."""
    for pid, cfg in layout.get('configById', {}).items():
        if not (pid.startswith('3D!') and isinstance(cfg, dict)):
            continue
        topics = cfg.setdefault('topics', {})
        for n in range(1, num_robots + 1):
            # Overwrite (no setdefault): fully derived from env, so re-rendering
            # over own output repairs the per-robot gradient colors that
            # _expand_per_robot clones from robot_1.
            topics[f'/rayfronts_debug/robot_{n}/voxels_sim/all'] = {
                'visible': False,
                'pointShape': 'cube',
                'cubeSize': VOXEL_CUBE_SIZE,
                'cubeOutline': True,
                'colorField': 'sim_0',
                'colorMode': 'gradient',
                'gradient': [VOXEL_GRADIENT_LOW, _robot_color_hex(n)],
                'minValue': voxel_threshold,
                'maxValue': 1.0,
            }
            # rgb voxel map: colour by the packed 'rgb' field (scene's real
            # colours), 0.5 m cubes. colorField/colorMode pinned so Foxglove
            # doesn't auto-reset to colormap/turbo (same reason as above).
            topics[f'/rayfronts_debug/robot_{n}/voxel_rgb'] = {
                'visible': False,
                'pointShape': 'cube',
                'cubeSize': VOXEL_CUBE_SIZE,
                'colorField': 'rgb',
                'colorMode': 'rgb',
            }


def replace_robot_n(obj, src_n: int, dst_n: int):
    """Deep-replace robot_{src_n} → robot_{dst_n} in strings and dict keys.
    Also handles the human-readable 'robot {N}' tab title form."""
    src_us, dst_us = f'robot_{src_n}', f'robot_{dst_n}'
    src_sp, dst_sp = f'robot {src_n}', f'robot {dst_n}'

    def _swap(s: str) -> str:
        return s.replace(src_us, dst_us).replace(src_sp, dst_sp)

    def _do(o):
        if isinstance(o, dict):
            return {(_swap(k) if isinstance(k, str) else k): _do(v)
                    for k, v in o.items()}
        if isinstance(o, list):
            return [_do(v) for v in o]
        if isinstance(o, str):
            return _swap(o)
        return o

    return _do(obj)


def find_panel_ids(obj, ids=None):
    """Collect every panel-ID string (`Pkg!suffix`) appearing in a layout tree."""
    if ids is None:
        ids = set()
    if isinstance(obj, str):
        if PANEL_ID_RE.fullmatch(obj):
            ids.add(obj)
    elif isinstance(obj, dict):
        for v in obj.values():
            find_panel_ids(v, ids)
    elif isinstance(obj, list):
        for v in obj:
            find_panel_ids(v, ids)
    return ids


def remap_panel_ids(obj, mapping):
    if isinstance(obj, str):
        return mapping.get(obj, obj)
    if isinstance(obj, dict):
        return {k: remap_panel_ids(v, mapping) for k, v in obj.items()}
    if isinstance(obj, list):
        return [remap_panel_ids(v, mapping) for v in obj]
    return obj


def _mint_id(pid: str, n: int) -> str:
    m = PANEL_ID_RE.fullmatch(pid)
    if not m:
        return pid
    base = PANEL_SUFFIX_RE.sub('', m.group(2))
    return f'{m.group(1)}!{base}_r{n}'


def _expand_per_robot(obj, num_robots: int) -> None:
    """Walk obj in-place. For every dict whose keys match
    `<prefix>robot_<N><suffix>`, drop existing per-robot keys and re-add
    cloned ones for N=1..num_robots, using the lowest-N entry as the template."""
    if isinstance(obj, list):
        for v in obj:
            _expand_per_robot(v, num_robots)
        return
    if not isinstance(obj, dict):
        return

    by_base = {}
    keys_to_drop = []
    for k, v in obj.items():
        m = ROBOT_KEY_RE.match(k)
        if m:
            base = (m.group(1), m.group(3))
            n = int(m.group(2))
            cur = by_base.get(base)
            if cur is None or n < cur[0]:
                by_base[base] = (n, v)
            keys_to_drop.append(k)

    for k in keys_to_drop:
        obj.pop(k, None)

    for (prefix, suffix), (src_n, template_value) in by_base.items():
        for dst_n in range(1, num_robots + 1):
            new_key = f'{prefix}robot_{dst_n}{suffix}'
            obj[new_key] = replace_robot_n(template_value, src_n, dst_n)

    for v in obj.values():
        _expand_per_robot(v, num_robots)


def expand_layout(template_json: dict, num_robots: int) -> dict:
    out = copy.deepcopy(template_json)
    config_by_id = out['configById']

    tab_key = next(
        (k for k, v in config_by_id.items()
         if k.startswith('Tab!') and isinstance(v, dict) and 'tabs' in v),
        None)
    if tab_key is None:
        raise SystemExit('No Tab!* container with tabs[] found in configById')

    tab_container = config_by_id[tab_key]
    tabs = tab_container.get('tabs', [])
    if not tabs:
        raise SystemExit('Tab container has no tabs to use as template')

    template_tab = copy.deepcopy(tabs[0])
    template_panel_ids = find_panel_ids(template_tab.get('layout', {}))
    template_configs = {pid: copy.deepcopy(config_by_id[pid])
                        for pid in template_panel_ids if pid in config_by_id}

    old_panel_ids = set()
    for t in tabs:
        old_panel_ids |= find_panel_ids(t.get('layout', {}))
    for pid in old_panel_ids:
        config_by_id.pop(pid, None)

    new_tabs = []
    for n in range(1, num_robots + 1):
        cloned_tab = replace_robot_n(template_tab, 1, n)
        id_map = {pid: _mint_id(pid, n) for pid in template_panel_ids}
        cloned_tab['layout'] = remap_panel_ids(cloned_tab['layout'], id_map)
        new_tabs.append(cloned_tab)
        for old_pid, new_pid in id_map.items():
            if old_pid in template_configs:
                config_by_id[new_pid] = replace_robot_n(
                    template_configs[old_pid], 1, n)

    tab_container['tabs'] = new_tabs
    if tab_container.get('activeTabIdx', 0) >= num_robots:
        tab_container['activeTabIdx'] = 0

    for k, v in config_by_id.items():
        if k.startswith('3D!') and isinstance(v, dict):
            _expand_per_robot(v, num_robots)

    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', help='Source template JSON (LAYOUT_TEMPLATE env)',
                    default=os.environ.get(
                        'LAYOUT_TEMPLATE',
                        '/root/AirStack/gcs/foxglove_extensions/airstack_default.json'))
    ap.add_argument('--output', help='Rendered output (LAYOUT_OUTPUT env). Defaults to '
                    '/root/airstack_layout_num_robots_<N>.json so the file appears in '
                    'Foxglove\'s "Import Layout" file browser.',
                    default=os.environ.get('LAYOUT_OUTPUT'))
    ap.add_argument('--num-robots', type=int,
                    default=int(os.environ.get('NUM_ROBOTS', '1')))
    ap.add_argument('--voxel-threshold', type=float,
                    help='Gradient min for voxels_sim/all sim coloring '
                         '(VOXEL_SCORE_THRESHOLD env)',
                    default=float(os.environ.get('VOXEL_SCORE_THRESHOLD')
                                  or DEFAULT_VOXEL_THRESHOLD))
    args = ap.parse_args()
    if args.output is None:
        args.output = f'/root/airstack_layout_num_robots_{args.num_robots}.json'

    with open(args.input) as f:
        template = json.load(f)
    rendered = expand_layout(template, args.num_robots)
    inject_rayfronts_debug(rendered, args.num_robots, args.voxel_threshold)
    inject_conavgpt2(rendered)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    tmp = args.output + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(rendered, f, indent=2)
    os.replace(tmp, args.output)
    print(f'rendered {args.num_robots}-robot layout → {args.output}')


if __name__ == '__main__':
    main()
