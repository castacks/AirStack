#!/usr/bin/env python3
"""Render airstack_default.json with NUM_ROBOTS tabs from a single-robot template.

Foxglove layout JSON has no native templating, so we generate it at GCS startup
based on the NUM_ROBOTS env var. Tab[0] of the input file is treated as the
canonical robot_1 template; we replicate it for robots 1..NUM_ROBOTS, mint
unique panel IDs per tab, and patch the 3D panel's per-robot transforms /
topics / namespaces to cover the same range.
"""

import argparse
import copy
import json
import os
import re

PANEL_ID_RE = re.compile(r'^([A-Za-z0-9_.\- ]+)!(\w+)$')
ROBOT_KEY_RE = re.compile(r'^(.*?)robot_(\d+)(.*)$')
# Trailing `_r<digits>` is the per-robot suffix this script appends. Strip it
# before re-minting so re-running over its own output doesn't stack suffixes
# (e.g. `_r1_r1`) — gcs startup runs render_layout.py on every container start.
PANEL_SUFFIX_RE = re.compile(r'_r\d+$')


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
    ap.add_argument('--input', default=os.environ.get(
        'LAYOUT_TEMPLATE',
        '/root/AirStack/gcs/foxglove_extensions/airstack_default.json'))
    ap.add_argument('--output', default=os.environ.get(
        'LAYOUT_OUTPUT',
        '/root/AirStack/gcs/foxglove_extensions/airstack_default.json'))
    ap.add_argument('--num-robots', type=int,
                    default=int(os.environ.get('NUM_ROBOTS', '1')))
    args = ap.parse_args()

    with open(args.input) as f:
        template = json.load(f)
    rendered = expand_layout(template, args.num_robots)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(rendered, f, indent=2)
    print(f'rendered {args.num_robots}-robot layout → {args.output}')


if __name__ == '__main__':
    main()
