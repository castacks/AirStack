"""EXP123 actual-byte storage replay of locked EXP122 arrivals and queries."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/retention-capacity/run-001'
PLAN = ROOT / 'experiments/retention-capacity-2026-11-03.json'
OLD = ROOT / 'results/buffered-handover/run-001/captures'
CAPACITIES = [None, 0, 2 * 1048576, 4 * 1048576, 8 * 1048576]
ARMS = ['unperturbed', 'full_read_cache', 'bounded_probe', 'fixture_off', 'fixture_on', 'buffered_probe']


def read(p): return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def write(p, value):
    p = Path(p); p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('x') as f: json.dump(value, f, indent=2, allow_nan=False); f.write('\n')
def name(cap): return 'unlimited' if cap is None else str(cap)
def records(p): return [json.loads(line) for line in Path(p).read_text().splitlines()]


def freeze():
    assert not PLAN.exists() and not OUT.exists()
    parent_path = ROOT / 'experiments/buffered-handover-2026-11-03.json'
    parent = read(parent_path)
    pins = dict(parent['input_source_sha256'])
    pins[str(parent_path.relative_to(ROOT))] = sha(parent_path)
    for arm in ARMS:
        for filename in ['retained-inputs.json', 'timeline.jsonl']:
            p = OLD / arm / filename; pins[str(p.relative_to(ROOT))] = sha(p)
    for path in ['MAC-VO-main/Scripts/Setup/capacity_retained_inputs.py',
                 'MAC-VO-main/Scripts/UnitTest/test_retention_capacity.py',
                 'tools/retention_capacity_study.py', 'tools/retention_capacity_audit.py',
                 'docs/research/retention-capacity-protocol-2026-11-03.md']:
        pins[path] = sha(ROOT / path)
    for path, digest in pins.items(): assert sha(ROOT / path) == digest, path
    head = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip()
    plan = {'date': '2026-11-03', 'parent_commit': head, 'window': parent['window'],
            'runtime_image': parent['runtime_image'], 'arms': ARMS, 'capacities_bytes': CAPACITIES,
            'cases': 30, 'input_source_sha256': pins, 'eviction': 'oldest source then camera lexicographic',
            'scope': 'fixed trace storage availability only; no native or delivery counterfactual',
            'capacity_scope': 'store-owned encoded bytes; staging/leases/IPC/RSS excluded',
            'training': False, 'reserve_access': False, 'policy_adopted': False}
    write(PLAN, plan); write(OUT / 'plan.json', plan)
    print('FROZEN', len(pins), 'pins; 30 storage cases')


def verify():
    plan = read(PLAN); assert sha(PLAN) == sha(OUT / 'plan.json')
    for path, digest in plan['input_source_sha256'].items(): assert sha(ROOT / path) == digest, path
    return plan


def replay():
    plan = verify()
    sys.path.insert(0, str(ROOT / 'MAC-VO-main'))
    from Scripts.Setup.capacity_retained_inputs import CapacityInputBuffer
    rows = plan['window']['rows']
    summaries = []
    for arm in plan['arms']:
        old = read(OLD / arm / 'retained-inputs.json')
        queries = [e for e in records(OLD / arm / 'timeline.jsonl') if e['kind'] == 'buffer_query']
        lookups = [e for e in records(OLD / arm / 'timeline.jsonl') if e['kind'] == 'buffer_lookup']
        assert len(queries) == len(lookups)
        for cap in plan['capacities_bytes']:
            dest = OUT / 'cases' / arm / name(cap); dest.mkdir(parents=True, exist_ok=False)
            b = CapacityInputBuffer(ROOT / plan['window']['dataset'], rows, arm != 'unperturbed', cap)
            hits = []
            for arrival in old['arrivals']:
                camera, source = arrival['camera'], arrival['source']
                b.advance(camera, source)
                if cap is None:
                    for key in ['camera', 'source', 'expired_sources', 'stored', 'images', 'retained_sources']:
                        assert b.audit[-1].get(key) == arrival.get(key), (arm, key, source)
                for q in queries:
                    if q['camera'] == camera and q['delivery_group'] == source:
                        payload = b.retrieve(camera, q['source'], source)
                        hits.append({'camera': camera, 'source': q['source'], 'delivery_group': source,
                                     'retained': payload is not None,
                                     'images': {side: {k: v for k, v in image.items() if k != 'data'}
                                                for side, image in payload['images'].items()} if payload else None})
            report = b.report()
            if cap is None:
                for key in ['ingress_bytes', 'retrieved_bytes', 'peak_retained_bytes']:
                    assert report[key] == old[key], (arm, key)
                assert [h['retained'] for h in hits] == [h['retained'] for h in lookups]
            summary = {'arm': arm, 'capacity_bytes': cap, 'queries': len(hits),
                       'hits': sum(h['retained'] for h in hits), 'misses': sum(not h['retained'] for h in hits),
                       **{k: report[k] for k in ['ingress_bytes', 'retrieved_bytes', 'peak_retained_bytes',
                                                 'evicted_pairs', 'rejected_pairs']}}
            write(dest / 'retention.json', report); write(dest / 'queries.json', hits)
            write(dest / 'summary.json', summary); summaries.append(summary)
            print(arm, name(cap), 'hits', summary['hits'], '/', summary['queries'], flush=True)
    verify()
    write(OUT / 'replay-lock.json', {'utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                                   'plan_sha256': sha(PLAN), 'sha256': {
                                       str(p.relative_to(ROOT)): sha(p) for p in (OUT / 'cases').rglob('*.json')}})
    write(OUT / 'summary.json', {'cases': summaries, 'native_execution': False,
                               'scope': plan['scope']})


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('action', choices=['freeze', 'verify', 'replay'])
    a = p.parse_args()
    if a.action == 'verify': print('VERIFIED', len(verify()['input_source_sha256']), 'pins')
    else: globals()[a.action]()
