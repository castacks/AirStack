"""Independent metadata-only oracle; never imports the capacity implementation."""
import json
from retention_capacity_study import ROOT, OUT, OLD, read, sha, write, records, verify, name


def audit():
    plan = verify(); checks = 0
    def check(condition, label):
        nonlocal checks
        if not condition: raise AssertionError(label)
        checks += 1
    lock = read(OUT / 'replay-lock.json')
    check(lock['plan_sha256'] == sha(ROOT / 'experiments/retention-capacity-2026-11-03.json'), 'plan lock')
    for path, digest in lock['sha256'].items(): check(sha(ROOT / path) == digest, path)
    summaries = []
    for arm in plan['arms']:
        old = read(OLD / arm / 'retained-inputs.json')
        query_events = [e for e in records(OLD / arm / 'timeline.jsonl') if e['kind'] == 'buffer_query']
        for cap in plan['capacities_bytes']:
            dest = OUT / 'cases' / arm / name(cap)
            report = read(dest / 'retention.json'); queries = read(dest / 'queries.json')
            store = {}; ingress = peak = evicted = rejected = retrieved = 0; expected_queries = []
            check(len(report['arrivals']) == len(old['arrivals']), 'arrival count')
            for expected, actual in zip(old['arrivals'], report['arrivals']):
                camera, source = expected['camera'], expected['source']
                expiry = sorted(s for c, s in store if c == camera and s < source - 1)
                store = {key: value for key, value in store.items() if not (key[0] == camera and key[1] < source - 1)}
                check(actual['expired_sources'] == expiry, 'expiry')
                check((actual['camera'], actual['source']) == (camera, source), 'arrival order')
                check(actual.get('images') == expected.get('images'), 'input metadata/hashes')
                dropped = []; too_large = False; admitted = False
                if expected['stored']:
                    images = expected['images']; size = sum(i['bytes'] for i in images.values()); ingress += size
                    too_large = cap is not None and size > cap
                    if too_large: rejected += 1
                    else:
                        while cap is not None and sum(v[0] for v in store.values()) + size > cap:
                            victim = sorted(store, key=lambda key: (key[1], key[0]))[0]
                            del store[victim]; dropped.append({'camera': victim[0], 'source': victim[1]}); evicted += 1
                        store[(camera, source)] = (size, images); admitted = True
                resident = sum(v[0] for v in store.values()); peak = max(peak, resident)
                check(cap is None or resident <= cap, 'payload cap')
                check(actual['resident_bytes'] == resident, 'resident byte count')
                check(actual['resident'] == {c: sorted(s for cam, s in store if cam == c) for c in ['rgb', 'thermal']}, 'whole store')
                check(actual['retained_sources'] == sorted(s for c, s in store if c == camera), 'camera state')
                check(actual['stored'] == admitted, 'atomic admission')
                check(actual['rejected_oversize'] == too_large, 'oversize rejection')
                check(actual['evicted'] == dropped, 'deterministic eviction')
                for q in query_events:
                    if q['camera'] == camera and q['delivery_group'] == source:
                        check(q['source'] == source - 1, 'query age')
                        payload = store.get((camera, q['source']))
                        expected_queries.append({'camera': camera, 'source': q['source'], 'delivery_group': source,
                                                 'retained': payload is not None, 'images': payload[1] if payload else None})
                        if payload: retrieved += payload[0]
            check(queries == expected_queries, 'actual query bytes/hash and misses')
            for field, value in [('ingress_bytes', ingress), ('retrieved_bytes', retrieved),
                                 ('peak_retained_bytes', peak), ('evicted_pairs', evicted), ('rejected_pairs', rejected)]:
                check(report[field] == value, field)
            s = read(dest / 'summary.json')
            check(s == {'arm': arm, 'capacity_bytes': cap, 'queries': len(queries),
                        'hits': sum(q['retained'] for q in queries), 'misses': sum(not q['retained'] for q in queries),
                        'ingress_bytes': ingress, 'retrieved_bytes': retrieved, 'peak_retained_bytes': peak,
                        'evicted_pairs': evicted, 'rejected_pairs': rejected}, 'case summary')
            summaries.append(s)
    check(read(OUT / 'summary.json')['cases'] == summaries, 'aggregate summary')
    write(OUT / 'audit.json', {'checks': checks, 'cases': len(summaries), 'pass': True,
                             'frozen_pins': len(plan['input_source_sha256']),
                             'scope': 'independent metadata oracle; no estimator outcome claims'})
    print('PASS', checks, 'checks;', len(summaries), 'cases')


if __name__ == '__main__': audit()
