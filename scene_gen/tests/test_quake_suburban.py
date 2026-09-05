"""CPU-only contracts for the separate timber-house earthquake adapter."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from disaster import quake_suburban as qs
from detail import modular_house as mh


def houses():
    return [dict(style=style, x=i*3., y=2., yaw=90.)
            for i, style in enumerate(mh.STYLES)]


def test_deterministic_and_order_independent_damage():
    rows = houses()
    first = qs.plan_houses(rows, {}, lambda x,y: .9, 10)
    assert first == qs.plan_houses(rows, {}, lambda x,y: .9, 10)
    reverse = qs.plan_houses(rows[::-1], {}, lambda x,y: .9, 10)
    for a, b in zip(first, reverse[::-1]):
        assert {k:v for k,v in a.items() if k != 'id'} == {
            k:v for k,v in b.items() if k != 'id'}


def test_zero_shaking_cannot_trigger_soil_damage():
    cfg = {'disaster': {'soft_soil': {
        'center': [0,0], 'rx_m': 100, 'ry_m': 100, 'rate': 1}}}
    rows = qs.plan_houses(houses(), cfg, lambda x,y: 0., 10)
    assert all(r['mode'] == 'pristine' and not r['settlement'] for r in rows)


def test_soft_soil_rotation_and_boundary():
    soil = dict(center=[0,0], rx_m=10, ry_m=2, angle_deg=90)
    assert math.isclose(qs.soft_weight(0,5,soil), .75)
    assert qs.soft_weight(5,0,soil) == 0
    assert qs.soft_weight(0,10,soil) == 0
    assert qs.soft_weight(0,0,False) == 0


def test_weak_storey_requires_multistorey_garage():
    modes = set()
    for seed in range(150):
        for row in qs.plan_houses(houses(), {}, lambda x,y: 1., seed):
            modes.add(row['mode'])
            if row['mode'] == 'soft_storey':
                assert row['storeys'] > 1 and mh.STYLES[row['style']].get('garage')
            assert row['grade'] == qs.GRADE[row['mode']]
    assert set(qs.MODES) == modes


def test_local_world_rotation_roundtrip():
    h = dict(x=20., y=-12., yaw=73.)
    x,y = qs._world(3.,-7.,h)
    u,v = qs.local_xy(x,y,h)
    assert math.isclose(u,3.) and math.isclose(v,-7.)


def test_rubble_support_requires_every_body_sample(monkeypatch):
    from disaster import quake_people as qp
    person = dict(x=0., y=0., z=0., pose='buried_reach', yaw_deg=0.)
    monkeypatch.setattr(qp, '_support_z', lambda *a, **kw: None)
    assert qs.rubble_body_support(None,person,{'prim':'house'},[],'/g') is None
    monkeypatch.setattr(qp, '_support_z',
                        lambda stage,r,*a,**kw: .02 if r['prim']=='/g/ground' else None)
    assert qs.rubble_body_support(None,person,{'prim':'house'},[],'/g') == (.02,9)


def test_rubble_support_rejects_isolated_high_shard(monkeypatch):
    from disaster import quake_people as qp
    person = dict(x=0., y=0., z=1., pose='buried_reach', yaw_deg=0.)
    pts = qp._prone_support_points(0.,0.,'buried_reach',0.)
    def support(stage,r,x,y,*a,**kw):
        if r['prim'] == '/g/ground':
            return .02
        return 1. if (x,y)==pts[0] else None
    monkeypatch.setattr(qp,'_support_z',support)
    assert qs.rubble_body_support(None,person,{'prim':'house'},[],'/g') is None
