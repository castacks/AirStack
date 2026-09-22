import json,math
from pathlib import Path
import pytest
from conditions import validate,DIFFICULTY_COUNTS
from campaign import clean_twin,candidates
from episode import resolved
from feedback import choose_next
from dashboard import feedback_command
from layout_summary import describe

def overlaps(a,b,gap=.149):
    return all(a[0][i]<b[1][i]+gap and b[0][i]<a[1][i]+gap for i in range(3))

def test_catalog_counts_nested_geometry_and_protected_regions():
    catalog=json.loads(Path(__file__).with_name('layouts.json').read_text())
    for seed in range(8):
        previous={}
        for tier,count in DIFFICULTY_COUNTS.items():
            entry=catalog[tier][seed];c=validate({'layout':tier,'layout_seed':seed})
            assert describe(c)['added_counts']=={'plants':count,'columns':count}
            actual=entry['added_bounds_source_m'];assert len(actual)==1+2*count
            assert all(actual[k]==b for k,b in previous.items())
            boxes=list(actual.values())
            assert all(not overlaps(a,b) for i,a in enumerate(boxes) for b in boxes[i+1:])
            assert all(not overlaps(a,b) for a in boxes for b in entry['protected_regions_source_m'])
            previous=actual
        assert catalog['hard'][seed]['plant']!=catalog['hard'][(seed+1)%8]['plant']

@pytest.mark.parametrize('tier',['easy','medium','hard'])
def test_difficulty_is_independent_of_attack_and_fixed_in_feedback(tier,tmp_path):
    decision=choose_next([],profile='combined',difficulty=tier)
    assert decision['condition']['layout']==tier
    config=resolved({'planner':'kim','condition':decision['condition']})
    assert clean_twin(config)['condition']['layout']==tier
    assert clean_twin(config)['condition']['layout_seed']==config['condition']['layout_seed']
    failed={'outcome':'collision','metrics':{}}
    history=[{'decision':decision,'pairs':[{'planner':'kim','clean':failed,'perturbed':failed,'verdict':'invalid_clean_baseline'}]}]
    next_=choose_next(history,profile='combined',difficulty=tier)
    assert next_['condition']['layout']==tier and next_['condition']['layout_seed']!=decision['condition']['layout_seed']
    assert next_['level']==decision['level']
    argv=feedback_command({'planner':'kim','difficulty':tier},tmp_path)
    assert argv[argv.index('--difficulty')+1]==tier
    samples=candidates('random',4,42,['kim','mononav'],difficulty=tier)
    assert all(c['condition']['layout']==tier for c in samples)
    with pytest.raises(ValueError):feedback_command({'planner':'kim','difficulty':['hard']},tmp_path)
