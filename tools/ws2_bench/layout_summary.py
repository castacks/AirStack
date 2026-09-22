"""Describe realized Office object counts and per-seed placement changes."""
import json,math
from pathlib import Path

def describe(condition,previous=None):
    catalog=json.loads(Path(__file__).with_name('layouts.json').read_text())
    entry=catalog.get(condition['layout'],[{}]*8)[condition['layout_seed']]
    old=catalog.get(previous['layout'],[{}]*8)[previous['layout_seed']] if previous else {}
    names={'move':'Existing plant','plant':'Added plant 1','column':'Added column','plant2':'Added plant 2'}
    names.update({f'{kind}_{i}':f'Added {kind} {i}' for i in range(2,6) for kind in ['plant','column']})
    objects=[]
    for key,label in names.items():
        if key not in entry:continue
        local=entry[key];world=[local[1],-local[0],local[2]]
        objects.append({'name':label,'operation':'move' if key=='move' else 'duplicate',
                        'offset_from_source_world_m':[round(x,3) for x in world],
                        'shift_from_previous_m':round(math.dist(entry[key],old[key]),3) if key in old else None})
    counts={'plants':sum(k.startswith('plant') for k in entry),'columns':sum(k.startswith('column') for k in entry)}
    tier=entry.get('difficulty','legacy' if condition['layout']!='stock' else 'stock')
    return {'library':f'{tier}: {counts["plants"]} plants + {counts["columns"]} columns added; '+('one existing plant moved' if 'move' in entry else 'original Office unchanged'),
            'difficulty':tier,'added_counts':counts,'objects':objects,
            'same_as_previous':bool(previous and all(condition[k]==previous[k] for k in ['layout','layout_seed']))}
