import hashlib
import json

import pytest

from audit_corridor_suite import map_source_equivalence


def test_map_revision_cannot_hide_a_scorer_change(tmp_path):
    before="selected += [8.,length-9.] if spec['shape']=='u_obstacles' else [5.5,length-6.]"
    after="selected += [10.,length-10.] if spec['shape']=='u_obstacles' else [5.5,length-6.]"
    old=('def geometry():\n    '+before+'\n\ndef score():\n    return 1\n').encode()
    new=old.replace(before.encode(),after.encode())
    (tmp_path/'old.py').write_bytes(old)
    current=tmp_path/'current.py';current.write_bytes(new)
    old_hash=hashlib.sha256(old).hexdigest();new_hash=hashlib.sha256(new).hexdigest()
    (tmp_path/'map_revision.json').write_text(json.dumps(dict(
        schema='hard_u_spacing_revision_v1',old_source='old.py',old_sha256=old_hash,
        new_sha256=new_hash,replace_old=before,replace_new=after,affected_map=['hard','u_obstacles'])))
    assert map_source_equivalence(tmp_path,current)=={old_hash:old_hash,new_hash:old_hash}
    current.write_bytes(new.replace(b'return 1',b'return 2'))
    with pytest.raises(AssertionError,match='other behavior'):map_source_equivalence(tmp_path,current)
