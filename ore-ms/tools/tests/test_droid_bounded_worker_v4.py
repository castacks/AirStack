from pathlib import Path
import sys
from types import SimpleNamespace
import pytest
import torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import droid_bounded_worker_v4 as b


def video(count=8,slots=64):
 f=torch.arange(slots*2*128*8*8,dtype=torch.int32).remainder(1024).to(torch.float16).view(slots,2,128,8,8)
 return SimpleNamespace(counter=SimpleNamespace(value=count),fmaps=f)


def test_compaction_preserves_all_active_and_reserved_bits():
 v=video();old=v.fmaps;g=b.Guard(64)
 b.compact_feature_buffer(v,g)
 assert v.counter.value==8 and v.fmaps.shape==(25,2,128,8,8)
 assert torch.equal(v.fmaps.view(torch.int16),old[:25].view(torch.int16))
 assert g.feature_compaction['removed_unused_slots']==39
 assert g.feature_compaction['all_active_and_reserved_feature_bits_preserved'] is True


def test_reserve_boundary_and_no_source_state_removed():
 v=video(count=47);old=v.fmaps.clone();g=b.Guard(64)
 b.compact_feature_buffer(v,g)
 assert v.fmaps.shape[0]==64 and torch.equal(v.fmaps,old)
 v=video(count=48)
 with pytest.raises(b.NativeCapacityExceeded):b.compact_feature_buffer(v,b.Guard(64))


def test_nan_payload_bits_are_preserved():
 v=video();v.fmaps[1,0,0,0,0]=float('nan');bits=v.fmaps[:25].view(torch.int16).clone()
 b.compact_feature_buffer(v,b.Guard(64))
 assert torch.equal(v.fmaps.view(torch.int16),bits)


def test_unexpected_feature_layout_refused():
 v=video();v.fmaps=v.fmaps[:32]
 with pytest.raises(ValueError):b.compact_feature_buffer(v,b.Guard(64))


def test_active_features_exceeding_native_index_limit_refused_before_allocation():
 v=SimpleNamespace(counter=SimpleNamespace(value=4079),fmaps=torch.empty(4096,2,128,31,98,dtype=torch.float16,device='meta'))
 with pytest.raises(b.NativeFeatureIndexCapacityExceeded):b.compact_feature_buffer(v,b.Guard(4096))
