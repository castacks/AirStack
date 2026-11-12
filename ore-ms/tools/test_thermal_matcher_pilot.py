"""Verify intervention boundaries and failure cleanup without neural inference."""
from types import SimpleNamespace
import pytest
import torch
import thermal_matcher_pilot as pilot
from Module.Frontend.Frontend import FlowFormerCovFrontend as Frontend
from Module.Frontend.Matching import IMatcher


@pytest.mark.parametrize('arm', ['passthrough', 'adapted'])
def test_only_flow_replaced_and_binding_restored(monkeypatch, tmp_path, arm):
    original_flow = torch.zeros(1, 2, 3, 4)
    covariance = torch.ones(1, 3, 3, 4)
    mask = torch.ones(1, 1, 3, 4, dtype=torch.bool)
    depth = object()
    def native(self, a, b):
        return depth, IMatcher.Output(flow=original_flow, cov=covariance, mask=mask)
    monkeypatch.setattr(Frontend, 'estimate_pair', native)
    monkeypatch.setattr(pilot, 'OUT', tmp_path)
    matcher = SimpleNamespace(predict=lambda *a, **kw: SimpleNamespace(
        flow=torch.ones_like(original_flow), covisibility=torch.zeros(1, 3, 4)))
    monkeypatch.setattr(pilot, 'load_matcher', lambda: matcher)
    first = SimpleNamespace(frame_ns=100, imageL=torch.zeros(1, 3, 3, 4))
    second = SimpleNamespace(frame_ns=200, imageL=torch.zeros(1, 3, 3, 4))
    with pilot.frontend_override(arm) as receipts:
        d, m = Frontend.estimate_pair(None, first, second)
        assert d is depth and m.cov is covariance and m.mask is mask
        assert torch.equal(m.flow, original_flow if arm == 'passthrough' else torch.ones_like(original_flow))
        assert len(receipts) == 1
    assert Frontend.estimate_pair is native


def test_invalid_candidate_stops_and_restores(monkeypatch):
    def native(self, a, b):
        return None, IMatcher.Output(flow=torch.zeros(1, 2, 3, 4))
    monkeypatch.setattr(Frontend, 'estimate_pair', native)
    monkeypatch.setattr(pilot, 'load_matcher', lambda: SimpleNamespace(
        predict=lambda *a, **kw: SimpleNamespace(flow=torch.full((1, 2, 3, 4), float('nan')))))
    frame = SimpleNamespace(imageL=torch.zeros(1, 3, 3, 4))
    with pytest.raises(ValueError, match='Invalid replacement'):
        with pilot.frontend_override('adapted'):
            Frontend.estimate_pair(None, frame, frame)
    assert Frontend.estimate_pair is native
