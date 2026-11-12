"""Tests for background policy loading and the between-tick hand-over."""

import threading

from svg_ground_control.policy_hot_swap import PolicyHotSwap


def test_loaded_policy_is_handed_over_once():
    swap = PolicyHotSwap(lambda path: f'policy from {path}')
    swap.request('a.zip').join()

    loaded = swap.take_ready()
    assert loaded.path == 'a.zip' and loaded.policy == 'policy from a.zip'
    assert swap.take_ready() is None


def test_newer_request_supersedes_a_slower_older_one():
    release_first = threading.Event()

    def load(path):
        if path == 'old.zip':
            release_first.wait(timeout=5.0)  # finishes after the newer request
        return path

    swap = PolicyHotSwap(load)
    old = swap.request('old.zip')
    swap.request('new.zip').join()
    release_first.set()
    old.join()

    assert swap.take_ready().path == 'new.zip'
    assert swap.take_ready() is None


def test_load_failure_is_reported_and_nothing_is_swapped():
    def load(path):
        raise FileNotFoundError(path)

    swap = PolicyHotSwap(load)
    swap.request('missing.zip').join()

    assert swap.take_ready() is None
    errors = swap.take_errors()
    assert len(errors) == 1 and 'missing.zip' in errors[0] and 'FileNotFoundError' in errors[0]
    assert swap.take_errors() == []
