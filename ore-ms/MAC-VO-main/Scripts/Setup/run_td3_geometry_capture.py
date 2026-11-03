"""Opt-in v3 execution entrypoint with copied returned-pose disparity information.

Run with the same flags as run_td3_execution_v3, including --record-arbiter-events.
This entrypoint changes observation only; the original runner file stays unchanged.
"""
import runpy
import sys
from . import arbiter_event_recorder
from .geometry_event_recorder import GeometryEventRecorder

if __name__=='__main__':
    if '--record-arbiter-events' not in sys.argv:
        raise SystemExit('geometry capture requires --record-arbiter-events')
    original=arbiter_event_recorder.ArbiterEventRecorder
    arbiter_event_recorder.ArbiterEventRecorder=GeometryEventRecorder
    try:runpy.run_module('Scripts.Setup.run_td3_execution_v3',run_name='__main__')
    finally:arbiter_event_recorder.ArbiterEventRecorder=original
