#!/usr/bin/env python3
"""Closed-set (COCO) vs open-vocabulary (YOLO-World) detector selection.

The vendored `Object_Detection_and_Segmentation.__init__` calls
`set_classes(...)` unconditionally, inside its own constructor. A COCO
checkpoint has no such method, so selecting yolov8x used to kill the planner
at start-up. `airstack_agent` installs a no-op on the `YOLO` base class (which
`YOLOWorld` does NOT inherit from, so the real method is never shadowed) and
resolves `goal_id` from the checkpoint's own `names`.

Pure logic — no ultralytics, no GPU, no ROS.
"""
import os, re, sys, textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = open(os.path.join(HERE, '..', 'search_baselines', 'airstack_agent.py')).read()
FAILS = []


def check(cond, msg):
    print(('  PASS  ' if cond else '  FAIL  ') + msg)
    if not cond:
        FAILS.append(msg)


def _noop_set_classes(self, classes):
    return None
_noop_set_classes._noop = True


class FakeCoco:
    """A closed-set checkpoint AS IT EXISTS AT RUNTIME.

    Critically it DOES carry `set_classes` — the shim `airstack_agent`
    installs on the YOLO base class so the vendored constructor does not
    crash. The first version of this test omitted it, which is exactly why it
    passed while the real thing threw `IndexError: list index out of range`
    from the vendored overlay: the probe saw the shim, took the open-vocab
    path, and left `self.classes` as the 1-entry requested list while the
    model emitted COCO ids 0-79.
    """
    names = {0: 'person', 1: 'bicycle', 2: 'car', 15: 'cat'}
    set_classes = _noop_set_classes


class FakeCocoBare:
    """Same, before the shim is installed."""
    names = {0: 'person', 1: 'bicycle', 2: 'car', 15: 'cat'}


class FakeWorld:
    names = {}
    def __init__(self):
        self.set_called_with = None
    def set_classes(self, classes):
        self.set_called_with = list(classes)


def resolve(model, goal_name, classes):
    """Mirror of the branch in airstack_agent.__init__."""
    _sc = getattr(model, 'set_classes', None)
    open_vocab = _sc is not None and not getattr(_sc, '_noop', False)
    if open_vocab:
        cls = list(classes)
        if goal_name not in cls:
            cls = cls + [goal_name]
        model.set_classes(cls)
        return open_vocab, cls, cls.index(goal_name)
    names = dict(getattr(model, 'names', {}) or {})
    cls = [names[k] for k in sorted(names)]
    match = [i for i, n in names.items() if str(n).lower() == str(goal_name).lower()]
    if not match:
        raise ValueError('goal_name not in closed-set vocabulary')
    return open_vocab, cls, int(match[0])


def main():
    print('=' * 66)
    print('CLOSED-SET DETECTOR SELECTION')
    print('=' * 66)

    print('\n[1] the shim exists and is installed on the YOLO BASE class')
    check('set_classes' in SRC and 'hasattr(_UltralyticsYOLO' in SRC,
          'airstack_agent installs a set_classes shim')
    check('_UltralyticsYOLO.set_classes = _set_classes_noop' in SRC,
          'shim is attached to ultralytics.YOLO (not YOLOWorld)')
    check(re.search(r"if not hasattr\(_UltralyticsYOLO, 'set_classes'\)", SRC) is not None,
          'shim installed only when the base class lacks the method')

    print('\n[2] closed-set: goal_id comes from the checkpoint names')
    for label, M in (('with the shim installed', FakeCoco),
                     ('before the shim', FakeCocoBare)):
        ov2, cls2, gid2 = resolve(M(), 'person', ['person'])
        check(ov2 is False,
              f'COCO takes the CLOSED-SET path {label}')
        check(len(cls2) == 4 and cls2[gid2] == 'person',
              f'  classes is the full checkpoint vocabulary ({len(cls2)}) {label}')
    # The exact runtime crash: a car detection (COCO id 2) indexing the list
    # the vendored overlay is handed.
    _, cls_r, _ = resolve(FakeCoco(), 'person', ['person'])
    check(len(cls_r) > 2, 'a COCO id 2 (car) can index classes without IndexError')

    ov, cls, gid = resolve(FakeCoco(), 'person', ['person', 'car'])
    check(ov is False, 'COCO checkpoint takes the closed-set path')
    check(gid == 0 and cls[gid] == 'person', f'goal_id={gid} -> {cls[gid]!r}')
    _, cls2, gid2 = resolve(FakeCoco(), 'car', ['irrelevant'])
    check(cls2[gid2] == 'car', 'a different goal resolves to its own id')
    check(cls2[gid2] != 'person', 'requested detection_classes do NOT shift the id')

    print('\n[3] a goal outside the fixed vocabulary fails LOUDLY')
    try:
        resolve(FakeCoco(), 'smoke', ['smoke'])
        check(False, 'unknown goal raises')
    except ValueError:
        check(True, 'unknown goal raises ValueError rather than silently missing')

    print('\n[4] open-vocab is unaffected')
    w = FakeWorld()
    ov, cls, gid = resolve(w, 'person', ['car', 'tree'])
    check(ov is True, 'YOLO-World takes the open-vocab path')
    check(w.set_called_with == ['car', 'tree', 'person'],
          f'set_classes still called with the prompt list: {w.set_called_with}')
    check(cls[gid] == 'person', 'goal_id indexes the prompt list')

    print('\n' + '=' * 66)
    if FAILS:
        print(f'{len(FAILS)} FAILED')
        for f in FAILS:
            print('   -', f)
        return 1
    print('all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
