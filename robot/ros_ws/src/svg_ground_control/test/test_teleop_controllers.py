"""The teleop_controller registry: every profile is complete and lookups fail loudly."""

import pytest

from svg_ground_control.safe_teleop import controllers as c
from svg_ground_control.safe_teleop.velocity import VelocityMapper


def test_default_is_registered():
    assert c.DEFAULT_CONTROLLER in c.CONTROLLERS
    assert c.get_controller(c.DEFAULT_CONTROLLER).name == c.DEFAULT_CONTROLLER


def test_xbox_usb_profile():
    p = c.get_controller('xbox_usb')
    assert [d.executable for d in p.drivers] == ['joy_node']
    assert p.joy_topic == '/joy'
    # Right stick moves, left stick climbs/yaws, left bumper locks.
    assert (p.forward_axis, p.left_axis, p.climb_axis, p.yaw_axis) == (4, 3, 1, 0)
    assert p.lock_button == 4
    assert p.left_sign == -1.0


@pytest.mark.parametrize('name', c.controller_names())
def test_every_profile_feeds_the_mapper(name):
    p = c.get_controller(name)
    params = p.mapping_parameters()
    assert set(params) == {'forward_axis', 'left_axis', 'climb_axis', 'yaw_axis',
                           'lock_button', 'forward_sign', 'left_sign',
                           'climb_sign', 'yaw_sign'}
    # The mapper accepts the profile's map verbatim (same keyword names).
    VelocityMapper(**params)
    assert p.drivers, f'{name} must say how it becomes a Joy stream'


@pytest.mark.parametrize('bad', ['', 'xbox', 'XBOX_USB', 'ps4'])
def test_unknown_controller_lists_supported(bad):
    with pytest.raises(KeyError) as err:
        c.get_controller(bad)
    assert 'xbox_usb' in str(err.value)


def test_lookup_strips_whitespace():
    assert c.get_controller(' xbox_usb ').name == 'xbox_usb'
