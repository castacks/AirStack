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


def test_dragonrise_usb_profile():
    p = c.get_controller('dragonrise_usb')
    # DirectInput layout: right stick on 2/3 (Xbox has it on 3/4), left
    # bumper is button 6 (Xbox: 4). Axes 4 and 5 are the analog triggers and
    # must never appear in the map — they rest at full scale.
    assert (p.forward_axis, p.left_axis, p.climb_axis, p.yaw_axis) == (3, 2, 1, 0)
    assert p.lock_button == 6
    assert not {4, 5} & {p.forward_axis, p.left_axis, p.climb_axis, p.yaw_axis}


def test_xbox_and_dragonrise_differ_on_the_right_stick():
    xbox = c.get_controller('xbox_usb')
    dragon = c.get_controller('dragonrise_usb')
    assert (xbox.forward_axis, xbox.left_axis) != (dragon.forward_axis,
                                                   dragon.left_axis)
    # Signs are shared: both follow the Linux ABS convention via joy_node.
    assert (xbox.forward_sign, xbox.left_sign, xbox.climb_sign,
            xbox.yaw_sign) == (dragon.forward_sign, dragon.left_sign,
                               dragon.climb_sign, dragon.yaw_sign)


@pytest.mark.parametrize('name', c.controller_names())
def test_no_profile_maps_a_velocity_axis_onto_a_resting_trigger(name):
    """Each profile's four velocity axes must be distinct real stick axes."""
    p = c.get_controller(name)
    axes = [p.forward_axis, p.left_axis, p.climb_axis, p.yaw_axis]
    assert len(set(axes)) == 4, f'{name} reuses an axis: {axes}'
    assert all(a >= 0 for a in axes)


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
