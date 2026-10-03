"""Unit tests for scripts/svg_led_daemon.py (ESC LED packet port of modal_io.c).

    cd ~/AirStack/robot/ros_ws/src/svg_ground_control && python3 -m pytest test/test_led_packet.py
"""
import importlib.util
import os

import pytest

_PATH = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'svg_led_daemon.py')
_spec = importlib.util.spec_from_file_location('svg_led_daemon', _PATH)
led = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(led)


def test_crc16_vectors():
    # crc16_init() seed
    assert led.crc16(b'') == 0xFFFF
    # one byte through the C table by hand: (0xFFFF>>8) ^ table[(0xFFFF^0)&0xFF] = 0xFF ^ 0x4040
    assert led.crc16(b'\x00') == 0x40BF
    # this table + init 0xFFFF is CRC-16/MODBUS; standard check value
    assert led.crc16(b'123456789') == 0x4B37


def test_rgbw_packet_layout():
    n = 11
    data = led.solid_frame(n, (1, 2, 3, 4), rgbw=True)
    pkt = led.build_neopixel_packet(data, n, rgbw=True)
    assert len(pkt) == 4 * n + 6
    assert pkt[0] == 0xAF
    assert pkt[1] == len(pkt)
    assert pkt[2] == 26            # RGBW array cmd
    assert pkt[3] == 0             # ESC 0 (default, as ModalAI's neopixel tool)
    assert led.build_neopixel_packet(data, n, True, esc_id=255)[3] == 255   # modal_io.c broadcast
    assert pkt[4:8] == bytes([1, 2, 3, 4])
    crc = led.crc16(pkt[1:-2])
    assert pkt[-2] == crc & 0xFF and pkt[-1] == crc >> 8   # little-endian


def test_rgb_packet_layout():
    n = 5
    data = led.solid_frame(n, (9, 8, 7, 6), rgbw=False)
    assert len(data) == 3 * n and data[:3] == bytes([9, 8, 7])
    pkt = led.build_neopixel_packet(data, n, rgbw=False)
    assert pkt[2] == 25 and len(pkt) == 3 * n + 6


def test_bounds():
    with pytest.raises(ValueError):
        led.build_neopixel_packet(b'', 0, True)
    with pytest.raises(ValueError):
        led.build_neopixel_packet(bytes(4 * 33), 33, True)
    with pytest.raises(ValueError):
        led.build_neopixel_packet(bytes(4 * 3), 4, True)   # wrong data length


def test_parse_color():
    assert led.parse_color('green') == (0, 255, 0, 0)
    assert led.parse_color('GREEN', 80) == (0, 80, 0, 0)
    assert led.parse_color('off') == (0, 0, 0, 0)
    assert led.parse_color('255,0,0') == (255, 0, 0, 0)
    assert led.parse_color('10,20,30,40', 255) == (10, 20, 30, 40)
    assert led.parse_color('255,255,255', 51) == (51, 51, 51, 0)
    for bad in ('pink', '1,2', '1,2,3,4,5', '300,0,0', 'a,b,c'):
        with pytest.raises(ValueError):
            led.parse_color(bad)


def test_x25_crc():
    assert led.x25_crc(b'') == 0xFFFF
    assert led.x25_crc(b'123456789') == 0x6F91      # CRC-16/MCRF4XX check value


def test_tunnel_message_struct():
    n = 11
    pkt = led.build_neopixel_packet(led.solid_frame(n, (0, 80, 0, 0)), n, True)   # 50 bytes
    msg = led.build_tunnel_message_struct(pkt, seq=7, payload_type=201, crc_extra=147)
    assert len(msg) == 291                          # sizeof(mavlink_message_t), MAVPACKED
    assert msg[2] == 0xFD                           # magic (v2)
    plen = msg[3]
    assert msg[4] == 0 and msg[5] == 0              # incompat/compat flags
    assert msg[6] == 7 and msg[7] == 255 and msg[8] == 191   # seq, sysid, compid
    assert msg[9:12] == (385).to_bytes(3, 'little')          # TUNNEL msgid
    payload = msg[12:12 + 133]
    ptype, tsys, tcomp, length = led.struct.unpack('<HBBB', payload[:5])
    assert (ptype, tsys, tcomp, length) == (201, 1, 1, len(pkt))
    assert payload[5:5 + len(pkt)] == pkt
    # v2 trailing-zero trim: len covers everything up to the last non-zero payload byte
    # (the two ck bytes that finalize_message drops after the payload are not payload)
    full_payload = led.struct.pack('<HBBB', 201, 1, 1, len(pkt)) + pkt + bytes(128 - len(pkt))
    assert plen == len(full_payload.rstrip(b'\x00'))
    assert payload[:plen] == full_payload[:plen]
    # checksum = X.25 over (len..msgid + trimmed payload) then crc_extra, stored LE at offset 0
    crc = led.x25_crc(msg[3:12] + msg[12:12 + plen])
    crc = led.x25_crc(bytes([147]), crc)
    assert msg[0:2] == crc.to_bytes(2, 'little')
    assert msg[12 + plen:12 + plen + 2] == msg[0:2]          # ck bytes after the payload too
    with pytest.raises(ValueError):
        led.build_tunnel_message_struct(bytes(129), 0)


def test_ground_ip_parse(tmp_path):
    f = tmp_path / 'voxl-px4-start'
    f.write_text('px4-microdds_client start -t udp -h 192.168.50.6 -p 8888 -n drone_1\n')
    assert led.ground_ip_from_px4_start(str(f)) == '192.168.50.6'
    assert led.ground_ip_from_px4_start(str(tmp_path / 'missing')) is None


def test_px4_led_muter(monkeypatch):
    calls = []
    monkeypatch.setattr(led.Px4LedMuter, 'px4_pid', staticmethod(lambda: 4242))
    monkeypatch.setattr(led.subprocess, 'run',
                        lambda cmd, **kw: (calls.append(cmd), type('R', (), {'returncode': 0, 'stdout': b'ok'})())[1])
    mu = led.Px4LedMuter('mute-cmd', settle_s=0.0, log=lambda m: None)
    mu.poll()                      # first sight of the pid: records it, no command yet
    assert calls == []
    mu.next_check = 0.0
    mu.poll()                      # settled -> command sent once
    assert calls == ['mute-cmd'] and mu.muted_pid == 4242
    mu.next_check = 0.0
    mu.poll()                      # same pid -> not repeated
    assert calls == ['mute-cmd']
    monkeypatch.setattr(led.Px4LedMuter, 'px4_pid', staticmethod(lambda: 4343))
    mu.next_check = 0.0; mu.poll(); mu.next_check = 0.0; mu.poll()   # PX4 restarted -> sent again
    assert calls == ['mute-cmd', 'mute-cmd'] and mu.muted_pid == 4343


def test_stale_seq_and_fallback(monkeypatch):
    args = led.parse_args(['--name', 'drone_1', '--ground-ip', '127.0.0.1', '--port', '0',
                           '--dry-run', '--fallback-s', '10'])
    d = led.LedDaemon(args, led.DryRunSink(lambda m: None), lambda m: None)
    assert d.color == led.parse_color('green', 80)
    d.handle_datagram(b'{"seq": 5, "r": 0, "g": 0, "b": 255, "w": 0}', ('127.0.0.1', 1))
    assert d.color == (0, 0, 255, 0)
    d.handle_datagram(b'{"seq": 4, "r": 255, "g": 0, "b": 0, "w": 0}', ('127.0.0.1', 1))  # stale
    assert d.color == (0, 0, 255, 0)
    d.handle_datagram(b'{"seq": 6, "r": 255, "g": 0, "b": 0, "w": 0, "mode": "blink", "hz": 4}',
                      ('127.0.0.1', 1))
    assert d.color == (255, 0, 0, 0) and d.mode == 'blink' and d.blink_hz == 4.0
    # blink frame alternates between color and off
    on = led.solid_frame(11, (255, 0, 0, 0))
    off = led.solid_frame(11, (0, 0, 0, 0))
    frames = {d.current_frame(t) for t in [0.0, 0.125, 0.25, 0.375]}
    assert frames == {on, off}
    d.sock.close()
