#!/usr/bin/env python3
"""SVG onboard LED daemon — runs ON THE VOXL2, no ROS.

Drives the drone's NeoPixel strip (ESC LED output) and takes color commands from
the ground PC over UDP. Ported from the ModalAI reference in
led_ws/src/led_manager (vendor/modal_io/modal_io.c + crc16.c): the strip is
driven by writing an ESC "LED array" packet into the voxl-px4 pipe
/run/mpa/modal_io_bridge. Why no ROS here: the VOXL runs ROS 2 Foxy and the
ground stack Jazzy — Foxy DDS on the ground domain breaks the Jazzy tooling
(experiment.md B6), so this daemon speaks plain UDP instead.

Protocol (all JSON, one datagram each; ONE socket bound to --port):
  daemon -> ground:<--ground-port>  every 1 s   {"name": "drone_1", "seq": 12, "uptime": 34.5,
                                                 "color": [r,g,b,w], "mode": "solid"}
  ground -> daemon (reply to that source)       {"seq": 1712345678901, "r": 0, "g": 80, "b": 0,
                                                 "w": 0, "mode": "solid"|"blink", "hz": 2.0}
Stale commands (seq <= last accepted) are ignored, so late datagrams cannot
revert a color. The ground node (svg_ground_control/led_controller.py) learns
each drone's address from the heartbeat, so no IP configuration is needed.

PX4 interference (found on the Starling 2 Max, ESC fw 39.21): PX4's voxl_esc
driver stamps its status-LED bits (disarmed: red; armed: blue/green, OFFBOARD:
red) into every motor command, and the ESC mirrors those bits onto an active
NeoPixel strip -> our green alternates with PX4's red at the 20 Hz passthrough
rate and looks like an orange flicker. The driver's LED test mode freezes those
bits: `px4-qshell voxl_esc -l 0 led` (options BEFORE the verb). The daemon runs
that at start and again after every voxl-px4 restart (--px4-led-mute-cmd;
--no-px4-led-mute to disable). Side effect: the ESCs' own status LEDs stop
showing the arm state.

Behaviour:
  * shows --default-color (green) immediately at start — before any ground link
  * re-sends the frame at --refresh-hz (blink is generated here)
  * after --fallback-s without any ground datagram, reverts to the default color
  * SIGINT/SIGTERM: strip off, exit

Must stay Python 3.6 compatible (VOXL2 SDK ships python3.6) and stdlib-only.

    svg_led_daemon.py --name drone_1                # ground IP read from voxl-px4-start
    svg_led_daemon.py --name drone_1 --ground-ip 192.168.50.6 --num-leds 11
    svg_led_daemon.py --name drone_1 --ground-ip 127.0.0.1 --dry-run   # bench test, no hardware
"""
import argparse
import errno
import json
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import time

# --------------------------------------------------------------------------
# ESC LED packet (port of modal_io.c / crc16.c)
# --------------------------------------------------------------------------
ESC_PACKET_HEADER = 0xAF
ESC_PACKET_TYPE_LED_RGB_ARRAY_CMD = 25
ESC_PACKET_TYPE_LED_RGBW_ARRAY_CMD = 26
ESC_ID_ALL = 255          # modal_io.c broadcasts; ModalAI's voxl-esc-neopixel-test.py
ESC_ID_DEFAULT = 0        # addresses ESC 0 (its RGB_OUT pin drives the strip on M0138)
MAX_NEOPIXELS = 32
MODAL_IO_BRIDGE_SINK = '/run/mpa/modal_io_bridge'
VOXL_PX4_START = '/usr/bin/voxl-px4-start'

# CRC-16 (poly 0x8005 reflected, init 0xFFFF — same table as crc16.c / MODBUS)
CRC16_TABLE = [
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
    0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
    0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
    0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
    0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
    0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
    0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
    0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
    0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
    0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
    0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
    0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040,
]


def crc16(data, crc=0xFFFF):
    """crc16.c: crc16(crc16_init(), data, len)."""
    for b in bytearray(data):
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


def build_neopixel_packet(led_data, num_leds, rgbw=True, esc_id=ESC_ID_DEFAULT):
    """Byte-exact port of modal_io_send_neopixel_packet_generic()'s packet.

    [0xAF, len, type(25 RGB | 26 RGBW), esc_id, <3n or 4n bytes>, crc16 little-endian]
    CRC covers everything after the header (len, type, id, data). modal_io.c uses
    esc_id 255 (all ESCs); ModalAI's voxl-esc-neopixel-test.py uses 0 (--id).
    """
    if not 1 <= num_leds <= MAX_NEOPIXELS:
        raise ValueError('num leds should be between 1 and %d' % MAX_NEOPIXELS)
    bpp = 4 if rgbw else 3
    led_data = bytes(bytearray(led_data))
    if len(led_data) != bpp * num_leds:
        raise ValueError('expected %d bytes of LED data, got %d' % (bpp * num_leds, len(led_data)))
    length = bpp * num_leds + 6
    ptype = ESC_PACKET_TYPE_LED_RGBW_ARRAY_CMD if rgbw else ESC_PACKET_TYPE_LED_RGB_ARRAY_CMD
    out = bytearray([ESC_PACKET_HEADER, length, ptype, int(esc_id) & 0xFF]) + led_data
    crc = crc16(out[1:])
    out += bytearray([crc & 0xFF, (crc >> 8) & 0xFF])
    return bytes(out)


# --------------------------------------------------------------------------
# MAVLink TUNNEL fallback (port of modal_io_write_to_sink()'s second branch)
#
# voxl-px4 builds without the modal_io_bridge FIFO still accept the same ESC
# packet wrapped in a MAVLink TUNNEL message (payload type "ModalAI ESC UART
# passthru"), handed to voxl-mavlink-server through its control pipe, which
# forwards it to PX4. modal_io.c writes the raw C `mavlink_message_t` struct
# (MAVPACKED, 291 bytes) into that pipe — reproduced here byte for byte.
# --------------------------------------------------------------------------
MAVLINK_CONTROL_PIPE = '/run/mpa/mavlink_onboard/control'
MAVLINK_MSG_ID_TUNNEL = 385
MAVLINK_MSG_ID_TUNNEL_CRC = 147            # crc_extra of TUNNEL (common.xml)
MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_ESC_UART_PASSTHRU = 201   # ModalAI dialect; installer verifies
MAV_COMP_ID_ONBOARD_COMPUTER = 191
MAV_COMP_ID_AUTOPILOT1 = 1
MAVLINK_STX_V2 = 0xFD
TUNNEL_PAYLOAD_MAX = 128
MAVLINK_MESSAGE_T_SIZE = 291               # sizeof(mavlink_message_t), packed


def x25_crc(data, crc=0xFFFF):
    """MAVLink crc_accumulate() (CRC-16/MCRF4XX)."""
    for b in bytearray(data):
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def build_tunnel_message_struct(esc_packet, seq, payload_type=MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_ESC_UART_PASSTHRU,
                                crc_extra=MAVLINK_MSG_ID_TUNNEL_CRC, sysid=255,
                                compid=MAV_COMP_ID_ONBOARD_COMPUTER, target_system=1,
                                target_component=MAV_COMP_ID_AUTOPILOT1):
    """mavlink_msg_tunnel_pack(255, ONBOARD_COMPUTER, &msg, 1, AUTOPILOT1, type, len, data)
    as the in-memory mavlink_message_t that modal_io.c write()s to the control pipe.

    struct layout (packed): u16 checksum | u8 magic len incompat compat seq sysid compid |
    u24 msgid | u8 payload64[264] | u8 ck[2] | u8 signature[13]  = 291 bytes.
    TUNNEL wire payload: u16 payload_type, u8 target_system, u8 target_component,
    u8 payload_length, u8 payload[128] (133 bytes; MAVLink v2 trims trailing zeros).
    """
    esc_packet = bytes(bytearray(esc_packet))
    if not 1 <= len(esc_packet) <= TUNNEL_PAYLOAD_MAX:
        raise ValueError('tunnel payload must be 1..%d bytes' % TUNNEL_PAYLOAD_MAX)
    payload = (struct.pack('<HBBB', payload_type, target_system, target_component, len(esc_packet))
               + esc_packet + bytes(TUNNEL_PAYLOAD_MAX - len(esc_packet)))
    trimmed = payload.rstrip(b'\x00') or b'\x00'
    header = (struct.pack('<BBBBBB', len(trimmed), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF)
              + struct.pack('<I', MAVLINK_MSG_ID_TUNNEL)[:3])
    crc = x25_crc(header + trimmed)
    crc = x25_crc(bytes([crc_extra & 0xFF]), crc)
    ck = struct.pack('<H', crc)
    payload64 = bytearray(264)
    payload64[:len(payload)] = payload
    payload64[len(trimmed):len(trimmed) + 2] = ck     # finalize_message also drops ck after payload
    msg = ck + bytes([MAVLINK_STX_V2]) + header + bytes(payload64) + ck + bytes(13)
    assert len(msg) == MAVLINK_MESSAGE_T_SIZE
    return msg


def solid_frame(num_leds, rgbw_color, rgbw=True):
    """Whole strip one color. rgbw_color = (r, g, b, w); w dropped for RGB strips."""
    r, g, b, w = [max(0, min(255, int(c))) for c in rgbw_color]
    px = [r, g, b, w] if rgbw else [r, g, b]
    return bytes(bytearray(px * num_leds))


# --------------------------------------------------------------------------
# Colors (keep in sync with svg_ground_control/led_controller.py)
# --------------------------------------------------------------------------
COLOR_NAMES = {
    'off':     (0, 0, 0, 0),
    'black':   (0, 0, 0, 0),
    'red':     (255, 0, 0, 0),
    'green':   (0, 255, 0, 0),
    'blue':    (0, 0, 255, 0),
    'white':   (0, 0, 0, 255),
    'yellow':  (255, 160, 0, 0),
    'cyan':    (0, 255, 255, 0),
    'magenta': (255, 0, 255, 0),
    'orange':  (255, 60, 0, 0),
    'purple':  (128, 0, 255, 0),
}


def parse_color(text, brightness=255):
    """'green' or 'r,g,b[,w]' -> (r,g,b,w) scaled by brightness/255. Raises ValueError."""
    text = str(text).strip().lower()
    if text in COLOR_NAMES:
        base = COLOR_NAMES[text]
    else:
        parts = [p.strip() for p in text.split(',')]
        if len(parts) not in (3, 4):
            raise ValueError('unknown color %r (use a name or r,g,b[,w])' % text)
        vals = [int(p) for p in parts]
        if any(v < 0 or v > 255 for v in vals):
            raise ValueError('color channels must be 0-255')
        base = tuple(vals) + ((0,) if len(vals) == 3 else ())
    scale = max(0, min(255, int(brightness))) / 255.0
    return tuple(int(round(c * scale)) for c in base)


# --------------------------------------------------------------------------
# Sinks
# --------------------------------------------------------------------------
class PipeSink(object):
    """Writes ESC packets into the voxl-px4 modal_io_bridge FIFO.

    Mirrors modal_io.c: open O_WRONLY|O_NONBLOCK lazily; on ENOENT/ENXIO
    (voxl-px4 not up yet) retry every second; on a short write or EPIPE/EAGAIN
    close and reopen next time.
    """

    def __init__(self, path=MODAL_IO_BRIDGE_SINK, log=print, esc_id=ESC_ID_DEFAULT):
        self.path = path
        self.esc_id = esc_id
        self.fd = None
        self.last_try = 0.0
        self.warned = False
        self.log = log

    def _close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None

    def send(self, led_data, num_leds, rgbw):
        packet = build_neopixel_packet(led_data, num_leds, rgbw, self.esc_id)
        now = time.time()
        if self.fd is None:
            if now - self.last_try < 1.0:
                return False
            self.last_try = now
            try:
                self.fd = os.open(self.path, os.O_WRONLY | os.O_NONBLOCK)
            except OSError as e:
                if not self.warned:
                    self.log('WARN: cannot open %s (%s). Is voxl-px4 running and does '
                             'this build support modal_io_bridge? Retrying every 1 s.'
                             % (self.path, e.strerror))
                    self.warned = True
                return False
            self.log('opened LED sink %s' % self.path)
            self.warned = False
        try:
            n = os.write(self.fd, packet)
        except OSError as e:
            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                self.log('WARN: LED sink write failed (%s), reopening' % e.strerror)
            self._close()
            return False
        if n < len(packet):
            self.log('WARN: short write to LED sink, reopening')
            self._close()
            return False
        return True

    def close(self):
        self._close()


class MavlinkTunnelSink(object):
    """Writes the ESC packet as a MAVLink TUNNEL mavlink_message_t into
    voxl-mavlink-server's control pipe (modal_io.c fallback path)."""

    def __init__(self, path=MAVLINK_CONTROL_PIPE, payload_type=MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_ESC_UART_PASSTHRU,
                 crc_extra=MAVLINK_MSG_ID_TUNNEL_CRC, log=print, esc_id=ESC_ID_DEFAULT):
        self.path = path
        self.esc_id = esc_id
        self.payload_type = payload_type
        self.crc_extra = crc_extra
        self.fd = None
        self.seq = 0
        self.last_try = 0.0
        self.warned = False
        self.log = log

    def _close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None

    def send(self, led_data, num_leds, rgbw):
        packet = build_neopixel_packet(led_data, num_leds, rgbw, self.esc_id)
        msg = build_tunnel_message_struct(packet, self.seq, self.payload_type, self.crc_extra)
        now = time.time()
        if self.fd is None:
            if now - self.last_try < 1.0:
                return False
            self.last_try = now
            try:
                self.fd = os.open(self.path, os.O_WRONLY | os.O_NONBLOCK)
            except OSError as e:
                if not self.warned:
                    self.log('WARN: cannot open %s (%s) — is voxl-mavlink-server running? '
                             'Retrying every 1 s.' % (self.path, e.strerror))
                    self.warned = True
                return False
            self.log('opened MAVLink tunnel sink %s (TUNNEL payload_type=%d, crc_extra=%d, esc_id=%d)'
                     % (self.path, self.payload_type, self.crc_extra, self.esc_id))
            self.warned = False
        try:
            n = os.write(self.fd, msg)
        except OSError as e:
            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                self.log('WARN: tunnel write failed (%s), reopening' % e.strerror)
            self._close()
            return False
        if n < len(msg):
            self.log('WARN: short write to tunnel sink, reopening')
            self._close()
            return False
        self.seq = (self.seq + 1) & 0xFF
        return True

    def close(self):
        self._close()


class LibModalIoSink(object):
    """Alternative: call modal_io_send_neopixel_packet_generic() from libmodalio.so
    (what led_ws/src/led_manager/src/index_test.cpp links). Gives the MAVLink-tunnel
    fallback for free on builds without the modal_io_bridge pipe."""

    def __init__(self, path='/usr/lib64/libmodalio.so', log=print):
        import ctypes
        self.ctypes = ctypes
        self.lib = ctypes.CDLL(path)
        fn = self.lib.modal_io_send_neopixel_packet_generic
        fn.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint8, ctypes.c_int]
        fn.restype = ctypes.c_int
        self.fn = fn
        self.log = log
        log('using %s' % path)

    def send(self, led_data, num_leds, rgbw):
        buf = (self.ctypes.c_uint8 * len(led_data))(*bytearray(led_data))
        return self.fn(buf, num_leds, 1 if rgbw else 0) == 0

    def close(self):
        pass


class DryRunSink(object):
    """Bench testing without hardware: print the frame whenever it changes."""

    def __init__(self, log=print):
        self.log = log
        self.last = None

    def send(self, led_data, num_leds, rgbw):
        packet = build_neopixel_packet(led_data, num_leds, rgbw)
        if packet != self.last:
            self.last = packet
            bpp = 4 if rgbw else 3
            self.log('LED frame: pixel0=%s  (%d leds, %d-byte packet, esc_id=%d)'
                     % (list(bytearray(led_data[:bpp])), num_leds, len(packet), packet[3]))
        return True

    def close(self):
        pass


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def ground_ip_from_px4_start(path=VOXL_PX4_START):
    """The uXRCE client line written by voxl_setup_real_drone.sh carries '-h <ground ip>'."""
    try:
        with open(path) as f:
            m = re.search(r'-h\s+(\d{1,3}(?:\.\d{1,3}){3})', f.read())
    except (IOError, OSError):
        return None
    return m.group(1) if m else None


class Px4LedMuter(object):
    """Freeze the voxl_esc driver's LED bits at 0 so PX4 stops repainting the strip.

    Runs `cmd` once PX4 has been up for `settle_s` seconds, and again whenever
    voxl-px4's main PID changes (service restart). Never raises.
    """

    def __init__(self, cmd, settle_s=8.0, log=print):
        self.cmd = cmd
        self.settle_s = settle_s
        self.log = log
        self.muted_pid = None
        self.pid_seen = None
        self.pid_since = 0.0
        self.next_check = 0.0
        self.fail_count = 0

    @staticmethod
    def px4_pid():
        try:
            out = subprocess.check_output(['systemctl', 'show', '-p', 'MainPID', '--value', 'voxl-px4'],
                                          timeout=5).decode().strip()
            pid = int(out or '0')
            return pid if pid > 0 else None
        except Exception:
            return None

    def poll(self):
        now = time.time()
        if now < self.next_check:
            return
        self.next_check = now + 2.0
        pid = self.px4_pid()
        if pid != self.pid_seen:
            self.pid_seen = pid
            self.pid_since = now
            if pid is not None and pid != self.muted_pid:
                self.log('voxl-px4 (pid %d) detected, will mute its ESC LED bits in %.0f s' % (pid, self.settle_s))
            return
        if pid is None or pid == self.muted_pid or now - self.pid_since < self.settle_s:
            return
        try:
            r = subprocess.run(self.cmd, shell=True, timeout=15,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            ok = (r.returncode == 0)
            detail = r.stdout.decode(errors='replace').strip().splitlines()[-1:] if r.stdout else []
        except Exception as e:
            ok, detail = False, [str(e)]
        if ok:
            self.muted_pid = pid
            self.fail_count = 0
            self.log('PX4 ESC LED bits muted (%s)' % self.cmd)
        else:
            self.fail_count += 1
            self.pid_since = now          # retry after another settle period
            if self.fail_count <= 3 or self.fail_count % 30 == 0:
                self.log('WARN: PX4 LED mute failed (%s) %s — retrying' % (self.cmd, ' '.join(detail)))


class LedDaemon(object):
    def __init__(self, args, sink, log=print):
        self.args = args
        self.sink = sink
        self.log = log
        self.default_color = parse_color(args.default_color, args.brightness)
        self.color = self.default_color
        self.mode = 'solid'
        self.blink_hz = 2.0
        self.last_seq = -1
        self.last_ground = None
        self.fallen_back = False
        self.hb_seq = 0
        self.running = True
        self.start = time.time()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', args.port))
        self.sock.setblocking(False)
        self.muter = None
        if not args.no_px4_led_mute and not args.dry_run:
            self.muter = Px4LedMuter(args.px4_led_mute_cmd, args.px4_led_mute_settle_s, log)
        self.ground = (args.ground_ip, args.ground_port) if args.ground_ip else None
        if self.ground is None:
            log('WARN: no ground IP (not given and not found in %s) — LEDs stay at the '
                'default color; heartbeats disabled' % VOXL_PX4_START)

    # -- inbound ------------------------------------------------------------
    def handle_datagram(self, data, addr):
        try:
            msg = json.loads(data.decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            return
        seq = int(msg.get('seq', 0))
        # A new ground session (restart) starts a new seq series; accept it if
        # we have not heard from the ground for a while, else drop stale ones.
        stale_link = (self.last_ground is None
                      or time.time() - self.last_ground > self.args.fallback_s)
        if seq <= self.last_seq and not stale_link and seq != 0:
            return
        self.last_seq = seq
        self.last_ground = time.time()
        self.fallen_back = False
        prev = (self.color, self.mode)
        if 'r' in msg:
            self.color = tuple(max(0, min(255, int(msg.get(k, 0)))) for k in ('r', 'g', 'b', 'w'))
        mode = str(msg.get('mode', self.mode)).lower()
        self.mode = 'blink' if mode == 'blink' else 'solid'
        if (self.color, self.mode) != prev:
            self.log('color -> %s %s (seq %d from %s:%d)' % (list(self.color), self.mode, seq, addr[0], addr[1]))
        try:
            hz = float(msg.get('hz', self.blink_hz))
            self.blink_hz = hz if 0.2 <= hz <= 20.0 else self.blink_hz
        except (TypeError, ValueError):
            pass
        # Learn/refresh the ground address from the command itself as well.
        if self.ground is None or addr[0] != self.ground[0]:
            self.ground = (addr[0], self.args.ground_port)

    def drain_socket(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, socket.error):
                return
            self.handle_datagram(data, addr)

    # -- outbound -----------------------------------------------------------
    def heartbeat(self):
        if self.ground is None:
            return
        self.hb_seq += 1
        msg = {'name': self.args.name, 'seq': self.hb_seq,
               'uptime': round(time.time() - self.start, 1),
               'color': list(self.color), 'mode': self.mode}
        try:
            self.sock.sendto(json.dumps(msg).encode('utf-8'), self.ground)
        except socket.error as e:
            self.log('WARN: heartbeat send failed: %s' % e)

    def current_frame(self, now):
        color = self.color
        if self.mode == 'blink' and int(now * self.blink_hz * 2.0) % 2 == 1:
            color = (0, 0, 0, 0)
        return solid_frame(self.args.num_leds, color, not self.args.rgb)

    # -- main loop ----------------------------------------------------------
    def run(self):
        period = 1.0 / self.args.refresh_hz
        next_hb = 0.0
        self.log('svg_led_daemon %s: %d %s leds, default %s, listening udp/%d, ground %s'
                 % (self.args.name, self.args.num_leds, 'RGB' if self.args.rgb else 'RGBW',
                    self.args.default_color, self.args.port,
                    '%s:%d' % self.ground if self.ground else 'unknown'))
        while self.running:
            now = time.time()
            self.drain_socket()
            if (self.last_ground is not None and not self.fallen_back
                    and now - self.last_ground > self.args.fallback_s):
                self.log('no ground traffic for %.0f s — reverting to default color'
                         % self.args.fallback_s)
                self.color = self.default_color
                self.mode = 'solid'
                self.fallen_back = True
            if now >= next_hb:
                self.heartbeat()
                next_hb = now + 1.0
            if self.muter is not None:
                self.muter.poll()
            self.sink.send(self.current_frame(now), self.args.num_leds, not self.args.rgb)
            time.sleep(period)
        # shutdown: strip off
        off = solid_frame(self.args.num_leds, (0, 0, 0, 0), not self.args.rgb)
        for _ in range(3):
            if self.sink.send(off, self.args.num_leds, not self.args.rgb):
                break
            time.sleep(0.05)
        self.sink.close()
        self.sock.close()
        self.log('svg_led_daemon stopped, LEDs off')

    def stop(self, *_):
        self.running = False


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--name', required=True, help='drone name, e.g. drone_1 (sent in heartbeats)')
    p.add_argument('--ground-ip', default=None,
                   help='ground PC IP for heartbeats (default: -h flag in %s)' % VOXL_PX4_START)
    p.add_argument('--port', type=int, default=47900, help='UDP port to listen on (default 47900)')
    p.add_argument('--ground-port', type=int, default=47901,
                   help='ground led_controller heartbeat port (default 47901)')
    p.add_argument('--num-leds', type=int, default=11, help='pixels on the strip (default 11)')
    p.add_argument('--rgb', action='store_true', help='RGB strip (default: RGBW, 4 bytes/pixel)')
    p.add_argument('--esc-id', type=int, default=ESC_ID_DEFAULT,
                   help='ESC id byte in the LED packet: 0 = ESC 0 (RGB_OUT pin; ModalAI tool '
                        'default), 255 = all ESCs (modal_io.c). Default %d' % ESC_ID_DEFAULT)
    p.add_argument('--default-color', default='green',
                   help='color at boot and after ground-link loss (default green)')
    p.add_argument('--brightness', type=int, default=80,
                   help='0-255 scale applied to the default color (default 80; keep low for mocap)')
    p.add_argument('--refresh-hz', type=float, default=10.0, help='frame re-send rate (default 10)')
    p.add_argument('--fallback-s', type=float, default=10.0,
                   help='seconds without ground traffic before reverting to default (default 10)')
    p.add_argument('--use-libmodalio', action='store_true',
                   help='force libmodalio.so (default: used automatically when present — it '
                        'falls back to a MAVLink tunnel via voxl-mavlink-server on voxl-px4 '
                        'builds without the modal_io_bridge FIFO)')
    p.add_argument('--no-libmodalio', action='store_true',
                   help='never use libmodalio.so; write the modal_io_bridge FIFO directly')
    p.add_argument('--libmodalio-path', default=None,
                   help='path to libmodalio.so (default: /usr/lib64 or /usr/lib)')
    p.add_argument('--sink', choices=['auto', 'fifo', 'tunnel'], default='auto',
                   help='auto (default): modal_io_bridge FIFO if it exists, else the MAVLink '
                        'tunnel through voxl-mavlink-server; or force one')
    p.add_argument('--tunnel-pipe', default=MAVLINK_CONTROL_PIPE,
                   help='voxl-mavlink-server control pipe (default %s)' % MAVLINK_CONTROL_PIPE)
    p.add_argument('--tunnel-payload-type', type=int, default=MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_ESC_UART_PASSTHRU,
                   help='MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_ESC_UART_PASSTHRU value of this SDK '
                        '(default %d; voxl_setup_led.sh reads it from the mavlink headers)'
                        % MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_ESC_UART_PASSTHRU)
    p.add_argument('--tunnel-crc-extra', type=int, default=MAVLINK_MSG_ID_TUNNEL_CRC,
                   help='MAVLINK_MSG_ID_TUNNEL_CRC (default %d)' % MAVLINK_MSG_ID_TUNNEL_CRC)
    p.add_argument('--dry-run', action='store_true', help='print frames instead of driving LEDs')
    p.add_argument('--px4-led-mute-cmd', default='px4-qshell voxl_esc -l 0 led',
                   help='command that freezes the voxl_esc driver LED bits (run at start and after '
                        'each voxl-px4 restart); default "%(default)s"')
    p.add_argument('--px4-led-mute-settle-s', type=float, default=8.0,
                   help='seconds after voxl-px4 (re)starts before sending the mute command (default 8)')
    p.add_argument('--no-px4-led-mute', action='store_true',
                   help='do not touch the PX4 ESC driver LED bits (strip will flicker under PX4)')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    def log(msg):
        sys.stdout.write('[%s] %s\n' % (time.strftime('%H:%M:%S'), msg))
        sys.stdout.flush()

    if args.ground_ip is None:
        args.ground_ip = ground_ip_from_px4_start()
        if args.ground_ip:
            log('ground IP %s (from %s)' % (args.ground_ip, VOXL_PX4_START))
    if args.dry_run:
        sink = DryRunSink(log)
    else:
        sink = None
        lib_candidates = ([args.libmodalio_path] if args.libmodalio_path
                          else ['/usr/lib64/libmodalio.so', '/usr/lib/libmodalio.so'])
        lib_path = next((c for c in lib_candidates if c and os.path.exists(c)), None)
        if args.use_libmodalio and lib_path is None:
            log('ERROR: --use-libmodalio given but none of %s exist' % lib_candidates)
            return 2
        if lib_path and not args.no_libmodalio:
            # Preferred: ModalAI's own library tries the modal_io_bridge FIFO first and
            # otherwise tunnels the packet through voxl-mavlink-server, which is the only
            # path on voxl-px4 builds that never create /run/mpa/modal_io_bridge.
            try:
                sink = LibModalIoSink(lib_path, log=log)
            except (OSError, AttributeError) as e:
                log('WARN: %s unusable (%s) — falling back to the FIFO writer' % (lib_path, e))
        if sink is None:
            use_tunnel = (args.sink == 'tunnel'
                          or (args.sink == 'auto' and not os.path.exists(MODAL_IO_BRIDGE_SINK)))
            if use_tunnel:
                if args.sink == 'auto':
                    log('%s absent on this voxl-px4 build -> MAVLink tunnel via %s'
                        % (MODAL_IO_BRIDGE_SINK, args.tunnel_pipe))
                sink = MavlinkTunnelSink(args.tunnel_pipe, args.tunnel_payload_type,
                                         args.tunnel_crc_extra, log=log, esc_id=args.esc_id)
            else:
                sink = PipeSink(log=log, esc_id=args.esc_id)

    daemon = LedDaemon(args, sink, log)
    signal.signal(signal.SIGINT, daemon.stop)
    signal.signal(signal.SIGTERM, daemon.stop)
    daemon.run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
