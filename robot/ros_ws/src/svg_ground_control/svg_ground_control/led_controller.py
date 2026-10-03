"""Ground-side LED controller: colors the real drones' onboard LED strips.

Talks UDP to the svg_led_daemon.py running on each VOXL (scripts/svg_led_daemon.py,
installed by scripts/voxl_setup_led.sh). No IP configuration: every daemon sends a
1 Hz heartbeat {"name": "drone_N", ...} to this node's UDP port (default 47901)
and the node replies to that source address with color commands.

Color policy (per drone):
  base color   default 'green' (param default_color); changed by the services
  CBF override 'red' (param cbf_color) while the swarm_commander reports the drone
               on /svg/cbf_active (std_msgs/String, comma-separated names, every
               control tick). Held for at least cbf_hold_s after the last report
               so brief corrections are visible; ignored if the last report is
               older than cbf_stale_s (dead commander cannot leave a drone red).

Two ways to change a drone's base color — both work at ANY time the node is up
(disarmed, on the bench, mid-flight); the onboard daemon shows the last color it got:

Topic trigger (like /svg/formation_command), std_msgs/String:
  /svg/led_command            "<drone|all> <color> [blink]"
  /svg/<name>/led_command     "<color> [blink]"

    ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'drone_1 blue'}"
    ros2 topic pub --once /svg/led_command std_msgs/msg/String "{data: 'all green'}"
    ros2 topic pub --once /svg/drone_2/led_command std_msgs/msg/String "{data: 'red blink'}"

Services (airstack_msgs/srv/SetLedColor: color = name or "r,g,b[,w]", mode 0 solid / 1 blink):
  /svg/<name>/set_led_color   one drone's base color
  /svg/set_led_color          all configured drones

    ros2 service call /svg/drone_1/set_led_color airstack_msgs/srv/SetLedColor "{color: blue}"
    ros2 service call /svg/set_led_color airstack_msgs/srv/SetLedColor "{color: '255,60,0', mode: 1}"

Parameters (own `led_controller:` block in the config YAML):
  drone_names     drones with an LED daemon (the REAL drones)
  heartbeat_port  UDP port to listen on (daemons send here)         47901
  led_hosts       optional static "drone_1=ip[:port],..." fallback  ""
  default_color   base color at start                                "green"
  cbf_color       color while the CBF corrects a drone               "red"
  cbf_hold_s / cbf_stale_s / resend_hz / brightness (0-255 scale applied to every color)
"""
import json
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from airstack_msgs.srv import SetLedColor

# Keep in sync with scripts/svg_led_daemon.py (the daemon must stay stdlib-only,
# so the table is duplicated rather than imported).
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
DEFAULT_DAEMON_PORT = 47900


def parse_color(text, brightness=255):
    """'green' or 'r,g,b[,w]' -> (r,g,b,w) scaled by brightness/255. Raises ValueError."""
    text = str(text).strip().lower()
    if text in COLOR_NAMES:
        base = COLOR_NAMES[text]
    else:
        parts = [p.strip() for p in text.split(',')]
        if len(parts) not in (3, 4):
            raise ValueError(f'unknown color {text!r} (use a name or r,g,b[,w])')
        vals = [int(p) for p in parts]
        if any(v < 0 or v > 255 for v in vals):
            raise ValueError('color channels must be 0-255')
        base = tuple(vals) + ((0,) if len(vals) == 3 else ())
    scale = max(0, min(255, int(brightness))) / 255.0
    return tuple(int(round(c * scale)) for c in base)


class LedState:
    def __init__(self, name, base):
        self.name = name
        self.base = base            # (r,g,b,w)
        self.mode = 'solid'
        self.addr = None            # (ip, port) learned from heartbeat / static
        self.last_seen = None       # wall time of last heartbeat
        self.online_logged = False
        self.cbf_until = 0.0        # wall time until which the CBF color is shown
        self.last_sent = None       # (color, mode) last commanded
        self.last_send_time = 0.0


class LedController(Node):
    def __init__(self):
        super().__init__('led_controller')
        self.declare_parameter('drone_names', ['drone_1'])
        self.declare_parameter('heartbeat_port', 47901)
        self.declare_parameter('led_hosts', '')
        self.declare_parameter('default_color', 'green')
        self.declare_parameter('cbf_color', 'red')
        self.declare_parameter('cbf_active_topic', '/svg/cbf_active')
        self.declare_parameter('cbf_hold_s', 0.5)
        self.declare_parameter('cbf_stale_s', 1.0)
        self.declare_parameter('resend_hz', 2.0)
        self.declare_parameter('brightness', 80)
        self.declare_parameter('blink_hz', 3.0)

        names = [str(n) for n in self.get_parameter('drone_names').value]
        self.brightness = int(self.get_parameter('brightness').value)
        self.cbf_hold_s = float(self.get_parameter('cbf_hold_s').value)
        self.cbf_stale_s = float(self.get_parameter('cbf_stale_s').value)
        self.blink_hz = float(self.get_parameter('blink_hz').value)
        default = parse_color(self.get_parameter('default_color').value, self.brightness)
        self.cbf_rgbw = parse_color(self.get_parameter('cbf_color').value, self.brightness)

        self.drones = {n: LedState(n, default) for n in names}
        self.last_cbf_msg = None
        self.seq = int(time.time() * 1000)   # strictly increasing across restarts

        # Optional static hosts: "drone_1=192.168.50.11,drone_2=192.168.50.12:47900"
        for entry in str(self.get_parameter('led_hosts').value).split(','):
            entry = entry.strip()
            if not entry or '=' not in entry:
                continue
            name, host = entry.split('=', 1)
            ip, _, port = host.partition(':')
            if name.strip() in self.drones:
                self.drones[name.strip()].addr = (ip.strip(), int(port) if port else DEFAULT_DAEMON_PORT)
                self.get_logger().info(f'{name.strip()}: static LED host {ip.strip()}')

        port = int(self.get_parameter('heartbeat_port').value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Deliberately NO SO_REUSEADDR: two controllers on one port would each
        # see only some heartbeats and fight over the strips (one pushing its
        # default green, the other the commanded color -> "blinking" drones).
        try:
            self.sock.bind(('0.0.0.0', port))
        except OSError as e:
            raise SystemExit(
                f'led_controller: cannot bind udp/{port} ({e.strerror}). Another led_controller '
                f'is already running (a previous ground_control.launch or `ros2 run`) — stop it '
                f'first: pgrep -af led_controller') from e
        self.sock.setblocking(False)

        self.create_subscription(
            String, str(self.get_parameter('cbf_active_topic').value), self.cbf_callback, 10)
        for n in names:
            self.create_service(SetLedColor, f'/svg/{n}/set_led_color',
                                lambda req, res, name=n: self.handle_set(req, res, [name]))
            self.create_subscription(String, f'/svg/{n}/led_command',
                                     lambda msg, name=n: self.handle_command(msg.data, [name]), 10)
        self.create_service(SetLedColor, '/svg/set_led_color',
                            lambda req, res: self.handle_set(req, res, names))
        self.create_subscription(String, '/svg/led_command', self.led_command_callback, 10)

        self.create_timer(0.02, self.poll_socket)
        self.create_timer(1.0 / max(0.1, float(self.get_parameter('resend_hz').value)), self.resend)
        self.create_timer(1.0, self.housekeeping)
        self.start_time = time.time()
        self.get_logger().info(
            f'led_controller up: drones {names}, heartbeat udp/{port}, '
            f'default {self.get_parameter("default_color").value}, '
            f'CBF -> {self.get_parameter("cbf_color").value} (hold {self.cbf_hold_s}s), '
            f'brightness {self.brightness}/255')

    # ---- inbound ---------------------------------------------------------
    def poll_socket(self):
        now = time.time()
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, OSError):
                return
            try:
                msg = json.loads(data.decode('utf-8'))
            except (ValueError, UnicodeDecodeError):
                continue
            name = str(msg.get('name', ''))
            d = self.drones.get(name)
            if d is None:
                self.get_logger().warn(
                    f'heartbeat from unconfigured drone {name!r} at {addr[0]} — '
                    f'add it to led_controller.drone_names', throttle_duration_sec=10.0)
                continue
            new_addr = (addr[0], addr[1])
            if d.addr != new_addr:
                d.addr = new_addr
                d.last_sent = None            # force an immediate command to the new address
            d.last_seen = now
            if not d.online_logged:
                self.get_logger().info(f'{name}: LED daemon online at {addr[0]}:{addr[1]} '
                                       f'(reports {msg.get("color")} {msg.get("mode")})')
                d.online_logged = True
        # unreachable

    def cbf_callback(self, msg: String):
        now = time.time()
        self.last_cbf_msg = now
        active = {n.strip() for n in msg.data.split(',') if n.strip()}
        changed = False
        for name in active:
            d = self.drones.get(name)
            if d is not None:
                if now >= d.cbf_until:
                    changed = True
                d.cbf_until = now + self.cbf_hold_s
        if changed:
            self.push_all(now)

    # ---- color commands (service + topic share this) ----------------------
    def apply_color(self, names, color_text, mode):
        """Set the base color of `names`. Returns (ok, message)."""
        try:
            rgbw = parse_color(color_text, self.brightness)
        except ValueError as e:
            return False, str(e)
        for n in names:
            d = self.drones[n]
            d.base = rgbw
            d.mode = mode
        self.push_all(time.time())
        offline = [n for n in names if self.drones[n].addr is None]
        msg = (f'{", ".join(names)} -> {color_text} ({mode})'
               + (f'; not yet online: {", ".join(offline)}' if offline else ''))
        self.get_logger().info(msg)
        return True, msg

    def handle_set(self, req, res, names):
        mode = 'blink' if req.mode == SetLedColor.Request.MODE_BLINK else 'solid'
        res.success, res.message = self.apply_color(names, req.color, mode)
        return res

    def handle_command(self, text, names):
        """'<color> [blink|solid]' for the given drones (per-drone topic)."""
        parts = text.strip().split()
        if not parts:
            return
        mode = 'blink' if len(parts) > 1 and parts[-1].lower() == 'blink' else 'solid'
        color = parts[0] if len(parts) == 1 or parts[-1].lower() in ('blink', 'solid') else ' '.join(parts)
        ok, msg = self.apply_color(names, color, mode)
        if not ok:
            self.get_logger().warn(f'led_command {text!r}: {msg}')

    def led_command_callback(self, msg: String):
        """'<drone|all> <color> [blink|solid]' on /svg/led_command."""
        parts = msg.data.strip().split(None, 1)
        if len(parts) < 2:
            self.get_logger().warn(f'led_command {msg.data!r}: expected "<drone|all> <color> [blink]"')
            return
        target, rest = parts[0], parts[1]
        if target.lower() == 'all':
            names = list(self.drones)
        elif target in self.drones:
            names = [target]
        else:
            self.get_logger().warn(
                f'led_command: unknown drone {target!r} (configured: {", ".join(self.drones)})')
            return
        self.handle_command(rest, names)

    # ---- outbound --------------------------------------------------------
    def effective(self, d, now):
        cbf_fresh = self.last_cbf_msg is not None and now - self.last_cbf_msg < self.cbf_stale_s
        if cbf_fresh and now < d.cbf_until:
            return self.cbf_rgbw, 'solid'
        return d.base, d.mode

    def send(self, d, color, mode):
        if d.addr is None:
            return
        self.seq = max(self.seq + 1, int(time.time() * 1000))
        r, g, b, w = color
        msg = {'seq': self.seq, 'r': r, 'g': g, 'b': b, 'w': w, 'mode': mode, 'hz': self.blink_hz}
        try:
            self.sock.sendto(json.dumps(msg).encode('utf-8'), d.addr)
        except OSError as e:
            self.get_logger().warn(f'{d.name}: LED send failed: {e}', throttle_duration_sec=5.0)
            return
        d.last_sent = (color, mode)
        d.last_send_time = time.time()

    def push_all(self, now):
        """Send to every drone whose effective color changed."""
        for d in self.drones.values():
            eff = self.effective(d, now)
            if eff != d.last_sent:
                self.send(d, *eff)

    def resend(self):
        now = time.time()
        for d in self.drones.values():
            eff = self.effective(d, now)
            if eff != d.last_sent or now - d.last_send_time > 0.5 / max(0.1, 1.0):
                self.send(d, *eff)

    def housekeeping(self):
        now = time.time()
        for d in self.drones.values():
            if d.addr is None and now - self.start_time > 5.0:
                self.get_logger().warn(
                    f'{d.name}: no LED heartbeat yet — is svg-led running on the VOXL, '
                    f'does its ground IP point here, is udp/{self.get_parameter("heartbeat_port").value} '
                    f'open in the firewall?', throttle_duration_sec=30.0)
            elif d.last_seen is not None and now - d.last_seen > 5.0 and d.online_logged:
                self.get_logger().warn(f'{d.name}: LED heartbeat lost ({now - d.last_seen:.0f}s)',
                                       throttle_duration_sec=30.0)
                d.online_logged = False
        # A CBF hold expiring with no new message: push the base color back.
        self.push_all(now)

    def destroy_node(self):
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LedController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as e:
        print(e, file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
