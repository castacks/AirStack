#!/usr/bin/env python3

"""
NatNet Protocol Client (Pure Python)

Decodes NatNet 4.4 protocol UDP packets from OptiTrack Motive software.
Supports rigid body tracking via pure Python parsing (no SDK required).

Reference: OptiTrack NatNet 4.4 Documentation
https://v20.wiki.optitrack.com/index.php?title=NatNet
"""

import struct
import socket
import threading
import time
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass
from enum import IntEnum


class MessageType(IntEnum):
    """NatNet command-channel / packet IDs (NatNetTypes.h)."""

    NAT_CONNECT = 0
    NAT_SERVERINFO = 1
    NAT_REQUEST = 2
    NAT_RESPONSE = 3
    NAT_REQUEST_MODELDEF = 4
    NAT_MODELDEF = 5
    NAT_REQUEST_FRAMEOFDATA = 6
    FRAME_OF_DATA = 7  # NAT_FRAMEOFDATA (mocap data stream)
    NAT_MESSAGESTRING = 8
    NAT_UNRECOGNIZED_REQUEST = 100


@dataclass
class RigidBodyData:
    """Rigid body pose and kinematic data"""
    id: int
    position: Tuple[float, float, float]  # (x, y, z)
    rotation: Tuple[float, float, float, float]  # (qx, qy, qz, qw)
    markers: List[Tuple[float, float, float]]  # List of marker positions
    mean_marker_error: float
    tracking_valid: bool
    timestamp: float


@dataclass
class FrameData:
    """Complete frame of motion capture data"""
    frame_number: int
    timestamp: float
    rigid_bodies: Dict[int, RigidBodyData]


class NatNetClient:
    """
    Pure Python NatNet protocol client for OptiTrack Motive.
    
    Receives NatNet UDP packets and decodes motion capture frames.
    Supports rigid body tracking (basic motion capture data).
    
    Features:
    - Version negotiation: Requests NatNet version from Motive
    - Fallback support: Tries NatNet 4.4 → 4.3 → 4.2 → 4.1 → 4.0
    - Robust parsing: Handles optional fields and frame variations

    Usage:
        client = NatNetClient("192.168.1.1", 1511)  # Motive IP, local data UDP port
        client.set_frame_callback(my_callback)
        client.start()
        # ... receives frames in background thread
        client.stop()
    """

    def __init__(
        self,
        server_ip: str,
        data_port: int,
        *,
        command_port: int = 1510,
        local_ip: str = '0.0.0.0',
        negotiation_enabled: bool = True,
    ):
        """
        Initialize NatNet client.

        Args:
            server_ip: IP address of the Motive PC (command channel + unicast source).
            data_port: UDP port on **this machine** to bind for the NatNet **data** stream
                (default 1511). Not the Motive hostname; use ``server_ip`` for that.
            command_port: UDP port for NatNet commands on Motive (default 1510)
            local_ip: Address of the local NIC to bind for command + data sockets.
                Use ``0.0.0.0`` to let the OS choose (fine with a single interface).
                Set to your LAN IP (e.g. Jetson on OptiTrack subnet) when multiple
                interfaces exist so outbound packets to Motive use the correct
                source address for unicast streaming (NatNet SDK ``localAddress``).
            negotiation_enabled: If False, skip UDP command-channel handshake
                (``NAT_CONNECT`` / ``SetNatNetVersion``). Use when Motive never replies
                on ``command_port`` (firewall, Docker bridge, etc.) but mocap data still
                arrives on ``data_port``.
        """
        self.server_ip = server_ip
        self.data_port = data_port
        self.command_port = command_port
        self.local_ip = local_ip
        self.negotiation_enabled = negotiation_enabled
        self.sock = None
        self.running = False
        self.receive_thread = None
        self._user_callback: Optional[Callable[[FrameData], None]] = None
        self.negotiated_version = None
        
        # Versions to try in order (4.4 → 4.3 → ... → 4.0)
        self.supported_versions = [
            (4, 4, 0, 0),
            (4, 3, 0, 0),
            (4, 2, 0, 0),
            (4, 1, 0, 0),
            (4, 0, 0, 0),
        ]

    def set_frame_callback(self, callback: Callable[[FrameData], None]):
        """
        Register callback to receive decoded frames.
        
        Args:
            callback: Function(FrameData) called when frame is received
        """
        self._user_callback = callback

    @staticmethod
    def _pack_command_packet(message_id: int, payload: bytes = b'') -> bytes:
        """NatNet sPacket: uint16 iMessage, uint16 nDataBytes, payload."""
        return struct.pack('<HH', message_id, len(payload)) + payload

    @classmethod
    def _print_outgoing_command_packet(cls, packet: bytes, label: str) -> None:
        """Decode and print NatNet command packet layout before send (debug)."""
        if len(packet) < 4:
            print(
                f"[NatNetClient] negotiate TX {label}: INVALID len={len(packet)} "
                f"hex={packet.hex()}"
            )
            return
        msg_id, n_data = struct.unpack('<HH', packet[0:4])
        try:
            msg_name = MessageType(msg_id).name
        except ValueError:
            msg_name = f'UNKNOWN({msg_id})'
        payload = packet[4:]
        print(
            f"[NatNetClient] negotiate TX {label}: "
            f"iMessage={msg_id} ({msg_name}) nDataBytes={n_data} "
            f"wire_total={len(packet)} hex={packet.hex()}"
        )
        if payload:
            if payload.endswith(b'\x00'):
                body = payload[:-1].decode('ascii', errors='replace')
                print(
                    f"[NatNetClient] negotiate TX {label}: "
                    f'payload_nul_terminated_ascii="{body}"'
                )
            else:
                print(
                    f"[NatNetClient] negotiate TX {label}: "
                    f'payload_bytes={payload!r}'
                )

    @classmethod
    def _message_type_name(cls, msg_id: int) -> str:
        try:
            return MessageType(msg_id).name
        except ValueError:
            return f'UNKNOWN({msg_id})'

    @classmethod
    def _log_incoming_command_packet(cls, data: bytes, addr: Tuple[str, int]) -> None:
        """Log one datagram received on the NatNet command socket during negotiation."""
        if len(data) < 4:
            print(
                f"[NatNetClient] negotiate RX from {addr[0]}:{addr[1]}: "
                f'too_short len={len(data)} hex={data.hex()}'
            )
            return
        msg_id, n_payload = struct.unpack('<HH', data[0:4])
        body = data[4 : 4 + n_payload]
        preview = data[: min(64, len(data))].hex()
        print(
            f"[NatNetClient] negotiate RX from {addr[0]}:{addr[1]}: "
            f"iMessage={msg_id} ({cls._message_type_name(msg_id)}) "
            f"nDataBytes={n_payload} wire_len={len(data)} hex_preview={preview}"
        )
        if msg_id == MessageType.NAT_RESPONSE and body:
            if len(body) >= 4:
                rc = struct.unpack('<i', body[0:4])[0]
                print(f"[NatNetClient] negotiate RX NAT_RESPONSE rc_int32={rc}")
            else:
                print(
                    "[NatNetClient] negotiate RX NAT_RESPONSE "
                    f'payload_ascii={body.decode("ascii", errors="replace")!r}'
                )

    @staticmethod
    def _parse_natnet_version_from_server_info(packet: bytes) -> Optional[Tuple[int, int, int, int]]:
        """
        Parse NatNetVersion[4] from NAT_SERVERINFO (message id 1).

        Motive may send either:

        - **Fixed C struct** (common): ``szName[256]``, then App ``Version[4]``, then
          ``NatNetVersion[4]`` at offset **260** from the start of the payload — requires
          ``len(payload) >= 264`` (NatNetTypes.h ``#pragma pack(1)``).
        - **Tight packing**: null-terminated ``szName``, then versions — typical when the
          packet is **smaller** than the full padded struct.
        """
        payload = packet[4:]
        if len(payload) < 9:
            return None

        if len(payload) >= 264:
            nv = payload[260:264]
            return (int(nv[0]), int(nv[1]), int(nv[2]), int(nv[3]))

        z = payload.find(b'\x00')
        if z >= 0:
            o = z + 1
            if o + 8 <= len(payload):
                nv = payload[o + 4 : o + 8]
                return (int(nv[0]), int(nv[1]), int(nv[2]), int(nv[3]))

        return None

    def _recv_command_until(
        self,
        sock: socket.socket,
        *,
        end_mono: float,
        accept_message_ids: Tuple[int, ...],
        log_all_rx: bool = False,
    ) -> Optional[bytes]:
        """Drain UDP replies until one matches ``accept_message_ids`` or timeout."""
        while time.monotonic() < end_mono:
            sock.settimeout(max(0.001, end_mono - time.monotonic()))
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if hasattr(self, '_negotiation_recv_count'):
                self._negotiation_recv_count += 1
            if log_all_rx:
                try:
                    self._log_incoming_command_packet(data, addr)
                except Exception:
                    pass
            if len(data) < 4:
                continue
            msg_id = struct.unpack('<H', data[0:2])[0]
            if msg_id in accept_message_ids:
                return data
        return None

    def _negotiate_version(self) -> bool:
        """
        Align with Motive's NatNet command protocol (UDP command port, default 1510):

        1. Send NAT_CONNECT; read NAT_SERVERINFO to learn the server's NatNet version.
        2. For **unicast** bitstream selection (Motive 3.0+), send NAT_REQUEST with the
           remote command ``SetNatNetVersion,<major>,<minor>,<build>,<revision>`` until
           NAT_RESPONSE indicates success (SDK / PacketClient pattern).

        Reference: OptiTrack NatNetTypes.h (message IDs), PacketClient.cpp, NatNet 4.x docs.
        """
        command_sock = None

        try:
            command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            command_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            per_phase_timeout = 3.0
            self._negotiation_recv_count = 0
            if self.local_ip and self.local_ip != '0.0.0.0':
                command_sock.bind((self.local_ip, 0))

            server_from_info: Optional[Tuple[int, int, int, int]] = None

            # --- Phase 1: NAT_CONNECT → NAT_SERVERINFO ---
            connect_pkt = self._pack_command_packet(MessageType.NAT_CONNECT, b'')
            self._print_outgoing_command_packet(
                connect_pkt,
                f'NAT_CONNECT → {self.server_ip}:{self.command_port}',
            )
            try:
                for attempt in range(3):
                    command_sock.sendto(
                        connect_pkt,
                        (self.server_ip, self.command_port),
                    )
                    if attempt < 2:
                        time.sleep(0.05)
                t_end = time.monotonic() + per_phase_timeout
                info = self._recv_command_until(
                    command_sock,
                    end_mono=t_end,
                    accept_message_ids=(MessageType.NAT_SERVERINFO,),
                    log_all_rx=True,
                )
                if info:
                    server_from_info = self._parse_natnet_version_from_server_info(
                        info)
                    if server_from_info:
                        print(
                            "[NatNetClient] Motive NatNet version (from SERVERINFO): "
                            f"{server_from_info[0]}.{server_from_info[1]}."
                            f"{server_from_info[2]}.{server_from_info[3]}"
                        )
                    else:
                        pay_len = struct.unpack('<H', info[2:4])[0]
                        print(
                            "[NatNetClient] NAT_SERVERINFO received but could not parse "
                            f"NatNetVersion (payload_bytes={pay_len}). "
                            "Check negotiate RX hex_preview vs Motive / firewall."
                        )
                else:
                    n_rx = getattr(self, '_negotiation_recv_count', 0)
                    print(
                        "[NatNetClient] No NAT_SERVERINFO after NAT_CONNECT "
                        f"(timeout {per_phase_timeout}s). "
                        f"command_socket_recv_datagrams={n_rx}. "
                        "If this is 0, no UDP replies reached this host on the "
                        "ephemeral command socket (Windows Firewall on Motive for "
                        "UDP 1510, Docker without host networking, or wrong route). "
                        "Unicast data on 1511 may still arrive without command replies."
                    )
            except OSError as e:
                print(f"[NatNetClient] NAT_CONNECT send failed: {e}")

            # --- Phase 2: SetNatNetVersion via NAT_REQUEST (remote command string) ---
            for version_tuple in self.supported_versions:
                major, minor, build, revision = version_tuple
                cmd_str = (
                    f"SetNatNetVersion,{major},{minor},{build},{revision}"
                )
                payload = cmd_str.encode('ascii') + b'\x00'
                req_pkt = self._pack_command_packet(MessageType.NAT_REQUEST, payload)
                self._print_outgoing_command_packet(
                    req_pkt,
                    f'NAT_REQUEST SetNatNetVersion → {self.server_ip}:{self.command_port}',
                )

                try:
                    command_sock.sendto(
                        req_pkt, (self.server_ip, self.command_port))
                except OSError as e:
                    print(f"[NatNetClient] SetNatNetVersion send failed: {e}")
                    continue

                t_end = time.monotonic() + 2.0
                while time.monotonic() < t_end:
                    resp = self._recv_command_until(
                        command_sock,
                        end_mono=t_end,
                        accept_message_ids=(
                            MessageType.NAT_RESPONSE,
                            MessageType.NAT_UNRECOGNIZED_REQUEST,
                        ),
                        log_all_rx=True,
                    )
                    if resp is None:
                        break
                    msg_id = struct.unpack('<H', resp[0:2])[0]
                    n_payload = struct.unpack('<H', resp[2:4])[0]
                    body = resp[4 : 4 + n_payload]

                    if msg_id == MessageType.NAT_UNRECOGNIZED_REQUEST:
                        print(
                            "[NatNetClient] Motive rejected SetNatNetVersion "
                            f"{major}.{minor} (unrecognized request)."
                        )
                        break

                    # NAT_RESPONSE: 4-byte int 0 == OK (WinForms / SDK samples)
                    if len(body) >= 4:
                        rc = struct.unpack('<i', body[0:4])[0]
                        if rc == 0:
                            self.negotiated_version = version_tuple
                            print(
                                "[NatNetClient] Negotiated NatNet bitstream "
                                f"{major}.{minor}.{build}.{revision} "
                                "(SetNatNetVersion OK)."
                            )
                            return True
                        break
                    # Some builds return a short string payload; treat as handled.
                    self.negotiated_version = version_tuple
                    print(
                        "[NatNetClient] Negotiated NatNet bitstream "
                        f"{major}.{minor}.{build}.{revision} "
                        f"(response: {body!r})."
                    )
                    return True

            # Fallback: use SERVERINFO version tuple if available
            if server_from_info:
                self.negotiated_version = server_from_info
                print(
                    "[NatNetClient] Using Motive NatNet version from SERVERINFO "
                    "(SetNatNetVersion not confirmed)."
                )
                return True

            print(
                "[NatNetClient] Warning: Could not negotiate NatNet version with Motive. "
                "Will attempt to parse frames anyway."
            )
            self.negotiated_version = self.supported_versions[0]
            return False

        except Exception as e:
            print(f"[NatNetClient] Version negotiation failed: {e}")
            self.negotiated_version = self.supported_versions[0]
            return False

        finally:
            if command_sock:
                command_sock.close()
            if hasattr(self, '_negotiation_recv_count'):
                delattr(self, '_negotiation_recv_count')

    def start(self) -> bool:
        """
        Start listening for NatNet UDP packets.
        
        First negotiates NatNet version with Motive PC, then binds to data port.
        
        Returns:
            True if socket bound successfully, False otherwise
        """
        # Negotiate NatNet version with Motive before starting data listener (optional).
        if self.negotiation_enabled:
            print("[NatNetClient] Attempting version negotiation with Motive PC...")
            self._negotiate_version()
        else:
            print(
                "[NatNetClient] Skipping command-channel negotiation "
                "(negotiation_enabled=false)."
            )
            self.negotiated_version = self.supported_versions[0]
        
        bind_ip = (
            self.local_ip
            if self.local_ip and self.local_ip != '0.0.0.0'
            else '0.0.0.0'
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, self.data_port))

        self.sock.setblocking(True)

        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()

        print(
            f"[NatNetClient] Listening for unicast mocap on {bind_ip}:{self.data_port} "
            f"(Motive {self.server_ip}; command UDP {self.command_port}; "
            f"NatNet parse {self.negotiated_version[0]}.{self.negotiated_version[1]})"
        )
        return True
    

    def stop(self):
        """Stop listening and clean up resources."""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        if self.sock:
            self.sock.close()
            self.sock = None
        print("[NatNetClient] Stopped")

    def _receive_loop(self):
        """Background thread that receives and processes UDP packets."""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(65535)
                frame = self.parse_frame(data)
                if frame and self._user_callback:
                    self._user_callback(frame)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[NatNetClient] Error receiving packet: {e}")

    def parse_frame(self, data: bytes) -> Optional[FrameData]:
        """
        Parse a NatNet frame from UDP packet data.
        
        NatNet FrameOfData (message id 7) core layout used here:

        - Header (4), frame number (4), marker sets, **unlabeled/other markers** (count + 12×count),
          rigid body count, rigid bodies (NatNet 3+ layout: no per-body marker blobs).

        Skeletons, labeled markers, timing, etc. follow rigid bodies and are ignored so long as
        rigid-body parsing stays aligned with ``negotiated_version`` (major/minor).
        
        Args:
            data: Raw UDP packet bytes from Motive
            
        Returns:
            FrameData if successful, None if parsing fails
        """
        try:
            if len(data) < 4:
                return None
            
            offset = 0
            
            # Parse packet header
            message_type, packet_size = struct.unpack('<HH', data[offset:offset+4])
            offset += 4
            
            # NAT_FRAMEOFDATA == 7 (NatNetTypes.h). 101 was an older mistake; accept both.
            if message_type not in (
                MessageType.FRAME_OF_DATA,
                101,
            ):
                return None
            
            # Parse frame number
            frame_number = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            
            # Skip marker sets (we don't need marker cloud data for basic tracking)
            num_marker_sets = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            for _ in range(num_marker_sets):
                offset = self._skip_marker_set(data, offset)

            # Unlabeled / "other" markers (vector3 each) — required skip before rigid bodies
            # (matches PacketClient + python_natnet MocapFrameMessage.deserialize).
            if offset + 4 > len(data):
                return None
            num_other_markers = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            skip_other = num_other_markers * 12
            if offset + skip_other > len(data):
                return None
            offset += skip_other

            major = self.negotiated_version[0] if self.negotiated_version else 4
            minor = self.negotiated_version[1] if self.negotiated_version else 4

            # Parse rigid bodies
            if offset + 4 > len(data):
                return None
            num_rigid_bodies = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            rigid_bodies = {}
            for _ in range(num_rigid_bodies):
                rb_data, offset = self._parse_rigid_body(
                    data, offset, natnet_major=major, natnet_minor=minor,
                )
                if rb_data:
                    rigid_bodies[rb_data.id] = rb_data

            # Remaining packet holds skeletons, labeled markers, timing, etc.; we only need RBs.
            return FrameData(
                frame_number=frame_number,
                timestamp=float(frame_number),
                rigid_bodies=rigid_bodies,
            )
        
        except Exception as e:
            print(f"[NatNetClient] Error parsing frame: {e}")
            return None

    def _skip_marker_set(self, data: bytes, offset: int) -> int:
        """Skip over marker set data in frame (we don't need it for rigid body tracking)."""
        try:
            # Marker set name (null-terminated string)
            while offset < len(data) and data[offset] != 0:
                offset += 1
            offset += 1  # Skip null terminator
            
            # Number of markers in this set
            if offset + 4 > len(data):
                return offset
            num_markers = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            
            # Skip marker positions (each is 3 floats = 12 bytes)
            offset += num_markers * 12
            
            return offset
        except:
            return offset

    def _parse_rigid_body(
        self,
        data: bytes,
        offset: int,
        *,
        natnet_major: int,
        natnet_minor: int,
    ) -> Tuple[Optional[RigidBodyData], int]:
        """
        Parse one rigid body from a FrameOfData payload.

        NatNet 3.0+ omits per-body marker positions; use labeled markers if needed.

        NatNet 2.x (major < 3) embeds marker positions/IDs/sizes before mean error.
        Tracking validity uses ``params & 0x01`` when NatNet >= 2.6 or Motive 3+.

        Returns:
            (RigidBodyData, new_offset) or (None, original_offset) if parsing fails
        """
        try:
            start_offset = offset

            if offset + 4 > len(data):
                return None, offset
            rb_id = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            if offset + 28 > len(data):
                return None, start_offset
            pos_x, pos_y, pos_z = struct.unpack('<fff', data[offset:offset+12])
            offset += 12
            qx, qy, qz, qw = struct.unpack('<ffff', data[offset:offset+16])
            offset += 16

            markers: List[Tuple[float, float, float]] = []

            if natnet_major < 3:
                if offset + 4 > len(data):
                    return None, start_offset
                num_markers = struct.unpack('<I', data[offset:offset+4])[0]
                offset += 4
                if offset + num_markers * 12 > len(data):
                    return None, start_offset
                for _ in range(num_markers):
                    mx, my, mz = struct.unpack('<fff', data[offset:offset+12])
                    markers.append((mx, my, mz))
                    offset += 12
                if natnet_major >= 2:
                    id_bytes = num_markers * 4
                    sz_bytes = num_markers * 4
                    if offset + id_bytes + sz_bytes > len(data):
                        return None, start_offset
                    offset += id_bytes + sz_bytes

            mean_error = 0.0
            if natnet_major >= 2:
                if offset + 4 > len(data):
                    return None, start_offset
                mean_error = struct.unpack('<f', data[offset:offset+4])[0]
                offset += 4

            tracking_valid = True
            if (natnet_major == 2 and natnet_minor >= 6) or natnet_major > 2:
                if offset + 2 > len(data):
                    return None, start_offset
                params = struct.unpack('<h', data[offset:offset+2])[0]
                offset += 2
                tracking_valid = (params & 0x01) != 0

            rb_data = RigidBodyData(
                id=rb_id,
                position=(pos_x, pos_y, pos_z),
                rotation=(qx, qy, qz, qw),
                markers=markers,
                mean_marker_error=mean_error,
                tracking_valid=tracking_valid,
                timestamp=0.0,
            )

            return rb_data, offset

        except Exception as e:
            print(f"[NatNetClient] Error parsing rigid body: {e}")
            return None, start_offset


if __name__ == "__main__":
    print("NatNet protocol client (pure Python) for OptiTrack Motive integration")
    print("\nUsage:")
    print("  client = NatNetClient('192.168.1.1', 1511)")
    print("  client.set_frame_callback(my_callback)")
    print("  client.start()")
