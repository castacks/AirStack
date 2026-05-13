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
    """NatNet message types"""
    FRAME_OF_DATA = 101
    MODELDEF = 102
    REQUEST_MODELDEF = 103
    SENDMODE_MULTICAST = 200
    SENDMODE_UNICAST = 201
    NAT_REQUEST = 0
    NAT_RESPONSE = 1
    NAT_REQUEST_REQUEST = 3
    NAT_REQUEST_MODELDEF = 4


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
        client = NatNetClient("192.168.1.1", 1511)
        client.set_frame_callback(my_callback)
        client.start()
        # ... receives frames in background thread
        client.stop()
    """

    def __init__(self, server_ip: str, server_port: int):
        """
        Initialize NatNet client.
        
        Args:
            server_ip: IP address of Motive PC
            server_port: Local UDP port to listen on (default 1511)
        """
        self.server_ip = server_ip
        self.server_port = server_port
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

    def _negotiate_version(self) -> bool:
        """
        Negotiate NatNet version with Motive PC via command port (1510).
        
        Sends SetNatNetVersion request for each supported version in order.
        Stores the negotiated version in self.negotiated_version.
        
        Returns:
            True if version successfully negotiated, False if all attempts failed
        """
        command_port = 1510
        command_sock = None
        
        try:
            # Create UDP socket for command communication
            command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            command_sock.settimeout(2.0)  # 2 second timeout per attempt
            
            # Try each supported version in order (4.4 → 4.3 → ... → 4.0)
            for version_tuple in self.supported_versions:
                major, minor, build, revision = version_tuple
                
                try:
                    # Build SetNatNetVersion command packet
                    # Format: message_type (2B) + packet_size (2B) + version (4B each)
                    message_type = 0  # SetNatNetVersion request
                    packet_size = 20  # Header + version data
                    
                    packet = struct.pack(
                        '<HHIIII',
                        message_type,      # uint16: message type
                        packet_size,       # uint16: packet size
                        major,             # uint32: major version
                        minor,             # uint32: minor version
                        build,             # uint32: build number
                        revision           # uint32: revision
                    )
                    
                    # Send version request to Motive command port
                    command_sock.sendto(packet, (self.server_ip, command_port))
                    
                    # Wait for response
                    try:
                        response, addr = command_sock.recvfrom(65535)
                        
                        # Parse response: message_type (2B) + packet_size (2B) + status (4B)
                        if len(response) >= 8:
                            resp_type, resp_size = struct.unpack('<HH', response[0:4])
                            
                            # Response type 1 indicates success
                            if resp_type == 1:  # NAT_RESPONSE
                                self.negotiated_version = version_tuple
                                print(f"[NatNetClient] Negotiated NatNet version {major}.{minor} "
                                      f"(build {build}.{revision})")
                                return True
                    
                    except socket.timeout:
                        # This version didn't respond, try next one
                        pass
                
                except Exception as e:
                    print(f"[NatNetClient] Error trying version {major}.{minor}: {e}")
                    continue
            
            # If we get here, no version negotiated successfully
            print("[NatNetClient] Warning: Could not negotiate NatNet version with Motive. "
                  "Will attempt to parse frames anyway.")
            # Set to highest version as fallback
            self.negotiated_version = self.supported_versions[0]
            return False
        
        except Exception as e:
            print(f"[NatNetClient] Version negotiation failed: {e}")
            return False
        
        finally:
            if command_sock:
                command_sock.close()

    def start(self) -> bool:
        """
        Start listening for NatNet UDP packets.
        
        First negotiates NatNet version with Motive PC, then binds to data port.
        
        Returns:
            True if socket bound successfully, False otherwise
        """
        try:
            # Attempt to negotiate version with Motive before starting data listener
            print("[NatNetClient] Attempting version negotiation with Motive PC...")
            self._negotiate_version()
            
            # Bind to data port for frame reception
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('0.0.0.0', self.server_port))
            self.sock.setblocking(True)
            
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            
            version_str = f"{self.negotiated_version[0]}.{self.negotiated_version[1]}"
            print(f"[NatNetClient] Listening on UDP port {self.server_port} "
                  f"(NatNet {version_str})")
            return True
        
        except Exception as e:
            print(f"[NatNetClient] Failed to start: {e}")
            return False

    def stop(self):
        """Stop listening and clean up resources."""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        if self.sock:
            self.sock.close()
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
        
        NatNet 4.4 frame structure:
        - Packet ID (2 bytes): Message type
        - Packet size (2 bytes): Total packet size
        - Frame number (4 bytes)
        - Marker set count (4 bytes) + [marker set data]
        - Rigid body count (4 bytes) + [rigid body data]
        - Latency (4 bytes)
        - Timestamp (8 bytes, optional)
        
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
            
            # Only process frame data packets
            if message_type != MessageType.FRAME_OF_DATA:
                return None
            
            # Parse frame number
            frame_number = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            
            # Skip marker sets (we don't need marker cloud data for basic tracking)
            num_marker_sets = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            for _ in range(num_marker_sets):
                offset = self._skip_marker_set(data, offset)
            
            # Parse rigid bodies
            num_rigid_bodies = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            
            rigid_bodies = {}
            for _ in range(num_rigid_bodies):
                rb_data, offset = self._parse_rigid_body(data, offset)
                if rb_data:
                    rigid_bodies[rb_data.id] = rb_data
            
            # Parse latency (in milliseconds)
            if offset + 4 <= len(data):
                latency = struct.unpack('<f', data[offset:offset+4])[0]
                offset += 4
            else:
                latency = 0.0
            
            # Parse timestamp (optional, present in newer versions)
            # For NatNet 4.4, timestamp is typically present
            timestamp = 0.0
            if offset + 8 <= len(data):
                timestamp = struct.unpack('<d', data[offset:offset+8])[0]
                offset += 8
            
            return FrameData(
                frame_number=frame_number,
                timestamp=timestamp if timestamp > 0 else latency,
                rigid_bodies=rigid_bodies
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

    def _parse_rigid_body(self, data: bytes, offset: int) -> Tuple[Optional[RigidBodyData], int]:
        """
        Parse a single rigid body from frame data.
        
        Returns:
            (RigidBodyData, new_offset) or (None, offset) if parsing fails
        """
        try:
            start_offset = offset
            
            # Rigid body ID
            if offset + 4 > len(data):
                return None, offset
            rb_id = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            
            # Position (x, y, z)
            if offset + 12 > len(data):
                return None, start_offset
            pos_x, pos_y, pos_z = struct.unpack('<fff', data[offset:offset+12])
            offset += 12
            
            # Rotation (quaternion: x, y, z, w)
            if offset + 16 > len(data):
                return None, start_offset
            qx, qy, qz, qw = struct.unpack('<ffff', data[offset:offset+16])
            offset += 16
            
            # Number of markers associated with this rigid body
            if offset + 4 > len(data):
                return None, start_offset
            num_markers = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            
            # Parse marker positions (optional, can skip)
            markers = []
            if offset + num_markers * 12 > len(data):
                # Not enough data for markers
                num_markers = 0
            else:
                for _ in range(num_markers):
                    mx, my, mz = struct.unpack('<fff', data[offset:offset+12])
                    markers.append((mx, my, mz))
                    offset += 12
            
            # Mean marker error
            if offset + 4 > len(data):
                return None, start_offset
            mean_error = struct.unpack('<f', data[offset:offset+4])[0]
            offset += 4
            
            # Tracking valid flag (4 bytes, typically 0 or 1)
            if offset + 4 > len(data):
                return None, start_offset
            tracking_valid = bool(struct.unpack('<I', data[offset:offset+4])[0])
            offset += 4
            
            rb_data = RigidBodyData(
                id=rb_id,
                position=(pos_x, pos_y, pos_z),
                rotation=(qx, qy, qz, qw),
                markers=markers,
                mean_marker_error=mean_error,
                tracking_valid=tracking_valid,
                timestamp=0.0  # Will be set from frame timestamp
            )
            
            return rb_data, offset
        
        except Exception as e:
            print(f"[NatNetClient] Error parsing rigid body: {e}")
            return None, offset


if __name__ == "__main__":
    print("NatNet protocol client (pure Python) for OptiTrack Motive integration")
    print("\nUsage:")
    print("  client = NatNetClient('192.168.1.1', 1511)")
    print("  client.set_frame_callback(my_callback)")
    print("  client.start()")
