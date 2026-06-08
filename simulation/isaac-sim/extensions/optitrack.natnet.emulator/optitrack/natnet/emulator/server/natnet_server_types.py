import ctypes
import struct
from enum import IntEnum

MAX_NAMELENGTH = 256
MAX_PACKETSIZE = 65503

# NatNet SDK sServerDescription uses default struct alignment (#pragma pack(pop)), not pack(1).
SERVER_DESCRIPTION_WIRE_SIZE = 552
# NAT_CONNECT / NAT_SERVERINFO reply uses packed sSender_Server (#pragma pack(1) in NatNetTypes.h).
SENDER_SERVER_WIRE_SIZE = 256 + 4 + 4 + 8 + 2 + 1 + 4  # 279

# Client/server message ids
class MessageId(IntEnum):
    NAT_CONNECT                 = 0
    NAT_SERVERINFO              = 1
    NAT_REQUEST                 = 2
    NAT_RESPONSE                = 3
    NAT_REQUEST_MODELDEF        = 4
    NAT_MODELDEF                = 5
    NAT_REQUEST_FRAMEOFDATA     = 6
    NAT_FRAMEOFDATA             = 7
    NAT_MESSAGESTRING           = 8
    NAT_DISCONNECT              = 9
    NAT_KEEPALIVE               = 10
    NAT_DISCONNECTBYTIMEOUT     = 11
    NAT_ECHOREQUEST             = 12
    NAT_ECHORESPONSE            = 13
    NAT_DISCOVERY               = 14
    NAT_UNRECOGNIZED_REQUEST    = 100

# Server/Sender configuration and info
def _fixed_name(field: ctypes.Array) -> bytes:
    raw = bytes(field).split(b"\x00", 1)[0] + b"\x00"
    if len(raw) > MAX_NAMELENGTH:
        raw = raw[: MAX_NAMELENGTH - 1] + b"\x00"
    return raw + b"\x00" * (MAX_NAMELENGTH - len(raw))


class sSender(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("szName", ctypes.c_char * MAX_NAMELENGTH), # host app's name
        ("Version", ctypes.c_uint8 * 4),            # host app's version [major.minor.build.revision]
        ("NatNetVersion", ctypes.c_uint8 * 4)       # host app's NatNet version
    ]

    def pack(self) -> bytes:
        payload = bytearray()
        payload += _fixed_name(self.szName)
        payload += bytes(self.Version)
        payload += bytes(self.NatNetVersion)
        return bytes(payload)

class sSender_Server(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("Common", sSender),
        ("HighResClockFrequency", ctypes.c_uint64),
        ("DataPort", ctypes.c_uint16),
        ("IsMulticast", ctypes.c_bool),
        ("MulticastGroupAddress", ctypes.c_uint8 * 4)
    ]

    def pack(self) -> bytes:
        payload = bytearray(self.Common.pack())
        payload += struct.pack("<QH", self.HighResClockFrequency, self.DataPort)
        payload.append(1 if self.IsMulticast else 0)
        payload += bytes(self.MulticastGroupAddress)
        if len(payload) != SENDER_SERVER_WIRE_SIZE:
            raise ValueError(
                f"sSender_Server wire size {len(payload)} != expected {SENDER_SERVER_WIRE_SIZE}"
            )
        return bytes(payload)

# Mocap server application description
class sServerDescription(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("HostPresent", ctypes.c_bool),
        ("szHostComputerName", ctypes.c_char * MAX_NAMELENGTH),
        ("HostComputerAddress", ctypes.c_uint8 * 4),
        ("szHostApp", ctypes.c_char * MAX_NAMELENGTH),
        ("HostAppVersion", ctypes.c_uint8 * 4),
        ("NatNetVersion", ctypes.c_uint8 * 4),
        ("HighResClockFrequency", ctypes.c_uint64),
        ("bConnectionInfoValid", ctypes.c_bool),
        ("ConnectionDataPort", ctypes.c_uint16),
        ("ConnectionMulticast", ctypes.c_bool),
        ("ConnectionMulticastAddress", ctypes.c_uint8 * 4)
    ]

    def pack(self) -> bytes:
        # Wire layout matches NatNet SDK on x86-64 (3 pad bytes before HighResClockFrequency).
        payload = bytearray()
        payload.append(1 if self.HostPresent else 0)
        payload += _fixed_name(self.szHostComputerName)
        payload += bytes(self.HostComputerAddress)
        payload += _fixed_name(self.szHostApp)
        payload += bytes(self.HostAppVersion)
        payload += bytes(self.NatNetVersion)
        while len(payload) % 8:
            payload.append(0)
        payload += struct.pack("<Q", self.HighResClockFrequency)
        payload.append(1 if self.bConnectionInfoValid else 0)
        payload.append(0)  # pad before ConnectionDataPort (offset 537)
        payload += struct.pack("<H", self.ConnectionDataPort)
        payload.append(1 if self.ConnectionMulticast else 0)
        payload += bytes(self.ConnectionMulticastAddress)
        while len(payload) < SERVER_DESCRIPTION_WIRE_SIZE:
            payload.append(0)
        if len(payload) != SERVER_DESCRIPTION_WIRE_SIZE:
            raise ValueError(
                f"sServerDescription wire size {len(payload)} != "
                f"expected {SERVER_DESCRIPTION_WIRE_SIZE}"
            )
        return bytes(payload)

# Base packet layout
class sPacketHeader(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("iMessage", ctypes.c_uint16),    # e.g., MessageId.NAT_FRAMEOFDATA
        ("nDataBytes", ctypes.c_uint16),  # Bytes purely in the payload following this header
    ]

    def pack(self) -> bytes:
        return bytes(self)

# Connection types enum matching NatNet SDK rules
class ConnectionType(IntEnum):
    ConnectionType_Multicast = 0
    ConnectionType_Unicast   = 1

class sNatNetClientConnectParams(ctypes.Structure):
    """
    Python ctypes translation of the C++ sNatNetClientConnectParams struct.
    Enforces a packed structure byte alignment matching the NatNet binary network protocol.
    """
    _pack_ = 1
    _fields_ = [
        ("connectionType", ctypes.c_int32),         # 4 bytes (mapping to standard ConnectionType enum)
        ("serverCommandPort", ctypes.c_uint16),     # 2 bytes
        ("serverDataPort", ctypes.c_uint16),        # 2 bytes
        
        # NOTE: Represented as void pointers (c_void_p) to safely match the host system's native bit size (e.g., 8 bytes on 64-bit) without string data unpacking overhead.
        ("serverAddress", ctypes.c_void_p),         
        ("localAddress", ctypes.c_void_p),          
        ("multicastAddress", ctypes.c_void_p),      
        
        ("subscribedDataOnly", ctypes.c_bool),      # 1 byte
        ("BitstreamVersion", ctypes.c_uint8 * 4)    # 4 bytes: [Major, Minor, Build, Revision]
    ]

    def pack(self) -> bytes:
        return bytes(self)