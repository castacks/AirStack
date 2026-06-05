import ctypes
from enum import IntEnum

MAX_NAMELENGTH = 256
MAX_PACKETSIZE = 65503

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
class sSender(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("szName", ctypes.c_char * MAX_NAMELENGTH), # host app's name
        ("Version", ctypes.c_uint8 * 4),            # host app's version [major.minor.build.revision]
        ("NatNetVersion", ctypes.c_uint8 * 4)       # host app's NatNet version
    ]

    def pack(self) -> bytes:
        return bytes(self)

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
        return bytes(self)

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
        return bytes(self)

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