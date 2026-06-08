import ctypes
import struct
from enum import IntEnum

from .natnet_common import MarkerData, ModelLimits


# NatNet data types
class DataDescriptors(IntEnum):
    Descriptor_MarkerSet = 0
    Descriptor_RigidBody = 1
    Descriptor_Skeleton = 2
    Descriptor_ForcePlate = 3
    Descriptor_Device = 4
    Descriptor_Camera = 5
    Descriptor_Asset = 6


# Rigid Body Definition
class sRigidBodyDescription(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("szName", ctypes.c_char * ModelLimits.MAX_NAMELENGTH),  # RigidBody name
        ("ID", ctypes.c_int32),  # RigidBody identifier: Streaming ID value for rigid body assets, and Bone index value for skeleton rigid bodies.
        ("parentID", ctypes.c_int32),  # ID of parent Rigid Body (in case hierarchy exists; otherwise -1)
        ("offsetx", ctypes.c_float),  # offset position relative to parent
        ("offsety", ctypes.c_float),  # offset position relative to parent
        ("offsetz", ctypes.c_float),  # offset position relative to parent
        ("offsetqx", ctypes.c_float),  # Quaternion rotational offset relative to parent for skeleton bones
        ("offsetqy", ctypes.c_float),  # Quaternion rotational offset relative to parent for skeleton bones
        ("offsetqz", ctypes.c_float),  # Quaternion rotational offset relative to parent for skeleton bones
        ("offsetqw", ctypes.c_float),  # Quaternion rotational offset relative to parent for skeleton bones
        ("nMarkers", ctypes.c_int32),  # Number of markers associated with this rigid body
        ("MarkerPositions", MarkerData * ModelLimits.MAX_RBMARKERS),  # Array of marker locations ( [nMarkers][3] )
        ("MarkerRequiredLabels", ctypes.c_int32 * ModelLimits.MAX_RBMARKERS),  # Array of expected marker active labels - 0 if not specified. ( [nMarkers] )
        ("szMarkerNames", ctypes.c_char * ModelLimits.MAX_NAMELENGTH * ModelLimits.MAX_RBMARKERS),  # Array of marker names  ( [nMarkers][MAX_NAMELENGTH] )
    ]

    def pack(self) -> bytes:
        # szName is null-terminated on the wire, not fixed MAX_NAMELENGTH.
        name_bytes = self.szName.rstrip(b"\x00") + b"\x00"
        payload = bytearray(name_bytes)
        payload += struct.pack(
            "<ii",
            self.ID,
            self.parentID,
        )
        payload += struct.pack(
            "<3f",
            self.offsetx,
            self.offsety,
            self.offsetz,
        )
        payload += struct.pack(
            "<4f",
            self.offsetqx,
            self.offsetqy,
            self.offsetqz,
            self.offsetqw,
        )
        payload += struct.pack("<i", self.nMarkers)
        for i in range(self.nMarkers):
            payload += struct.pack(
                "<3f",
                self.MarkerPositions[i][0],
                self.MarkerPositions[i][1],
                self.MarkerPositions[i][2],
            )
        for i in range(self.nMarkers):
            payload += struct.pack("<i", self.MarkerRequiredLabels[i])
        for i in range(self.nMarkers):
            name_offset = i * ModelLimits.MAX_NAMELENGTH
            marker_name = self.szMarkerNames[
                name_offset : name_offset + ModelLimits.MAX_NAMELENGTH
            ]
            payload += marker_name.rstrip(b"\x00") + b"\x00"
        return bytes(payload)


# Tracked Object data description.
# A Mocap Server application (e.g. Arena or TrackingTools) may contain multiple
# tracked "objects (e.g. RigidBody, MarkerSet).  Each object will have its
# own DataDescription.
class sDataDescription(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("type", ctypes.c_int32),
        # union Data (NatNet SDK):
        #   sMarkerSetDescription*  MarkerSetDescription;
        #   sRigidBodyDescription*  RigidBodyDescription;
        #   sSkeletonDescription*   SkeletonDescription;
        #   sForcePlateDescription* ForcePlateDescription;
        #   sDeviceDescription*     DeviceDescription;
        #   sCameraDescription*     CameraDescription;
        #   sAssetDescription*      AssetDescription;
        ("RigidBodyDescription", sRigidBodyDescription),
    ]

    def pack(self) -> bytes:
        if self.type == int(DataDescriptors.Descriptor_RigidBody):
            body = self.RigidBodyDescription.pack()
        else:
            raise ValueError(f"Unsupported data description type: {self.type}")
        payload = bytearray(struct.pack("<ii", self.type, len(body)))
        payload += body
        return bytes(payload)


# All data descriptions for current session (as defined by host app)
class sDataDescriptions(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("nDataDescriptions", ctypes.c_int32),
        ("arrDataDescriptions", sDataDescription * ModelLimits.MAX_MODELS),
    ]

    def pack(self) -> bytes:
        payload = bytearray(struct.pack("<i", self.nDataDescriptions))
        for i in range(self.nDataDescriptions):
            payload += self.arrDataDescriptions[i].pack()
        return bytes(payload)


def make_default_drone_catalog() -> sDataDescriptions:
    """Build the default single-body catalog (Drone id=1) for natnet_ros2."""
    descriptions = sDataDescriptions()
    descriptions.nDataDescriptions = 1
    desc = descriptions.arrDataDescriptions[0]
    desc.type = int(DataDescriptors.Descriptor_RigidBody)
    rb = desc.RigidBodyDescription
    rb.szName = b"Drone"
    rb.ID = 1
    rb.parentID = -1
    rb.offsetqw = 1.0
    rb.nMarkers = 0
    return descriptions
