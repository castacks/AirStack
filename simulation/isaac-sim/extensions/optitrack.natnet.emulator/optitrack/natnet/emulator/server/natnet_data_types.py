import ctypes
import struct
from .natnet_common import ModelLimits, MarkerData

class sMarker(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("ID", ctypes.c_int32),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("size", ctypes.c_float),
        ("params", ctypes.c_int16),
        ("residual", ctypes.c_float)
    ]

    def pack(self) -> bytes:
        return struct.pack('<i5fhf', self.ID, self.x, self.y, self.z, self.size, self.params, self.residual)


class sMarkerSetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("szName", ctypes.c_char * ModelLimits.MAX_NAMELENGTH), # MarkerSet name.
        ("nMarkers", ctypes.c_int32),                           # Number of markers in MarkerSet.
        ("Markers", MarkerData * ModelLimits.MAX_MARKERS)
    ]

    def pack(self) -> bytes:
        # szName is null-terminated on the wire.
        name_bytes = self.szName.rstrip(b'\x00') + b'\x00'
        payload = bytearray(name_bytes)
        payload += struct.pack('<i', self.nMarkers)
        for i in range(self.nMarkers):
            payload += struct.pack('<3f', self.Markers[i][0], self.Markers[i][1], self.Markers[i][2])
        return bytes(payload)

class sRigidBodyData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("ID", ctypes.c_int32),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("qx", ctypes.c_float),
        ("qy", ctypes.c_float),
        ("qz", ctypes.c_float),
        ("qw", ctypes.c_float),
        ("MeanError", ctypes.c_float),
        ("params", ctypes.c_int16),
    ]

    def pack(self) -> bytes:
        return struct.pack('<i8fh', 
            self.ID, 
            self.x, self.y, self.z, 
            self.qx, self.qy, self.qz, self.qw, 
            self.MeanError, self.params
        )

class sSkeletonData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("skeletonID", ctypes.c_int32),
        ("nRigidBodies", ctypes.c_int32),
        ("RigidBodies", sRigidBodyData * ModelLimits.MAX_SKELRIGIDBODIES),
    ]

    def pack(self) -> bytes:
        payload = bytearray(struct.pack('<ii', self.skeletonID, self.nRigidBodies))
        for i in range(self.nRigidBodies):
            payload += self.RigidBodies[i].pack()
        return bytes(payload)

class sAssetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("assetID", ctypes.c_int32),
        ("nRigidBodies", ctypes.c_int32),
        ("RigidBodies", sRigidBodyData * ModelLimits.MAX_SKELRIGIDBODIES),
        ("nMarkers", ctypes.c_int32),
        ("Markers", sMarker * ModelLimits.MAX_MARKERS)
    ]

    def pack(self) -> bytes:
        payload = bytearray(struct.pack('<ii', self.assetID, self.nRigidBodies))
        for i in range(self.nRigidBodies):
            payload += self.RigidBodies[i].pack()
        payload += struct.pack('<i', self.nMarkers)
        for i in range(self.nMarkers):
            payload += self.Markers[i].pack()
        return bytes(payload)

class sAnalogChannelData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
         ("nFrames", ctypes.c_int32),
         ("Values", ctypes.c_float * ModelLimits.MAX_ANALOG_SUBFRAMES)
    ]

    def pack(self) -> bytes:
        payload = bytearray(struct.pack('<i', self.nFrames))
        for i in range(self.nFrames):
            payload += struct.pack('<f', self.Values[i])
        return bytes(payload)

class sForcePlateData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("ID", ctypes.c_int32),
        ("nChannels", ctypes.c_int32),
        ("ChannelData", sAnalogChannelData * ModelLimits.MAX_ANALOG_CHANNELS),
        ("params", ctypes.c_int16),
    ]

    def pack(self) -> bytes:
        payload = bytearray(struct.pack('<ii', self.ID, self.nChannels))
        for i in range(self.nChannels):
            payload += self.ChannelData[i].pack()
        payload += struct.pack('<h', self.params)
        return bytes(payload)

class sDeviceData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("ID", ctypes.c_int32),
        ("nChannels", ctypes.c_int32),
        ("ChannelData", sAnalogChannelData * ModelLimits.MAX_ANALOG_CHANNELS),
        ("params", ctypes.c_int16),
    ]

    def pack(self) -> bytes:
        payload = bytearray(struct.pack('<ii', self.ID, self.nChannels))
        for i in range(self.nChannels):
            payload += self.ChannelData[i].pack()
        payload += struct.pack('<h', self.params)
        return bytes(payload)

class sFrameOfMocapData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("iFrame", ctypes.c_int32),

        ("nMarkerSets", ctypes.c_int32),
        ("MocapData", sMarkerSetData * ModelLimits.MAX_MARKERSETS),

        ("nOtherMarkers", ctypes.c_int32),
        ("OtherMarkers", MarkerData * ModelLimits.MAX_UNLABELED_MARKERS),

        ("nRigidBodies", ctypes.c_int32),
        ("RigidBodies", sRigidBodyData * ModelLimits.MAX_RIGIDBODIES),

        ("nSkeletons", ctypes.c_int32),
        ("Skeletons", sSkeletonData * ModelLimits.MAX_SKELETONS),

        ("nAssets", ctypes.c_int32),
        ("Assets", sAssetData * ModelLimits.MAX_ASSETS),

        ("nLabeledMarkers", ctypes.c_int32),
        ("LabeledMarkers", sMarker * ModelLimits.MAX_LABELED_MARKERS),

        ("nForcePlates", ctypes.c_int32),
        ("ForcePlates", sForcePlateData * ModelLimits.MAX_FORCEPLATES),

        ("nDevices", ctypes.c_int32),
        ("Devices", sDeviceData * ModelLimits.MAX_DEVICES),

        ("Timecode", ctypes.c_uint32),
        ("TimecodeSubframe", ctypes.c_uint32),
        ("fTimestamp", ctypes.c_double),
        ("CameraMidExposureTimestamp", ctypes.c_uint64),
        ("CameraDataReceivedTimestamp", ctypes.c_uint64),
        ("TransmitTimestamp", ctypes.c_uint64),    
        ("PrecisionTimestampSecs", ctypes.c_uint32),
        ("PrecisionTimestampFractionalSecs", ctypes.c_uint32),
        ("params", ctypes.c_int16)
    ]

    @staticmethod
    def _pack_counted_section(count: int, data: bytes, *, natnet_major: int, natnet_minor: int) -> bytes:
        """NatNet 4.1+ prefixes each collection with a 4-byte byte count."""
        payload = bytearray(struct.pack('<i', count))
        if (natnet_major == 4 and natnet_minor > 0) or natnet_major > 4:
            payload += struct.pack('<i', len(data))
        payload += data
        return bytes(payload)

    def pack(self, natnet_major: int = 4, natnet_minor: int = 4) -> bytes:
        payload = bytearray()

        payload += struct.pack('<i', self.iFrame)

        markerset_data = bytearray()
        for i in range(self.nMarkerSets):
            markerset_data += self.MocapData[i].pack()
        payload += self._pack_counted_section(
            self.nMarkerSets, bytes(markerset_data), natnet_major=natnet_major, natnet_minor=natnet_minor
        )

        other_marker_data = bytearray()
        for i in range(self.nOtherMarkers):
            other_marker_data += struct.pack(
                '<3f', self.OtherMarkers[i][0], self.OtherMarkers[i][1], self.OtherMarkers[i][2]
            )
        payload += self._pack_counted_section(
            self.nOtherMarkers, bytes(other_marker_data), natnet_major=natnet_major, natnet_minor=natnet_minor
        )

        rigid_body_data = bytearray()
        for i in range(self.nRigidBodies):
            rigid_body_data += self.RigidBodies[i].pack()
        payload += self._pack_counted_section(
            self.nRigidBodies, bytes(rigid_body_data), natnet_major=natnet_major, natnet_minor=natnet_minor
        )

        skeleton_data = bytearray()
        for i in range(self.nSkeletons):
            skeleton_data += self.Skeletons[i].pack()
        payload += self._pack_counted_section(
            self.nSkeletons, bytes(skeleton_data), natnet_major=natnet_major, natnet_minor=natnet_minor
        )

        asset_data = bytearray()
        for i in range(self.nAssets):
            asset_data += self.Assets[i].pack()
        payload += self._pack_counted_section(
            self.nAssets, bytes(asset_data), natnet_major=natnet_major, natnet_minor=natnet_minor
        )

        labeled_marker_data = bytearray()
        for i in range(self.nLabeledMarkers):
            labeled_marker_data += self.LabeledMarkers[i].pack()
        payload += self._pack_counted_section(
            self.nLabeledMarkers, bytes(labeled_marker_data), natnet_major=natnet_major, natnet_minor=natnet_minor
        )

        force_plate_data = bytearray()
        for i in range(self.nForcePlates):
            force_plate_data += self.ForcePlates[i].pack()
        payload += self._pack_counted_section(
            self.nForcePlates, bytes(force_plate_data), natnet_major=natnet_major, natnet_minor=natnet_minor
        )

        device_data = bytearray()
        for i in range(self.nDevices):
            device_data += self.Devices[i].pack()
        payload += self._pack_counted_section(
            self.nDevices, bytes(device_data), natnet_major=natnet_major, natnet_minor=natnet_minor
        )

        # NatNet 3.0+ Timecodes & stamps
        payload += struct.pack('<II', self.Timecode, self.TimecodeSubframe)
        payload += struct.pack('<d', self.fTimestamp)
        payload += struct.pack('<QQQ', self.CameraMidExposureTimestamp, self.CameraDataReceivedTimestamp, self.TransmitTimestamp)
        payload += struct.pack('<II', self.PrecisionTimestampSecs, self.PrecisionTimestampFractionalSecs)
        payload += struct.pack('<h', self.params)

        # End-of-data tag. libNatNet's frame unpacker reads a trailing 4-byte tag after params; 
        payload += struct.pack('<i', 0)

        return bytes(payload)