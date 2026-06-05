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
        ("szName", ctypes.c_char * ModelLimits.MAX_NAMELENGTH),            # MarkerSet name
        ("nMarkers", ctypes.c_int32),                       # # of markers in MarkerSet
        ("Markers", MarkerData * ModelLimits.MAX_MARKERS)   # Using max array, pack will chunk it
    ]

    def pack(self) -> bytes:
        # szName is null-terminated rather than fixed 256 over network
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

    def pack(self) -> bytes:
        payload = bytearray()
        
        payload += struct.pack('<i', self.iFrame)

        payload += struct.pack('<i', self.nMarkerSets)
        for i in range(self.nMarkerSets):
            payload += self.MocapData[i].pack()

        payload += struct.pack('<i', self.nOtherMarkers)
        for i in range(self.nOtherMarkers):
            payload += struct.pack('<3f', self.OtherMarkers[i][0], self.OtherMarkers[i][1], self.OtherMarkers[i][2])

        payload += struct.pack('<i', self.nRigidBodies)
        for i in range(self.nRigidBodies):
            payload += self.RigidBodies[i].pack()

        payload += struct.pack('<i', self.nSkeletons)
        for i in range(self.nSkeletons):
            payload += self.Skeletons[i].pack()

        payload += struct.pack('<i', self.nAssets)
        for i in range(self.nAssets):
            payload += self.Assets[i].pack()

        payload += struct.pack('<i', self.nLabeledMarkers)
        for i in range(self.nLabeledMarkers):
            payload += self.LabeledMarkers[i].pack()

        payload += struct.pack('<i', self.nForcePlates)
        for i in range(self.nForcePlates):
            payload += self.ForcePlates[i].pack()

        payload += struct.pack('<i', self.nDevices)
        for i in range(self.nDevices):
            payload += self.Devices[i].pack()

        # NatNet 3.0+ Timecodes & stamps
        payload += struct.pack('<II', self.Timecode, self.TimecodeSubframe)
        payload += struct.pack('<d', self.fTimestamp)
        payload += struct.pack('<QQQ', self.CameraMidExposureTimestamp, self.CameraDataReceivedTimestamp, self.TransmitTimestamp)
        payload += struct.pack('<II', self.PrecisionTimestampSecs, self.PrecisionTimestampFractionalSecs)
        payload += struct.pack('<h', self.params)

        return bytes(payload)   