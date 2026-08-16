from enum import IntEnum
import ctypes

class ModelLimits(IntEnum):
    MAX_MODELS = 2000    # maximum number of total models (data descriptions)
    MAX_MARKERSETS = 1000    # maximum number of MarkerSets 
    MAX_RIGIDBODIES = 1000    # maximum number of RigidBodies
    MAX_ASSETS = 1000    # Maximum number of Assets
    MAX_NAMELENGTH = 256     # maximum length for strings
    MAX_MARKERS = 200     # maximum number of markers per MarkerSet
    MAX_RBMARKERS = 20      # maximum number of markers per RigidBody
    MAX_SKELETONS = 100     # maximum number of skeletons
    MAX_SKELRIGIDBODIES = 200     # maximum number of RididBodies per Skeleton
    MAX_LABELED_MARKERS = 1000    # maximum number of labeled markers per frame
    MAX_UNLABELED_MARKERS = 1000    # maximum number of unlabeled (other) markers per frame

    MAX_FORCEPLATES = 100     # maximum number of force plate 'bundles'
    MAX_DEVICES = 100     # maximum number of peripheral device 'bundles'
    MAX_ANALOG_CHANNELS = 32      # maximum number of data channels (signals) per analog/force plate device
    MAX_ANALOG_SUBFRAMES = 30      # maximum number of analog/force plate frames per mocap frame

    MAX_PACKETSIZE = 65503   # max size of packet in bytes (actual packet size is dynamic)
                                                # (65535 byte IP limit - 20 byte IP header - 8 byte UDP header - 4 byte sPacket header = 65503 bytes)



MarkerData = ctypes.c_float * 3
