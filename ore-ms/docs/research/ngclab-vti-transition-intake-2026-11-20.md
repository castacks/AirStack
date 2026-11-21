# NGCLAB VTI lighting-transition intake

The VTI release lists RGB, LWIR, Xsens IMU, and OptiTrack rigid-body
poses in the lab bags. The varying-light runs marked as paper
experiments are `xyz5` (45.2 s), `halfsphere5` (48.9 s), and
`circle5` (62 s). The catalog does not specify whether illumination
returns or whether RGB tracking fails during any of these runs.
[Pinned VTI release](https://github.com/NGCLAB/multi-spectral-dataset/blob/ad4d727aeb0a4b7abc2ccb0a584aa5fa701b2c73/VTI/TI.md).

The calibration directory provides pinhole intrinsics and explicitly
labeled camera-to-IMU transforms for both camera topics. It does not
provide an OptiTrack-body-to-IMU transform or a timing contract.
The three listed Baidu links returned HTML from this workspace. The
`xyz5` response was an extraction-code page even with the published
code in the URL. No bag bytes were obtained, so exposure history,
camera/IMU timing and mocap coverage remain unverified.
[Pinned calibration directory](https://github.com/NGCLAB/multi-spectral-dataset/tree/ad4d727aeb0a4b7abc2ccb0a584aa5fa701b2c73/VTI/calibration).
