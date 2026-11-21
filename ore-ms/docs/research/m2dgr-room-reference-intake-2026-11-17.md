# M2DGR bright and dark room reference intake

After the [M2P2 pose-reference result](m2p2-night-registration-and-support-2026-11-17.md),
screened M2DGR for an external mocap source and separate bright/dark recordings
for the existing RGB–thermal comparison. The [dataset project](https://sjtu-visys.github.io/M2DGR/)
and [paper](https://arxiv.org/abs/2112.13659) document Vicon for the room
sequences and separate camera and thermal topics. The released calibration
includes thermal intrinsics and a thermal-to-LiDAR matrix. Actual paired
streams and the tracked-body-to-camera/exposure contract remain unverified.

Downloaded all nine room TUM files at release source revision
`44acebee11a750768fee140bdcfce52209860f4b`. All matched the sizes and SHA-256
digests in their Git LFS pointers. All rows have eight finite fields and
strictly increasing timestamps. Every file contains some quaternions
whose norm differs from unity by more than 1e-4.

| Sequence | Rows | Non-unit quaternions | Maximum timestamp gap |
|---|---:|---:|---:|
| room_01 | 3,696 | 165 | 333 ms |
| room_02 | 3,831 | 90 | 331 ms |
| room_03 | 6,444 | 216 | 335 ms |
| room_dark_01 | 5,621 | 122 | 428 ms |
| room_dark_02 | 8,317 | 191 | 333 ms |
| room_dark_03 | 5,933 | 145 | 332 ms |
| room_dark_04 | 6,856 | 107 | 1,157 ms |
| room_dark_05 | 7,871 | 78 | 355 ms |
| room_dark_06 | 9,245 | 160 | 336 ms |

Some norms exceed 10. The [maintainer's reply on quaternion jumps](https://github.com/SJTU-ViSYS/M2DGR/issues/54)
attributes abrupt changes to intermittent mocap tracking trouble. Retain the
original values and gaps. Any future eligibility rule needs to identify
unsupported intervals before camera results and account for impact on
bright/dark coverage. Normalizing alone cannot recover the orientations.

A 32-byte range request for the officially linked 20.2 GB `room_dark_01`
OneDrive bag returned HTTP 403. Historical links returned HTTP 429 in this
workspace. No bag bytes were obtained. Reference files and the audit are
saved under `/root/ore-ms-artifacts/m2dgr-intake/`. This intake establishes
a candidate with external mocap and a concrete release issue. Camera input
and a qualified 6-DoF scoring reference remain unavailable.

## Dark-room access follow-up

On 20 November 2026, bounded 32-byte requests to all six current
dark-room OneDrive bag links returned HTTP 403 from this workspace.
The release labels these runs dark and the three bright runs
separately. No camera payloads were available to establish an
in-sequence lighting change, paired camera timing, or motion-capture
support across a transition.
[Official sequence list](https://sjtu-visys.github.io/M2DGR/#dataset-sequences).
