# Dataset versions and thermal hardware: verified source boundaries

Checked 2026-10-20 (America/New_York), after EXP-019, on `ore_ms`.

The user supplied a Google-generated summary about TartanDrive1.0/2.0 and
thermal hardware. Its release boundaries and citations do not establish the
provenance of our October2025 recordings. Keep the following distinctions.

## Public TartanDrive2.0 versus later thermal hardware

The [TartanDrive2.0 paper](https://arxiv.org/html/2402.01913v1), III-A/III-C,
describes the Yamaha ATV, added LiDARs, and MultiSense S21 stereo grayscale
images with left-camera RGB. Its conclusions explicitly describe thermal
cameras as a future addition. A shared platform or repository name does not
prove fixed mounting across later acquisitions. This paper does not validate
the calibration of our20251027 thermal streams.

AirLab's [official off-road project page](https://theairlab.org/offroad/),
Hardware System and Dataset, separately states that thermal cameras were added
to the ATV payload after discussing the two public TartanDrive releases.
Thus an older release lacking thermal data does not imply that the later ATV
had no physical thermal hardware.

Yifei Liu's [CMU thermal autonomy thesis](https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf),
Chapter4 Hardware Setup, documents two FLIR BOSON+640 cameras at640x512,
alongside MultiSense S21 stereo at1024x544, with these streams at10Hz.
Its introduction calls the ground thermal dataset TartanDrive2.5T.
These nominal dimensions match our loaded arrays. That is supporting context,
not proof of identical camera identities, mounting, calibration revisions,
training/test splits, or recording membership. The storage/project label
TartanDrive3 and the thesis's TartanDrive2.5T must not be equated without
recording-specific provenance. No dataset paths are renamed from this search.

## The GO claim in the supplied summary is incorrect

The [GO primary paper](https://arxiv.org/html/2501.19274v1), II-A/II-B/II-D,
describes its own Clearpath Warthog with FLIR Blackfly stereo and a physical
FLIR Boson640 thermal camera. It describes calibration between sensors and
transferring semantic labels to thermal images. This is not evidence that GO
consists of TartanDrive imagery with thermal labels retroactively attached.
Semantic label transfer and acquisition of thermal sensor images are different
operations. GO's calibration cannot be substituted for the Yamaha rig's.

## Repository checks

The public [TartanDrive2.0 calibration directory](https://github.com/castacks/tartan_drive_2.0/tree/main/calibration)
contains `extrinsics.yaml` and `multisense_intrinsics.txt` when inspected.
The existence of these files does not establish later thermal calibration.
The linked [AnyThermal issue6](https://github.com/castacks/AnyThermal/issues/6)
raises questions about ZED image resolution, factory calibration and an
identity stereo/IMU transform. Its opening report does not support applying
that calibration or the summary's retroactive-thermal claim to our recordings.
The discussion was not used to assert a resolution for our geometry issue.

Public GitHub resources were inspected through the connected GitHub fetch
tool. Papers/project descriptions were inspected from primary-source web
text/search extraction; full copyrighted PDFs are not included in recovery
artifacts. No hardware changes or exact mounting history were inferred from
photographs or secondary summaries.

## Consequence for this project

Our supplied archives contain named raw uint16 thermal left/right streams,
processed streams, timestamps, optical-frame metadata and calibration. Prior
audits reproduce sampled processing exactly and find physical geometry
inconsistencies. Those measured facts remain intact. Public evidence confirms
that a later AirLab stereo thermal rig existed, while per-recording physical
left/right identity and calibration applicability remain unknown. Keep
defaults and do not replace the supplied thermal calibration with TD2.0, GO
or AnyThermal parameters.
