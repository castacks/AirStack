# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

"""
To generate several frames, create and run a bash script with the following content:

for i in $(seq 64 512); do
    echo ${i}x${i}
    PATH/TO/python.sh test_frame_delay.py --resolution ${i}x${i} --/app/updateOrder/checkForHydraRenderComplete=1000
done
"""


USE_REPLICATOR_WRITER = True
CAMERA_PATH = "/camera"
CAMERA_POS = [0, 0, 25]
COLLECTION_STEPS = 10

# parse any command-line arguments specific to the standalone application
import argparse
import os

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--resolution", type=str, default="256x256", help="Resolution (WxH)")
# Parse only known arguments, so that any (eg) Kit settings are passed through to the core Kit app
args, _ = parser.parse_known_args()

RESOLUTION = tuple([int(item) for item in args.resolution.split("x")])
PIXELS_PER_METER = 0.09765625 * RESOLUTION[0]

simulation_app = SimulationApp(
    {"headless": True}, experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.base.zero_delay.kit'
)

import pprint

import carb
import cv2
import isaacsim.core.utils.numpy.rotations as rot_utils
import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import add_update_semantics
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera
from omni.replicator.core import AnnotatorRegistry, Writer
from pxr import UsdGeom

# rep.settings.set_render_rtx_realtime(antialiasing="DLAA")


class CustomWriter(Writer):
    def __init__(self):
        self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))
        self.annotators.append(AnnotatorRegistry.get_annotator("semantic_segmentation"))
        self.annotators.append(AnnotatorRegistry.get_annotator("bounding_box_2d_tight"))

    def write(self, data):
        pass


def get_data(sensor: Camera | Writer) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Get RGB, semantic segmentation and BBox from Camera or Writer (according to `USE_REPLICATOR_WRITER`)"""
    if USE_REPLICATOR_WRITER:
        rgb = sensor.get_data()["rgb"]
        semantic_segmentation = sensor.get_data()["semantic_segmentation"]["data"]
        bbox = sensor.get_data()["bounding_box_2d_tight"]["data"][0]
    else:
        rgb = sensor.get_rgba()
        semantic_segmentation = sensor._custom_annotators["semantic_segmentation"].get_data()["data"]
        bbox = sensor._custom_annotators["bounding_box_2d_tight"].get_data()["data"][0]

    semantic_segmentation = (semantic_segmentation * 255 / np.max(semantic_segmentation)).astype(np.uint8)
    semantic_segmentation = np.repeat(semantic_segmentation[:, :, np.newaxis], 3, axis=2)
    return rgb[:, :, :3], semantic_segmentation, bbox


def draw_data(frame, position, bbox, label):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = cv2.rectangle(
        img=frame,
        pt1=(
            int(RESOLUTION[1] / 2 + PIXELS_PER_METER * position[0] - PIXELS_PER_METER / 2),
            int(RESOLUTION[1] / 2 + PIXELS_PER_METER * position[1] - PIXELS_PER_METER),
        ),
        pt2=(
            int(RESOLUTION[1] / 2 + PIXELS_PER_METER * position[0] + PIXELS_PER_METER / 2),
            int(RESOLUTION[1] / 2 + PIXELS_PER_METER * position[1] + PIXELS_PER_METER),
        ),
        color=(255, 255, 0),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    frame = cv2.rectangle(
        img=frame,
        pt1=(bbox["x_min"], bbox["y_min"]),
        pt2=(bbox["x_max"], bbox["y_max"]),
        color=(0, 255, 0),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    frame = cv2.putText(
        img=frame,
        text=label,
        org=(5, 15),
        fontFace=cv2.FONT_HERSHEY_PLAIN,
        fontScale=0.75,
        color=(0, 255, 255),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    frame = cv2.putText(
        img=frame,
        text=f"position: {(round(position[0], 2), round(position[1], 2), round(position[2], 2))}",
        org=(5, 30),
        fontFace=cv2.FONT_HERSHEY_PLAIN,
        fontScale=0.75,
        color=(255, 255, 0),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    frame = cv2.putText(
        img=frame,
        text=f'bbox: {(bbox["x_min"], bbox["y_min"])} {(bbox["x_max"], bbox["y_max"])}',
        org=(5, 45),
        fontFace=cv2.FONT_HERSHEY_PLAIN,
        fontScale=0.75,
        color=(0, 255, 0),
        thickness=1,
        lineType=cv2.LINE_AA,
    )
    return frame


def generate_result(data: list[dict], banner: list[str] = []):
    rgb_frames = []
    semantic_segmentation_frames = []
    for item in data:
        rgb_frames.append(draw_data(item["rgb"], item["position"], item["bbox"], item["label"]))
        semantic_segmentation_frames.append(
            draw_data(item["semantic_segmentation"], item["position"], item["bbox"], item["label"])
        )

    separator = np.full((RESOLUTION[0], 5, 3), 0, dtype=np.uint8)
    rgb_frames = [x for item in rgb_frames for x in (item, separator)][:-1]
    semantic_segmentation_frames = [x for item in semantic_segmentation_frames for x in (item, separator)][:-1]

    frame = cv2.vconcat([cv2.hconcat(rgb_frames), cv2.hconcat(semantic_segmentation_frames)])
    if banner:
        frame = cv2.copyMakeBorder(
            frame, top=25, bottom=0, left=0, right=0, borderType=cv2.BORDER_CONSTANT, value=[0] * 3
        )
        frame = cv2.putText(
            img=frame,
            text=", ".join(banner),
            org=(5, 15),
            fontFace=cv2.FONT_HERSHEY_PLAIN,
            fontScale=0.75,
            color=(255, 255, 255),
            thickness=1,
            lineType=cv2.LINE_AA,
        )
    return frame


simulation_app.update()

# Setup scene
set_camera_view(eye=CAMERA_POS, target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

cube = world.scene.add(
    DynamicCuboid(
        prim_path="/cube",
        name="cube",
        position=np.array([-3.0, 0.0, 0.1]),
        scale=np.array([1.0, 2.0, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )
)
add_update_semantics(cube.prim, "cube")

camera = None
writer = None
if USE_REPLICATOR_WRITER:
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.DefinePrim(CAMERA_PATH, "Camera")
    UsdGeom.Xformable(camera_prim).AddTranslateOp().Set(tuple(CAMERA_POS))
    render_product = rep.create.render_product(str(camera_prim.GetPrimPath()), resolution=RESOLUTION)
else:
    camera = Camera(
        prim_path=CAMERA_PATH,
        position=np.array(CAMERA_POS),
        resolution=RESOLUTION,
        orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 90]), degrees=True),
    )

world.reset()

if USE_REPLICATOR_WRITER:
    rep.WriterRegistry.register(CustomWriter)
    writer = rep.WriterRegistry.get("CustomWriter")
    writer.initialize()
    writer.attach([render_product])
else:
    camera.initialize()
    camera.add_bounding_box_2d_tight_to_frame()
    camera.add_semantic_segmentation_to_frame()


# Do some warmup steps
for _ in range(5):
    world.step(render=True)

data = []
# Get data and object info before running the collection steps
position = cube.get_world_pose()[0]
rgb, semantic_segmentation, bbox = get_data(camera or writer)
data.append(
    {"position": position, "rgb": rgb, "semantic_segmentation": semantic_segmentation, "bbox": bbox, "label": "before"}
)

# Do some collection steps
for i in range(COLLECTION_STEPS):
    # Move object
    position = cube.get_world_pose()[0]
    position[0] += 0.5
    cube.set_world_pose(position=position)

    # Step the simulation
    world.step(render=True)

    # Get data and object info
    position = cube.get_world_pose()[0]
    rgb, semantic_segmentation, bbox = get_data(camera or writer)
    data.append(
        {
            "position": position,
            "rgb": rgb,
            "semantic_segmentation": semantic_segmentation,
            "bbox": bbox,
            "label": f"step {i + 1}",
        }
    )

# Export result
banner = [
    f"source: {'rep.Writer' if USE_REPLICATOR_WRITER else 'Camera'}",
    f"checkForHydraRenderComplete: {carb.settings.get_settings().get('/app/updateOrder/checkForHydraRenderComplete')}",
    f"app.hydraEngine.waitIdle: {carb.settings.get_settings().get('/app/hydraEngine/waitIdle')}",
    f"rtx.post.aa.op: {carb.settings.get_settings().get('/rtx/post/aa/op')}",
]
print("")
pprint.pprint(banner)
print("")
cv2.imwrite(f"result-{args.resolution}.png", generate_result(data, banner))

simulation_app.close()
