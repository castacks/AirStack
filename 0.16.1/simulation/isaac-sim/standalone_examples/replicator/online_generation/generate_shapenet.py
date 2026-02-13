# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


"""Dataset with online randomized scene generation for Instance Segmentation training.

Use OmniKit to generate a simple scene. At each iteration, the scene is populated by
adding assets from the user-specified classes with randomized pose and colour. 
The camera position is also randomized before capturing groundtruth consisting of
an RGB rendered image, Tight 2D Bounding Boxes and Instance Segmentation masks. 
"""


import glob
import os
import signal
import sys

import numpy as np
import torch
from isaacsim import SimulationApp

LABEL_TO_SYNSET = {
    "table": "04379243",
    "monitor": "03211117",
    "phone": "04401088",
    "watercraft": "04530566",
    "chair": "03001627",
    "lamp": "03636649",
    "speaker": "03691459",
    "bench": "02828884",
    "plane": "02691156",
    "bathtub": "02808440",
    "bookcase": "02871439",
    "bag": "02773838",
    "basket": "02801938",
    "bowl": "02880940",
    "bus": "02924116",
    "cabinet": "02933112",
    "camera": "02942699",
    "car": "02958343",
    "dishwasher": "03207941",
    "file": "03337140",
    "knife": "03624134",
    "laptop": "03642806",
    "mailbox": "03710193",
    "microwave": "03761084",
    "piano": "03928116",
    "pillow": "03938244",
    "pistol": "03948459",
    "printer": "04004475",
    "rocket": "04099429",
    "sofa": "04256520",
    "washer": "04554684",
    "rifle": "04090263",
    "can": "02946921",
    "bottle": "02876657",
    "bowl": "02880940",
    "earphone": "03261776",
    "mug": "03797390",
}

SYNSET_TO_LABEL = {v: k for k, v in LABEL_TO_SYNSET.items()}

# Setup default variables
RESOLUTION = (1024, 1024)
OBJ_LOC_MIN = (-50, 5, -50)
OBJ_LOC_MAX = (50, 5, 50)
CAM_LOC_MIN = (100, 0, -100)
CAM_LOC_MAX = (100, 100, 100)
SCALE_MIN = 15
SCALE_MAX = 40

# Default rendering parameters
RENDER_CONFIG = {"headless": False}


class RandomObjects(torch.utils.data.IterableDataset):
    """Dataset of random ShapeNet objects.
    Objects are randomly chosen from selected categories and are positioned, rotated and coloured
    randomly in an empty room. RGB, BoundingBox2DTight and Instance Segmentation are captured by moving a
    camera aimed at the centre of the scene which is positioned at random at a fixed distance from the centre.

    This dataset is intended for use with ShapeNet but will function with any dataset of USD models
    structured as `root/category/**/*.usd. One note is that this is designed for assets without materials
    attached. This is to avoid requiring to compile MDLs and load textures while training.

    Args:
        categories (tuple of str): Tuple or list of categories. For ShapeNet, these will be the synset IDs.
        max_asset_size (int): Maximum asset file size that will be loaded. This prevents out of memory errors
            due to loading large meshes.
        num_assets_min (int): Minimum number of assets populated in the scene.
        num_assets_max (int): Maximum number of assets populated in the scene.
        split (float): Fraction of the USDs found to use for training.
        train (bool): If true, use the first training split and generate infinite random scenes.
    """

    def __init__(
        self, root, categories, max_asset_size=None, num_assets_min=3, num_assets_max=5, split=0.7, train=True
    ):
        assert len(categories) > 1
        assert (split > 0) and (split <= 1.0)

        self.kit = SimulationApp(RENDER_CONFIG)

        import carb
        import omni.replicator.core as rep
        import warp as wp

        self.rep = rep
        self.wp = wp

        from isaacsim.storage.native import get_assets_root_path

        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # If ShapeNet categories are specified with their names, convert to synset ID
        # Remove this if using with a different dataset than ShapeNet
        category_ids = [LABEL_TO_SYNSET.get(c, c) for c in categories]
        self.categories = category_ids
        self.range_num_assets = (num_assets_min, max(num_assets_min, num_assets_max))
        try:
            self.references = self._find_usd_assets(root, category_ids, max_asset_size, split, train)
        except ValueError as err:
            carb.log_error(str(err))
            self.kit.close()
            sys.exit()

        # Setup the scene, lights, walls, camera, etc.
        self.setup_scene()

        # Setup replicator randomizer graph
        self.setup_replicator()

        self.cur_idx = 0
        self.exiting = False

        signal.signal(signal.SIGINT, self._handle_exit)

    def _get_textures(self):
        return [
            self.assets_root_path + "/Isaac/Samples/DR/Materials/Textures/checkered.png",
            self.assets_root_path + "/Isaac/Samples/DR/Materials/Textures/marble_tile.png",
            self.assets_root_path + "/Isaac/Samples/DR/Materials/Textures/picture_a.png",
            self.assets_root_path + "/Isaac/Samples/DR/Materials/Textures/picture_b.png",
            self.assets_root_path + "/Isaac/Samples/DR/Materials/Textures/textured_wall.png",
            self.assets_root_path + "/Isaac/Samples/DR/Materials/Textures/checkered_color.png",
        ]

    def _handle_exit(self, *args, **kwargs):
        print("exiting dataset generation...")
        self.exiting = True

    def close(self):
        self.rep.orchestrator.stop()
        self.kit.close()

    def setup_scene(self):
        from isaacsim.core.utils.prims import create_prim
        from isaacsim.core.utils.rotations import euler_angles_to_quat
        from isaacsim.core.utils.stage import set_stage_up_axis

        """Setup lights, walls, floor, ceiling and camera"""
        # Set stage up axis to Y-up
        set_stage_up_axis("y")

        # In a practical setting, the room parameters should attempt to match those of the
        # target domain. Here, we instead opt for simplicity.
        create_prim("/World/Room", "Sphere", attributes={"radius": 1e3, "primvars:displayColor": [(1.0, 1.0, 1.0)]})
        create_prim(
            "/World/Ground",
            "Cylinder",
            position=np.array([0.0, -0.5, 0.0]),
            orientation=euler_angles_to_quat(np.array([90.0, 0.0, 0.0]), degrees=True),
            attributes={"height": 1, "radius": 1e4, "primvars:displayColor": [(1.0, 1.0, 1.0)]},
        )
        create_prim("/World/Asset", "Xform")

        self.camera = self.rep.create.camera()
        self.render_product = self.rep.create.render_product(self.camera, RESOLUTION)

        # Setup annotators that will report groundtruth
        self.rgb = self.rep.AnnotatorRegistry.get_annotator("rgb")
        self.bbox_2d_tight = self.rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
        self.instance_seg = self.rep.AnnotatorRegistry.get_annotator("instance_segmentation")
        self.rgb.attach(self.render_product)
        self.bbox_2d_tight.attach(self.render_product)
        self.instance_seg.attach(self.render_product)

        self.kit.update()

    def _find_usd_assets(self, root, categories, max_asset_size, split, train=True):
        """Look for USD files under root/category for each category specified.
        For each category, generate a list of all USD files found and select
        assets up to split * len(num_assets) if `train=True`, otherwise select the
        remainder.
        """
        references = {}
        for category in categories:
            all_assets = glob.glob(os.path.join(root, category, "*/*.usd"), recursive=True)
            print(os.path.join(root, category, "*/*.usd"))
            # Filter out large files (which can prevent OOM errors during training)
            if max_asset_size is None:
                assets_filtered = all_assets
            else:
                assets_filtered = []
                for a in all_assets:
                    if os.stat(a).st_size > max_asset_size * 1e6:
                        print(f"{a} skipped as it exceeded the max size {max_asset_size} MB.")
                    else:
                        assets_filtered.append(a)

            num_assets = len(assets_filtered)
            if num_assets == 0:
                raise ValueError(f"No USDs found for category {category} under max size {max_asset_size} MB.")

            if train:
                references[category] = assets_filtered[: int(num_assets * split)]
            else:
                references[category] = assets_filtered[int(num_assets * split) :]
        return references

    def _instantiate_category(self, category, references):
        with self.rep.randomizer.instantiate(references, size=1, mode="reference"):
            self.rep.modify.semantics([("class", category)])
            self.rep.modify.pose(
                position=self.rep.distribution.uniform(OBJ_LOC_MIN, OBJ_LOC_MAX),
                rotation=self.rep.distribution.uniform((0, -180, 0), (0, 180, 0)),
                scale=self.rep.distribution.uniform(SCALE_MIN, SCALE_MAX),
            )
            self.rep.randomizer.texture(self._get_textures(), project_uvw=True)

    def setup_replicator(self):
        """Setup the replicator graph with various attributes."""

        # Create two sphere lights
        light1 = self.rep.create.light(light_type="sphere", position=(-450, 350, 350), scale=100, intensity=30000.0)
        light2 = self.rep.create.light(light_type="sphere", position=(450, 350, 350), scale=100, intensity=30000.0)

        with self.rep.new_layer():
            with self.rep.trigger.on_frame():
                # Randomize light colors
                with self.rep.create.group([light1, light2]):
                    self.rep.modify.attribute("color", self.rep.distribution.uniform((0.1, 0.1, 0.1), (1.0, 1.0, 1.0)))

                # Randomize camera position
                with self.camera:
                    self.rep.modify.pose(
                        position=self.rep.distribution.uniform(CAM_LOC_MIN, CAM_LOC_MAX), look_at=(0, 0, 0)
                    )

                # Randomize asset positions and textures
                for category, references in self.references.items():
                    self._instantiate_category(category, references)

        # Run replicator for a single iteration without triggering any writes
        self.rep.orchestrator.preview()

    def __iter__(self):
        return self

    def __next__(self):
        # Step - trigger a randomization and a render
        self.rep.orchestrator.step(rt_subframes=4)

        # Collect Groundtruth
        gt = {
            "rgb": self.rgb.get_data(device="cuda"),
            "boundingBox2DTight": self.bbox_2d_tight.get_data(device="cpu"),
            "instanceSegmentation": self.instance_seg.get_data(device="cuda"),
        }

        # RGB
        # Drop alpha channel
        image = self.wp.to_torch(gt["rgb"])[..., :3]

        # Normalize between 0. and 1. and change order to channel-first.
        image = image.float() / 255.0
        image = image.permute(2, 0, 1)

        # Bounding Box
        gt_bbox = gt["boundingBox2DTight"]["data"]

        # Create mapping from categories to index
        bboxes = torch.tensor(gt_bbox[["x_min", "y_min", "x_max", "y_max"]].tolist(), device="cuda")
        id_to_labels = gt["boundingBox2DTight"]["info"]["idToLabels"]
        prim_paths = gt["boundingBox2DTight"]["info"]["primPaths"]

        # For each bounding box, map semantic label to label index
        cat_to_id = {cat: i + 1 for i, cat in enumerate(self.categories)}
        semantic_labels_mapping = {int(k): v.get("class", "") for k, v in id_to_labels.items()}
        semantic_labels = [cat_to_id[semantic_labels_mapping[i]] for i in gt_bbox["semanticId"]]
        labels = torch.tensor(semantic_labels, device="cuda")

        # Calculate bounding box area for each area
        areas = (bboxes[:, 2] - bboxes[:, 0]) * (bboxes[:, 3] - bboxes[:, 1])
        # Identify invalid bounding boxes to filter final output
        valid_areas = (areas > 0.0) * (areas < (image.shape[1] * image.shape[2]))

        # Instance Segmentation
        instance_data = self.wp.to_torch(gt["instanceSegmentation"]["data"].view(self.wp.int32)).squeeze()
        path_to_instance_id = {v: int(k) for k, v in gt["instanceSegmentation"]["info"]["idToLabels"].items()}

        instance_list = [im[0] for im in gt_bbox]
        masks = torch.zeros((len(instance_list), *instance_data.shape), dtype=bool, device="cuda")

        # Filter for the mask of each object
        for i, prim_path in enumerate(prim_paths):
            # Merge child instances of prim_path as one instance
            for instance in path_to_instance_id:
                if prim_path in instance:
                    masks[i] += torch.isin(instance_data, path_to_instance_id[instance])

        target = {
            "boxes": bboxes[valid_areas],
            "labels": labels[valid_areas],
            "masks": masks[valid_areas],
            "image_id": torch.LongTensor([self.cur_idx]),
            "area": areas[valid_areas],
            "iscrowd": torch.BoolTensor([False] * len(bboxes[valid_areas])),  # Assume no crowds
        }

        self.cur_idx += 1
        return image, target


if __name__ == "__main__":
    "Typical usage"
    import argparse
    import struct

    import matplotlib
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser("Dataset test")
    parser.add_argument("--categories", type=str, nargs="+", required=True, help="List of object classes to use")
    parser.add_argument(
        "--max_asset_size",
        type=float,
        default=10.0,
        help="Maximum asset size to use in MB. Larger assets will be skipped.",
    )
    parser.add_argument(
        "--num_test_images", type=int, default=10, help="number of test images to generate when executing main"
    )
    parser.add_argument(
        "--root",
        type=str,
        default=None,
        help="Root directory containing USDs. If not specified, use {SHAPENET_LOCAL_DIR}_mat as root.",
    )
    args, unknown_args = parser.parse_known_args()

    # If root is not specified use the environment variable SHAPENET_LOCAL_DIR with the _mat suffix as root
    if args.root is None:
        if "SHAPENET_LOCAL_DIR" in os.environ:
            shapenet_local_dir = f"{os.path.abspath(os.environ['SHAPENET_LOCAL_DIR'])}_mat"
            if os.path.exists(shapenet_local_dir):
                args.root = shapenet_local_dir
        if args.root is None:
            print(
                "root argument not specified and SHAPENET_LOCAL_DIR environment variable was not set or the path did not exist"
            )
            exit()

    dataset = RandomObjects(args.root, args.categories, max_asset_size=args.max_asset_size)
    from omni.replicator.core import random_colours

    categories = [LABEL_TO_SYNSET.get(c, c) for c in args.categories]

    # Iterate through dataset and visualize the output
    plt.ion()
    _, axes = plt.subplots(1, 2, figsize=(10, 5))
    plt.tight_layout()

    # Directory to save the example images to
    out_dir = os.path.join(os.getcwd(), "_out_gen_imgs", "")
    print(f"[Online-SDG] Saving images to {out_dir}")
    os.makedirs(out_dir, exist_ok=True)

    image_num = 0
    for image, target in dataset:
        for ax in axes:
            ax.clear()
            ax.axis("off")

        np_image = image.permute(1, 2, 0).cpu().numpy()
        axes[0].imshow(np_image)

        num_instances = len(target["boxes"])
        # Create random colors for each instance as rgb float lists
        colours = random_colours(num_instances, num_channels=3)
        colours = colours.astype(float) / 255.0
        colours = colours.tolist()

        overlay = np.zeros_like(np_image)
        for mask, colour in zip(target["masks"].cpu().numpy(), colours):
            overlay[mask, :3] = colour

        axes[1].imshow(overlay)
        mapping = {i + 1: cat for i, cat in enumerate(categories)}
        labels = [SYNSET_TO_LABEL[mapping[label.item()]] for label in target["labels"]]
        for bb, label, colour in zip(target["boxes"].tolist(), labels, colours):
            maxint = 2 ** (struct.Struct("i").size * 8 - 1) - 1
            # if a bbox is not visible, do not draw
            if bb[0] != maxint and bb[1] != maxint:
                x = bb[0]
                y = bb[1]
                w = bb[2] - x
                h = bb[3] - y
                box = plt.Rectangle((x, y), w, h, fill=False, edgecolor=colour)
                ax.add_patch(box)
                ax.text(bb[0], bb[1], label, fontdict={"family": "sans-serif", "color": colour, "size": 10})

        # Use plt.pause only if the backend is interactive
        if matplotlib.get_backend() in ["TkAgg", "nbAgg"]:
            plt.draw()
            plt.pause(0.01)
        fig_name = os.path.join(out_dir, f"domain_randomization_test_image_{image_num}.png")
        plt.savefig(fig_name)
        image_num += 1
        if dataset.exiting or (image_num >= args.num_test_images):
            break

    # cleanup
    dataset.close()
