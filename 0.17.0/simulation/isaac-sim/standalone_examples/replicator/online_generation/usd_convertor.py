# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


"""Convert ShapeNetCore V2 to USD without materials.
By only converting the ShapeNet geometry, we can more quickly load assets into scenes for the purpose of creating
large datasets or for online training of Deep Learning models.
"""


import argparse
import os

from isaacsim import SimulationApp

if "SHAPENET_LOCAL_DIR" not in os.environ:
    import carb

    carb.log_error("SHAPENET_LOCAL_DIR not defined:")
    carb.log_error(
        "Please specify the SHAPENET_LOCAL_DIR environment variable to the location of your local shapenet database, exiting"
    )
    exit()

kit = SimulationApp()

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.asset_converter")

from shapenet_utils import shapenet_convert

parser = argparse.ArgumentParser("Convert ShapeNet assets to USD")
parser.add_argument(
    "--categories", type=str, nargs="+", default=None, help="List of ShapeNet categories to convert (space seperated)."
)
parser.add_argument(
    "--max_models", type=int, default=50, help="If specified, convert up to `max_models` per category, default is 50"
)
parser.add_argument(
    "--load_materials", action="store_true", help="If specified, materials will be loaded from shapenet meshes"
)
args, unknown_args = parser.parse_known_args()

# Ensure Omniverse Kit is launched via SimulationApp before shapenet_convert() is called
shapenet_convert(args.categories, args.max_models, args.load_materials)
# cleanup
kit.close()
