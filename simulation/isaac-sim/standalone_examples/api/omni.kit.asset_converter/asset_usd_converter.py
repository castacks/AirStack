# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import argparse
import asyncio
import os

from isaacsim import SimulationApp


async def convert(in_file, out_file, load_materials=False):
    # This import causes conflicts when global
    import omni.kit.asset_converter

    def progress_callback(progress, total_steps):
        pass

    converter_context = omni.kit.asset_converter.AssetConverterContext()
    # setup converter and flags
    converter_context.ignore_materials = not load_materials
    # converter_context.ignore_animation = False
    # converter_context.ignore_cameras = True
    # converter_context.single_mesh = True
    # converter_context.smooth_normals = True
    # converter_context.preview_surface = False
    # converter_context.support_point_instancer = False
    # converter_context.embed_mdl_in_usd = False
    # converter_context.use_meter_as_world_unit = True
    # converter_context.create_world_as_default_root_prim = False
    instance = omni.kit.asset_converter.get_instance()
    task = instance.create_converter_task(in_file, out_file, progress_callback, converter_context)
    success = True
    while True:
        success = await task.wait_until_finished()
        if not success:
            await asyncio.sleep(0.1)
        else:
            break
    return success


def asset_convert(args):
    supported_file_formats = ["stl", "obj", "fbx"]
    for folder in args.folders:
        local_asset_output = folder + "_converted"
        result = omni.client.create_folder(f"{local_asset_output}")

    for folder in args.folders:
        print(f"\nConverting folder {folder}...")

        (result, models) = omni.client.list(folder)
        for i, entry in enumerate(models):
            if i >= args.max_models:
                print(f"max models ({args.max_models}) reached, exiting conversion")
                break

            model = str(entry.relative_path)
            model_name = os.path.splitext(model)[0]
            model_format = (os.path.splitext(model)[1])[1:]
            # Supported input file formats
            if model_format in supported_file_formats:
                input_model_path = folder + "/" + model
                converted_model_path = folder + "_converted/" + model_name + "_" + model_format + ".usd"
                if not os.path.exists(converted_model_path):
                    status = asyncio.get_event_loop().run_until_complete(
                        convert(input_model_path, converted_model_path, True)
                    )
                    if not status:
                        print(f"ERROR Status is {status}")
                    print(f"---Added {converted_model_path}")


if __name__ == "__main__":
    kit = SimulationApp()

    import omni
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.kit.asset_converter")

    parser = argparse.ArgumentParser("Convert OBJ/STL assets to USD")
    parser.add_argument(
        "--folders", type=str, nargs="+", default=None, help="List of folders to convert (space seperated)."
    )
    parser.add_argument(
        "--max-models", type=int, default=50, help="If specified, convert up to `max-models` per folder."
    )
    parser.add_argument(
        "--load-materials", action="store_true", help="If specified, materials will be loaded from meshes"
    )
    args, unknown_args = parser.parse_known_args()

    if args.folders is not None:
        # Ensure Omniverse Kit is launched via SimulationApp before asset_convert() is called
        asset_convert(args)
    else:
        print(f"No folders specified via --folders argument, exiting")

    # cleanup
    kit.close()
