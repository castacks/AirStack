# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import os

import carb

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


def get_local_shape_loc():
    g_local_shape_loc = None
    env_path = os.getenv("SHAPENET_LOCAL_DIR")
    if env_path == None:
        resolved_data_path = carb.tokens.get_tokens_interface().resolve("${data}")
        g_local_shape_loc = resolved_data_path + "/shapenet"
        print(f"env var SHAPENET_LOCAL_DIR not set, using default data dir {g_local_shape_loc}")
    else:
        g_local_shape_loc = env_path
        print(f"Using local env var SHAPENET_LOCAL_DIR {env_path}")

    return g_local_shape_loc


async def convert(in_file, out_file, load_materials=False):
    # This import causes conflicts when global
    import asyncio

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


def shapenet_convert(categories=None, max_models=50, load_materials=False):
    """Helper to convert shapenet assets to USD


    Args:
            categories (list of string): List of ShapeNet categories to convert.
            max_models (int): Maximum number of models to convert.
            load_materials (bool): If true, materials will be loaded from shapenet meshes.
    """
    import asyncio
    import pprint

    print("[DEPRECATION WARNING] the omni.isaac.shapenet extension will be removed next release.")
    if categories is None:
        print("The following categories and id's are supported:")
        pprint.pprint(LABEL_TO_SYNSET)
        raise ValueError(f"No categories specified via --categories argument")
    # Ensure all categories specified are valid
    invalid_categories = []
    for c in categories:
        if c not in LABEL_TO_SYNSET.keys() and c not in LABEL_TO_SYNSET.values():
            invalid_categories.append(c)

    if invalid_categories:
        raise ValueError(f"The following are not valid ShapeNet categories: {invalid_categories}")

    # This import needs to occur after kit is loaded so that physx can be discovered
    local_shapenet = get_local_shape_loc()
    local_shapenet_output = f"{os.path.abspath(local_shapenet)}_nomat"
    if load_materials:
        local_shapenet_output = f"{os.path.abspath(local_shapenet)}_mat"
    os.makedirs(local_shapenet_output, exist_ok=True)

    synsets = categories
    if synsets is None:
        synsets = LABEL_TO_SYNSET.values()

    for synset in synsets:
        print(f"\nConverting synset {synset}...")
        # If synset is specified by label, convert to synset
        if synset in LABEL_TO_SYNSET:
            synset = LABEL_TO_SYNSET[synset]

        model_dirs = os.listdir(os.path.join(local_shapenet, synset))
        for i, model_id in enumerate(model_dirs):
            if i >= max_models:
                print(f"max models ({max_models}) reached, exiting conversion")
                break
            local_path = os.path.join(local_shapenet, synset, model_id, "models/model_normalized.obj")

            shape_name = "model_normalized_nomat"
            if load_materials:
                shape_name = "model_normalized_mat"

            out_dir = os.path.join(local_shapenet_output, synset, model_id)
            os.makedirs(out_dir, exist_ok=True)
            out_path = os.path.join(out_dir, f"{shape_name}.usd")
            if not os.path.exists(out_path):
                status = asyncio.get_event_loop().run_until_complete(convert(local_path, out_path, load_materials))
                if not status:
                    print(f"ERROR OmniConverterStatus is {status}")
                print(f"---Added {out_path}")
