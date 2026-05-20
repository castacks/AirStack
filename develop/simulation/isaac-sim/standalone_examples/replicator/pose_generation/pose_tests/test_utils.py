# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import json
import os
import shutil

import numpy as np
import scipy.io as sio


def run_pose_generation_test(writer, output_folder, test_folder):
    if writer.lower() == "dope":
        run_dope_test(test_folder, output_folder)
    elif writer.lower() == "ycbvideo":
        run_ycbvideo_tests(test_folder, output_folder)
    else:
        raise Exception(f"No tests exist for the selected writer: {writer}")


# Cleans up output directory so tests are not reading output files from previous run
def clean_output_dir(output_folder):
    if os.path.isdir(output_folder):
        shutil.rmtree(output_folder, ignore_errors=True)


# Checks if distance between points is within threshold
def within_threshold(p1, p2, threshold=20):
    return np.linalg.norm(np.array(p1) - np.array(p2)) < threshold


def run_dope_test(test_folder, output_folder):

    groundtruth_path = os.path.join(test_folder, "dope/000000_groundtruth.json")
    # Look at output for 2nd frame because 1st frame does not get generated properly sometimes
    output_path = os.path.join(output_folder, "000001.json")

    with open(groundtruth_path) as gt_f:
        gt_data = json.load(gt_f)

    with open(output_path) as op_f:
        op_data = json.load(op_f)

    gt_objects, op_objects = gt_data["objects"], op_data["objects"]

    # Does not work with multiple objects. There should be only one object in testing mode.
    if not (len(gt_objects) == 1 and len(op_objects) == 1):
        raise Exception(
            f"Mismatch in .json files between number of objects. gt_objects: {len(gt_objects)}, op_objects: {len(op_objects)}"
        )

    for gt_obj, op_obj in zip(gt_objects, op_objects):
        if not within_threshold(gt_obj["location"], op_obj["location"], 10):
            raise Exception(
                f"Distance between groundtruth location and output location exceeds threshold. (location) {gt_obj['location']} and {op_obj['location']}"
            )
        for gt_pt, op_pt in zip(gt_obj["projected_cuboid"], op_obj["projected_cuboid"]):
            if not within_threshold(gt_pt, op_pt, 20.0):
                raise Exception(
                    f"Distance between groundtruth points and output points exceeds threshold. (projected_cuboid) {gt_pt} and {op_pt}"
                )

    print("Tests pass for DOPE Writer.")


def run_ycbvideo_tests(test_folder, output_folder, threshold=10):
    groundtruth_bbox_path = os.path.join(test_folder, "ycbvideo/000000-box_groundtruth.txt")
    groundtruth_meta_path = os.path.join(test_folder, "ycbvideo/000000-meta_groundtruth.mat")

    # Look at output for 2nd frame because 1st frame does not get generated properly sometimes
    output_bbox_path = os.path.join(output_folder, "data/YCB_Video/data/0000", "000001-box.txt")
    output_meta_path = os.path.join(output_folder, "data/YCB_Video/data/0000", "000001-meta.mat")

    # Compare BBox
    gt_bb = open(groundtruth_bbox_path, "r")
    op_bb = open(output_bbox_path, "r")

    for l1, l2 in zip(gt_bb, op_bb):
        for gt_point, bb_point in zip(l1.strip().split()[1:5], l2.strip().split()[1:5]):
            if not within_threshold([int(gt_point)], [int(bb_point)], 10):
                raise Exception(f"Mismatch between files {groundtruth_bbox_path} and {output_bbox_path}")

    gt_bb.close()
    op_bb.close()

    # Compare Meta File
    gt_meta = sio.loadmat(groundtruth_meta_path)
    op_meta = sio.loadmat(output_meta_path)

    keys_to_compare = ["poses", "intrinsic_matrix", "center"]

    print(f"gt_meta:\n{gt_meta}")
    print(f"op_meta:\n{op_meta}")

    for key in keys_to_compare:

        gt = gt_meta[key].flatten()
        op = op_meta[key].flatten()

        if not len(gt) == len(op):
            raise Exception(f"Mismatch between length of pose in {groundtruth_meta_path} and {output_meta_path}")

        for i in range(len(gt)):
            if abs(gt[i] - op[i]) > threshold:
                raise Exception(
                    f"Mismatch between {key} values in groundtruth and output at index {i}. Groundtruth: {gt[i]} Output: {op[i]}"
                )

        print(f"{key} matches between groundtruth and output.")

    print("Tests pass for YCBVideo Writer.")
