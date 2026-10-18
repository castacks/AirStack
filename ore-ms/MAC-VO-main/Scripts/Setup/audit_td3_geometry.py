"""Audit archived TD3 calibration as graph constraints; never rewrite its TF tree."""

import argparse
from collections import defaultdict, deque
import json
import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def pose_matrix(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError("Expected finite xyz, qx, qy, qz, qw transform")
    if not np.isclose(np.linalg.norm(values[3:]), 1., atol=1e-4):
        raise ValueError("Transform quaternion is not unit length")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def rectification_residual(relative, left, right):
    """Compare a candidate T_left<-right with the stereo projection geometry."""
    lp, rp = np.asarray(left["p"]), np.asarray(right["p"])
    baseline = -rp[0, 3] / rp[0, 0] + lp[0, 3] / lp[0, 0]
    translation = np.asarray(left["r"]) @ relative[:3, 3]
    rotation = np.asarray(left["r"]) @ relative[:3, :3] @ np.asarray(right["r"]).T
    return {"horizontal_baseline_error_m": float(translation[0] - baseline),
            "off_axis_translation_m": float(np.linalg.norm(translation[1:])),
            "rectified_rotation_error_deg": float(Rotation.from_matrix(rotation).magnitude() * 180 / np.pi)}


def transform_difference(actual, expected):
    residual = np.linalg.inv(expected) @ actual
    return {"translation_m": float(np.linalg.norm(residual[:3, 3])),
            "rotation_deg": float(Rotation.from_matrix(residual[:3, :3]).magnitude() * 180 / np.pi)}


def static_query(edges, target, source):
    """Compose explicit T_parent<-child constraints without changing their labels."""
    adjacency = defaultdict(list)
    for parent, child, matrix in edges:
        adjacency[parent].append((child, matrix))
        adjacency[child].append((parent, np.linalg.inv(matrix)))
    queue = deque([(target, np.eye(4))]); seen = {target}
    while queue:
        frame, matrix = queue.popleft()
        if frame == source:
            return matrix
        for other, step in adjacency[frame]:
            if other not in seen:
                seen.add(other); queue.append((other, matrix @ step))
    raise ValueError(f"No static path from {source} into {target}")


def compare_calibration(dataset, calibration):
    dataset, calibration = Path(dataset), Path(calibration)
    config = yaml.safe_load(calibration.read_text())
    comparisons, yaml_edges = [], []
    for entry in config["transform_params"]:
        parent, child = entry["from_frame"], entry["to_frame"]
        matrix = pose_matrix(entry["translation"] + entry["quaternion"])
        yaml_edges.append((parent, child, matrix))
        row = {"yaml_from": parent, "yaml_to": child, "archive_candidates": []}
        for first, second in [(parent, child), (child, parent)]:
            name = f"{first.replace('/', '-')}_to_{second.replace('/', '-')}"
            path = dataset / "tf" / name / "static_transform.txt"
            if path.is_file():
                archived = pose_matrix(np.loadtxt(path))
                row["archive_candidates"].append({
                    "directory": name, "labels_reversed": first != parent,
                    "same_matrix_residual": transform_difference(archived, matrix),
                    "inverse_matrix_residual": transform_difference(archived, np.linalg.inv(matrix))})
        comparisons.append(row)
    relative = static_query(yaml_edges, "thermal_left/optical_frame", "thermal_right/optical_frame")
    left = yaml.safe_load((dataset / "sensors/thermal_left_processed_camera_info/data.yaml").read_text())
    right = yaml.safe_load((dataset / "sensors/thermal_right_processed_camera_info/data.yaml").read_text())
    return {"source": str(calibration), "sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
            "yaml_matrix_convention": "T_from_frame<-to_frame, matching inspected TfManager YAML loader",
            "transform_comparisons": comparisons,
            "thermal_T_left_from_right_using_yaml": relative.tolist(),
            "thermal_projection_residuals_using_yaml": rectification_residual(relative, left, right),
            "status": "source comparison only; no source transforms or camera matrices changed"}


def compare_thermal_rectification(dataset, thermal_config):
    """Reproduce both matrix directions without adopting either as calibration."""
    import cv2
    dataset, thermal_config = Path(dataset), Path(thermal_config)
    cfg = yaml.safe_load(thermal_config.read_text())["calib"]
    left = yaml.safe_load((dataset / "sensors/thermal_left_processed_camera_info/data.yaml").read_text())
    right = yaml.safe_load((dataset / "sensors/thermal_right_processed_camera_info/data.yaml").read_text())
    matrix = np.array(cfg["cam1"]["T_cn_cnm1"])
    def camera(which):
        fx, fy, cx, cy = cfg[which]["intrinsics"]
        return np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
    candidates = []
    for label, transform in [("as_stored", matrix), ("inverse_as_in_inspected_script", np.linalg.inv(matrix))]:
        r1, r2, p1, p2, *_ = cv2.stereoRectify(
            camera("cam0"), np.array(cfg["cam0"]["distortion_coeffs"]),
            camera("cam1"), np.array(cfg["cam1"]["distortion_coeffs"]),
            tuple(cfg["cam0"]["resolution"]), np.ascontiguousarray(transform[:3, :3]),
            np.ascontiguousarray(transform[:3, 3]).reshape(3, 1),
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        candidates.append({"matrix_direction": label,
                           "left_R_max_abs_difference": float(np.max(np.abs(r1 - np.array(left["r"])))),
                           "right_R_max_abs_difference": float(np.max(np.abs(r2 - np.array(right["r"])))),
                           "left_P_max_abs_difference": float(np.max(np.abs(p1 - np.array(left["p"])))),
                           "right_P_max_abs_difference": float(np.max(np.abs(p2 - np.array(right["p"])))),
                           "signed_projection_baseline_m": float(-p2[0, 3] / p2[0, 0])})
    return {"source": str(thermal_config), "opencv": cv2.__version__, "candidates": candidates,
            "status": "producer-direction reproduction; not independent calibration validation"}


def audit(dataset, invert_serialized=False):
    dataset = Path(dataset)
    metadata = yaml.safe_load((dataset / "tf/metadata.yaml").read_text())
    if "transforms" in metadata:
        edges = [(e["src_frame"], e["dst_frame"], e["static"])
                 for e in metadata["transforms"]]
        schema = "transforms/src_frame/dst_frame"
    else:
        edges = [(e["parent"], e["frame"], e["static"]) for e in metadata["frames"]]
        schema = "frames/parent/frame"
    parents = defaultdict(list)
    adjacency = defaultdict(list)
    static_edges, dynamic_edges = [], []
    for parent, child, static in edges:
        parents[child].append(parent)
        folder = dataset / "tf" / f"{parent.replace('/', '-')}_to_{child.replace('/', '-')}"
        if not static:
            dynamic_edges.append({"parent": parent, "child": child,
                                  "samples": int(np.atleast_1d(np.loadtxt(folder / 'timestamps.txt')).size)})
            continue
        matrix = pose_matrix(np.loadtxt(folder / "static_transform.txt"))
        if invert_serialized:
            matrix = np.linalg.inv(matrix)
        static_edges.append((parent, child, matrix))
        # Convention: matrix is T_parent<-child, as in the inspected TfManager.
        adjacency[parent].append((child, matrix))
        adjacency[child].append((parent, np.linalg.inv(matrix)))
    poses, components = {}, {}
    for root in adjacency:
        if root in poses:
            continue
        poses[root] = np.eye(4); components[root] = root
        queue = deque([root])
        while queue:
            parent = queue.popleft()
            for child, matrix in adjacency[parent]:
                if child not in poses:
                    poses[child] = poses[parent] @ matrix
                    components[child] = root
                    queue.append(child)
    residuals = []
    for parent, child, matrix in static_edges:
        residual = np.linalg.inv(poses[parent] @ matrix) @ poses[child]
        residuals.append({"parent": parent, "child": child,
                          "translation_m": float(np.linalg.norm(residual[:3, 3])),
                          "rotation_deg": float(Rotation.from_matrix(residual[:3, :3]).magnitude() * 180 / np.pi)})
    report = {"schema": schema,
              "serialized_convention_hypothesis": "T_child<-parent" if invert_serialized else "T_parent<-child",
              "multiple_parents": {k: v for k, v in parents.items() if len(set(v)) > 1},
              "static_edges": len(static_edges), "dynamic_edges": dynamic_edges,
              "independent_static_cycles": len(static_edges) - len(poses) + len(set(components.values())),
              "static_constraint_residuals": residuals,
              "static_consistency": {"translation_tolerance_m": 1e-4,
                                     "rotation_tolerance_deg": 1e-3,
                                     "passes": all(r["translation_m"] <= 1e-4 and r["rotation_deg"] <= 1e-3 for r in residuals)},
              "stereo": {}, "status": "geometry audit only; image rectification applicability requires verification"}
    settings = {
        "rgb": ("multisense/left_camera_optical_frame", "multisense/right_camera_optical_frame",
                "multisense_left_rect_color_camera_info", "multisense_right_camera_info"),
        "thermal": ("thermal_left/optical_frame", "thermal_right/optical_frame",
                    "thermal_left_processed_camera_info", "thermal_right_processed_camera_info"),
    }
    for modality, (left, right, left_info, right_info) in settings.items():
        l = yaml.safe_load((dataset / "sensors" / left_info / "data.yaml").read_text())
        r = yaml.safe_load((dataset / "sensors" / right_info / "data.yaml").read_text())
        lp, rp = np.asarray(l["p"]), np.asarray(r["p"])
        item = {"left_raw_K": l["k"], "left_rectified_K": lp[:, :3].tolist(),
                "left_rectification_R": l["r"], "right_rectification_R": r["r"],
                "projection_baseline_m_signed": float(-rp[0, 3] / rp[0, 0] + lp[0, 3] / lp[0, 0])}
        if left in poses and right in poses and components[left] == components[right]:
            relative = np.linalg.inv(poses[left]) @ poses[right]
            item["T_left_from_right_from_static_constraints"] = relative.tolist()
            item["tf_baseline_norm_m"] = float(np.linalg.norm(relative[:3, 3]))
            item["rectified_right_origin_in_left_m"] = (np.asarray(l["r"]) @ relative[:3, 3]).tolist()
            item["baseline_norm_difference_m"] = abs(item["tf_baseline_norm_m"] - abs(item["projection_baseline_m_signed"]))
            item["projection_consistency_residuals"] = rectification_residual(relative, l, r)
        report["stereo"][modality] = item
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--thermal-config", type=Path)
    args = parser.parse_args()
    report = {"status": "unverified; neither convention is selected automatically",
              "hypotheses": [audit(args.dataset), audit(args.dataset, invert_serialized=True)]}
    if args.calibration:
        report["supplied_calibration"] = compare_calibration(args.dataset, args.calibration)
    if args.thermal_config:
        report["thermal_rectification_reproduction"] = compare_thermal_rectification(args.dataset, args.thermal_config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
