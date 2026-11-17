"""Bounded resolution-seeking policy for the restricted static red fixture.

Inputs are observations, not scene truth. Oracle depth and a static pinhole
approximation do not establish semantic information gain or object localization.
"""
import math
import statistics
from collections.abc import Mapping


def optical_forward_in_map(quaternion_xyzw):
    if len(quaternion_xyzw) != 4 or not all(
        isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
        for x in quaternion_xyzw
    ):
        raise ValueError('finite quaternion required')
    norm = math.sqrt(sum(x*x for x in quaternion_xyzw))
    if abs(norm-1) > .01:
        raise ValueError('unit quaternion required')
    x, y, z, w = [v/norm for v in quaternion_xyzw]
    return [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]


def select_viewpoint(observations, image_size, optical_axis_map):
    """Select one horizontal translation, or explicitly abstain without a goal."""
    def abstain(reason):
        return dict(status='ABSTAIN', reason=reason, delta_map_m=None)

    if not isinstance(observations, (list, tuple)) or len(observations) != 3:
        return abstain('requires_three_observations')
    if not isinstance(image_size, (list, tuple)) or len(image_size) != 2 or any(type(v) is not int or v <= 16 for v in image_size):
        return abstain('invalid_image_size')
    if not isinstance(optical_axis_map, (list, tuple)) or len(optical_axis_map) != 3 or not all(
        isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
        for v in optical_axis_map
    ):
        return abstain('invalid_optical_axis')
    if abs(math.sqrt(sum(v*v for v in optical_axis_map))-1) > .01 or abs(optical_axis_map[2]) > .15:
        return abstain('unqualified_camera_orientation')
    areas = []
    depths = []
    boxes = []
    for observation in observations:
        if not isinstance(observation, Mapping):
            return abstain('invalid_observation')
        if observation.get('status') != 'OBSERVED':
            return abstain('unobserved_or_ambiguous_target')
        box = observation.get('bbox_xywh', [])
        point = observation.get('surface_point_optical_m', [])
        depth = observation.get('surface_depth_z_m')
        if not isinstance(box, (list, tuple)) or len(box) != 4 or any(type(v) is not int for v in box):
            return abstain('invalid_bbox')
        x, y, width, height = box
        if width < 3 or height < 3 or x < 8 or y < 8 or x+width > image_size[0]-8 or y+height > image_size[1]-8:
            return abstain('cropped_or_small_target')
        if not isinstance(point, (list, tuple)) or len(point) != 3 or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            for v in (*point, depth)
        ) or not 0 < depth < 20 or abs(point[2]-depth) > 1e-6:
            return abstain('invalid_surface_range')
        if max(abs(point[0]/depth), abs(point[1]/depth)) > .5:
            return abstain('off_center_target')
        areas.append(width*height)
        depths.append(depth)
        boxes.append((x, y, width, height))
    if max(depths)-min(depths) > .1 or max(areas)/min(areas) > 1.25:
        return abstain('unstable_observation_window')
    min_iou = 1.0
    for index, (x, y, width, height) in enumerate(boxes):
        for other_x, other_y, other_width, other_height in boxes[index+1:]:
            overlap_width = max(0, min(x+width, other_x+other_width)-max(x, other_x))
            overlap_height = max(0, min(y+height, other_y+other_height)-max(y, other_y))
            intersection = overlap_width*overlap_height
            union = width*height+other_width*other_height-intersection
            min_iou = min(min_iou, intersection/union)
    if min_iou < .5:
        return dict(**abstain('spatially_unstable_observation_window'),
                    min_window_bbox_iou=min_iou, min_required_bbox_iou=.5)
    index = min(range(3), key=lambda i: abs(areas[i]-statistics.median(areas)))
    z = depths[index]
    horizontal = math.hypot(*optical_axis_map[:2])
    step = min(.75, (z-z/math.sqrt(1.5))/horizontal, (z-2.5)/horizontal)
    if step < .1:
        return abstain('insufficient_standoff_or_motion_budget')
    predicted_z = z-step*horizontal
    predicted_ratio = (z/predicted_z)**2
    if predicted_ratio < 1.2:
        return abstain('insufficient_predicted_resolution_gain')
    return dict(status='SELECTED', selected_observation_index=index,
                delta_map_m=[step*optical_axis_map[0]/horizontal,
                             step*optical_axis_map[1]/horizontal, 0.0],
                translation_m=step, predicted_surface_depth_z_m=predicted_z,
                predicted_bbox_area_ratio=predicted_ratio,
                min_window_bbox_iou=min_iou,
                limits=dict(max_translation_m=.75, min_optical_standoff_m=2.5,
                            min_window_bbox_iou=.5),
                scope='static red fixture, oracle depth, frontal pinhole resolution proxy')
