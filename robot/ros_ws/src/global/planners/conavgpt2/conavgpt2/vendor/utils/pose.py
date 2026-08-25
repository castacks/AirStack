import numpy as np


def get_l2_distance(x1, x2, y1, y2):
    """
    Computes the L2 distance between two points.
    """
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_rel_pose_change(pos2, pos1):
    x1, y1, o1 = pos1
    x2, y2, o2 = pos2

    theta = np.arctan2(y2 - y1, x2 - x1) - o1
    dist = get_l2_distance(x1, x2, y1, y2)
    dx = dist * np.cos(theta)
    dy = dist * np.sin(theta)
    do = o2 - o1

    return dx, dy, do


def get_new_pose(pose, rel_pose_change):
    x, y, o = pose
    dx, dy, do = rel_pose_change

    global_dx = dx * np.sin(np.deg2rad(o)) + dy * np.cos(np.deg2rad(o))
    global_dy = dx * np.cos(np.deg2rad(o)) - dy * np.sin(np.deg2rad(o))
    x += global_dy
    y += global_dx
    o += np.rad2deg(do)
    if o > 180.:
        o -= 360.

    return x, y, o


def threshold_poses(coords, shape):
    coords[0] = min(max(0, coords[0]), shape[0] - 1)
    coords[1] = min(max(0, coords[1]), shape[1] - 1)
    return coords


def roll_array(arr, shift, axis=0, fill_value=0):
    """
    Roll an array along a specified axis and clear the removed part.

    Parameters:
    - arr: numpy.ndarray, the array to roll.
    - shift: int, the number of places to shift the array. Positive values shift forwards, negative values shift backwards.
    - axis: int, the axis along which to roll (0 for rows, 1 for columns).
    - fill_value: scalar, the value to use to fill the cleared part.

    Returns:
    - numpy.ndarray, the rolled array with cleared part.
    """
    # Roll the array
    result = np.roll(arr, shift, axis=axis)

    # Clear the part that wrapped around
    if axis == 0:  # Rolling rows
        if shift > 0:
            result[:shift, :] = fill_value
        else:
            result[shift:, :] = fill_value
    elif axis == 1:  # Rolling columns
        if shift > 0:
            result[:, :shift] = fill_value
        else:
            result[:, shift:] = fill_value

    return result

def roll_pose(pos, shift, axis=0, fill_value=0):
    if axis == 0:
        pos[0] += shift
    elif axis == 1:
        pos[1] += shift
        
    return pos