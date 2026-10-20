import numpy as np


def figure_eight_reference(t, amplitude_x, amplitude_y, period, altitude, altitude_amplitude):
    theta = 2.0 * np.pi * t / period
    theta_dot = 2.0 * np.pi / period
    position = np.array([amplitude_x * np.sin(theta), 0.5 * amplitude_y * np.sin(2.0 * theta), altitude + altitude_amplitude * np.sin(theta)])
    velocity = np.array([amplitude_x * np.cos(theta) * theta_dot, amplitude_y * np.cos(2.0 * theta) * theta_dot, altitude_amplitude * np.cos(theta) * theta_dot])
    return position, velocity


def quaternion_to_rotation_matrix(quaternion):
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def outer_loop_command(position, rotation, reference_position, reference_velocity, yaw_mode, position_gain, yaw_gain, max_speed, max_yaw_rate,
                       integral=None, integral_gain=0.0, integral_limit=0.0, dt=0.0, vertical_gain=None):
    # Position error in the world frame becomes a world velocity, then rotates into the body frame the policy was trained in.
    # Optional integral term on the horizontal position error: a memoryless policy holds a steady velocity error against a
    # steady disturbance, which a P-only outer loop turns into a constant position bias; the integral winds it out. The
    # integral state is owned by the caller (reset when the loop is enabled), accumulates only while the P+ff command is not
    # saturated, and its contribution is clamped to +/-integral_limit m/s. Vertical and yaw are never integrated.
    position_error = reference_position - position
    gains = np.array([position_gain, position_gain, position_gain if vertical_gain is None or vertical_gain < 0.0 else vertical_gain])
    world_velocity = reference_velocity + gains * position_error
    if integral is not None and integral_gain > 0.0:
        if np.linalg.norm(world_velocity) < max_speed:
            integral[:2] += position_error[:2] * dt
            bound = integral_limit / integral_gain if integral_gain > 0.0 else 0.0
            integral[:2] = np.clip(integral[:2], -bound, bound)
        world_velocity = world_velocity + integral_gain * np.array([integral[0], integral[1], 0.0])
    speed = np.linalg.norm(world_velocity)
    if speed > max_speed:
        world_velocity *= max_speed / speed
    body_velocity = rotation.T @ world_velocity
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    if yaw_mode == "tangent" and np.linalg.norm(reference_velocity[:2]) > 1e-3:
        reference_yaw = np.arctan2(reference_velocity[1], reference_velocity[0])
    else:
        reference_yaw = 0.0
    yaw_error = np.arctan2(np.sin(reference_yaw - yaw), np.cos(reference_yaw - yaw))
    yaw_rate = float(np.clip(yaw_gain * yaw_error, -max_yaw_rate, max_yaw_rate))
    return np.array([body_velocity[0], body_velocity[1], body_velocity[2], yaw_rate], dtype=np.float32)
