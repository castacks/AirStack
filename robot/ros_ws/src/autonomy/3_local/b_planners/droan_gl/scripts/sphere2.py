#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt


def set_axes_equal(ax):
    """Set 3D plot axes to equal scale so spheres look like spheres."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    max_range = max([x_range, y_range, z_range]) / 2.0

    mid_x = np.mean(x_limits)
    mid_y = np.mean(y_limits)
    mid_z = np.mean(z_limits)

    ax.set_xlim3d([mid_x - max_range, mid_x + max_range])
    ax.set_ylim3d([mid_y - max_range, mid_y + max_range])
    ax.set_zlim3d([mid_z - max_range, mid_z + max_range])


def set_axes_equal_so(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


xs = []
ys = []
zs = []

# Image size
width = 480
height = 270

# Camera parameters
cx = float(width) / 2.0  # principal point x
cy = float(height) / 2.0  # principal point y
fx = 233.82684326171875  # focal length x
fy = 233.8268585205078  # focal length y
baseline = 0.12  # stereo baseline in meters
expansion_radius = 2.0
scale = 1000000.0


def horizontal(coord_x, coord_y, center, is_fg):
    center_depth = fx * baseline / center
    radius = int(expansion_radius * center / baseline)

    a_0 = (coord_x - cx) / fx
    b = (coord_y - cy) / fy
    b_0 = (coord_y - cy) / fy

    Zc = center_depth
    Xc = a_0 * Zc

    for dx in range(-radius, radius + 1):
        px = coord_x + dx
        py = coord_y

        if px < 0 or px >= width:
            continue

        a = (px - cx) / fx
        A = a * a + b * b + 1
        B = -2.0 * Zc * (a * a_0 + b * b_0 + 1)
        C = Zc * Zc * (a_0 * a_0 + b_0 * b_0 + 1) - expansion_radius * expansion_radius

        sign = -1.0
        if not is_fg:
            sign = 1.0
        Zp = (-B + sign * np.sqrt(B * B - 4.0 * A * C)) / (2.0 * A)
        new_disp = int(scale * fx * baseline / Zp)

        Xp = a * Zp
        raw_angle = np.arctan((Xp - Xc) / (Zp - Zc))
        angle = int((raw_angle + np.pi / 2.0) / np.pi * 1000.0)
        new_disp = (new_disp & ~0x3FF) | (angle & 0x3FF)

        # print('h', raw_angle)
        vertical(px, py, new_disp, is_fg)

        # visualization
        z = fx * baseline / (float(new_disp) / scale)
        x = (px - cx) * z / fx
        y = (py - cy) * z / fy
        xs.append(x)
        ys.append(y)
        zs.append(z)


def vertical(coord_x, coord_y, center, is_fg):
    center_depth = fx * baseline / (float(center) / scale)
    radius = int(expansion_radius + float(center) / scale / baseline)
    a = (coord_x - cx) / fx
    b_0 = (coord_y - cy) / fy

    angle = float(center & 0x3FF) / 1000.0 * np.pi - np.pi / 2.0
    # print('v', angle)
    Zc = center_depth + expansion_radius * np.cos(angle)
    Xc = (coord_x - cx) * center_depth / fx + expansion_radius * np.sin(angle)
    if not is_fg:
        Zc = center_depth - expansion_radius * np.cos(angle)
        Xc = (coord_x - cx) * center_depth / fx - expansion_radius * np.sin(angle)
    a_0 = Xc / Zc

    uc = fx * a_0 + cx
    vc = fy * b_0 + cy
    ru = fx * expansion_radius / Zc
    rv = fy * expansion_radius / Zc

    ymin = int(
        np.floor(vc - rv * np.sqrt(1.0 - ((coord_x - uc) * (coord_x - uc)) / (ru * ru)))
    )
    ymax = int(
        np.ceil(vc + rv * np.sqrt(1.0 - ((coord_x - uc) * (coord_x - uc)) / (ru * ru)))
    )

    if not is_fg:
        print(ymin - ymax)

    for y in range(ymin, ymax + 1):
        px = coord_x
        py = y

        if py < 0 or py >= height:
            continue

        b = (py - cy) / fy
        A = a * a + b * b + 1
        B = -2.0 * Zc * (a * a_0 + b * b_0 + 1)
        C = Zc * Zc * (a_0 * a_0 + b_0 * b_0 + 1) - expansion_radius * expansion_radius
        if (B * B - 4.0 * A * C) < 0.0:
            continue

        sign = -1.0
        if not is_fg:
            sign = 1.0
        Zp = (-B + sign * np.sqrt(B * B - 4.0 * A * C)) / (2.0 * A)
        new_disp = fx * baseline / Zp

        z = fx * baseline / new_disp
        x = (px - cx) * z / fx
        y = (py - cy) * z / fy
        xs.append(x)
        ys.append(y)
        zs.append(z)


# coord_x = float(int(3*width/4))
# coord_y = float(int(height/2))
coord_x = float(int(width / 2))
coord_y = float(int(height / 2))
center = 2.0

horizontal(coord_x, coord_y, center, True)
horizontal(coord_x, coord_y, center, False)

# Plot point cloud
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
# sc = ax.scatter(X, Y, Z, c=Z, cmap='viridis', s=1)
sc = ax.scatter(xs, ys, zs, s=1)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
# set_axes_equal_so(ax)

# Create cubic bounding box to simulate equal aspect ratio
xs = np.array(xs)
ys = np.array(ys)
zs = np.array(zs)
max_range = np.array(
    [xs.max() - xs.min(), ys.max() - ys.min(), zs.max() - zs.min()]
).max()
Xb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5 * (
    xs.max() + xs.min()
)
Yb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5 * (
    ys.max() + ys.min()
)
Zb = 0.5 * max_range * np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5 * (
    zs.max() + zs.min()
)
# Comment or uncomment following both lines to test the fake bounding box:
for xb, yb, zb in zip(Xb, Yb, Zb):
    ax.plot([xb], [yb], [zb], "w")
ax.set_box_aspect((1, 1, 1))
plt.title("3D Point Cloud from Depth Calculation")
plt.show()
