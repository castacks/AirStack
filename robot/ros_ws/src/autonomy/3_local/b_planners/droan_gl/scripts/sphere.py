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

# Image size
width = 480
height = 270

# Camera parameters
cx = width/2   # principal point x
cy = height/2   # principal point y
fx = 233   # focal length x
fy = 233   # focal length y
baseline = 0.12  # stereo baseline in meters


# Create pixel coordinate grid
u = np.arange(width)
v = np.arange(height)
u, v = np.meshgrid(u, v)

# Simulate disparity values (for example, closer in the center)
# You can replace this with a real disparity map or depth formula.
#disparity = 30 + 10 * np.exp(-((u - cx)**2 + (v - cy)**2) / (2 * (100**2)))

c_u = cx
c_v = cy

a_0 = (c_u - cx) / fx
a = (u - cx) / fx

b_0 = (c_v - cy) / fy
b = (v - cy) / fy

Zc = fx*baseline/2.
Xc = a_0*Zc
Yc = b_0*Zc
expansion_radius = 2.

A = a*a + b*b + 1.
B = -2.*Zc*(a*a_0 + b*b_0 + 1.)
C = Zc*Zc*(a_0*a_0 + b_0*b_0 + 1.) - expansion_radius*expansion_radius

disc = B*B - 4.*A*C
#mask = np.where(disc > 0.)
mask = np.where(np.logical_and(disc > 0., v == cy))
A = A[mask]
B = B[mask]
u = u[mask]
v = v[mask]
disc = disc[mask]
Zp = (-B - np.sqrt(disc))/(2.*A)
disparity = fx*baseline/Zp

xs = []
ys = []
zs = []

nan_disc_count = 0
for ut in range(0, width):
    v_cur = cy
    a_0 = (c_u - cx) / fx
    a = (ut - cx) / fx
    b_0 = (c_v - cy) / fy
    b = (v_cur - cy) / fy
    A = a*a + b*b + 1.
    B = -2.*Zc*(a*a_0 + b*b_0 + 1.)
    C = Zc*Zc*(a_0*a_0 + b_0*b_0 + 1.) - expansion_radius*expansion_radius
    disc = B*B - 4.*A*C
    Zp = (-B - np.sqrt(disc))/(2.*A)
    disp = fx*baseline/Zp
    if disc <= 0.:
        continue
    print('A B C', A, B, C, 'a a_0', a, a_0, 'b b_0', b, b_0)
    print('Zp disp', Zp, disp)

    z = fx*baseline/disp
    x = (ut - cx)*z/fx
    y = (v_cur - cy)*z/fy
    xs.append(x)
    ys.append(y)
    zs.append(z)

    Xp = a*Zp
    angle = np.arctan((Xp - Xc)/(Zp - Zc))
    
    depth = fx*baseline/disp
    Zc_cur = depth + expansion_radius*np.cos(angle)
    Xc_cur = (ut - cx)*depth/fx + expansion_radius*np.sin(angle)
    print(Zc, Zc_cur, Xc, Xc_cur)

    a_0 = Xc_cur/Zc_cur
    uc = fx*a_0 + cx
    vc = fy*b_0 + cy
    
    ru = fx*expansion_radius/Zc_cur
    rv = fy*expansion_radius/Zc_cur

    vmin = vc - rv*np.sqrt(1. - ((ut - uc)**2/(ru*ru)))
    vmax = vc + rv*np.sqrt(1. - ((ut - uc)**2/(ru*ru)))
    print(vmin, vmax)
    if(vmin != vmin or vmax != vmax):
        continue

    for vt in range(int(np.floor(vmin)), int(np.ceil(vmax))):
    #for vt in range(0, height):
        #a_0 = (c_u - cx) / fx
        a = (ut - cx) / fx
        b_0 = (c_v - cy) / fy
        b = (vt - cy) / fy
        A = a*a + b*b + 1.
        B = -2.*Zc_cur*(a*a_0 + b*b_0 + 1.)
        C = Zc_cur*Zc_cur*(a_0*a_0 + b_0*b_0 + 1.) - expansion_radius*expansion_radius
        Zp = (-B - np.sqrt(B*B - 4.*A*C ))/(2.*A)
        print('disc', B*B - 4.*A*C, np.sqrt(B*B - 4.*A*C))
        if (B*B - 4.*A*C) < 0.:
            nan_disc_count += 1
        disp = fx*baseline/Zp

        z = fx*baseline/disp
        x = (ut - cx)*z/fx
        y = (vt - cy)*z/fy
        #if Zc_cur == Zc_cur:
        #    print(a_0, a, b_0, b, A, B, C, Zp, disp, B*B - 4.*A*C )
        xs.append(x)
        ys.append(y)
        zs.append(z)
print('nan', nan_disc_count)
    

# Compute depth from disparity (Z = fx * baseline / disparity)
Z = fx * baseline / disparity

# Compute 3D coordinates in camera space
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy

# Flatten for scatter plot
X = X.flatten()
Y = Y.flatten()
Z = Z.flatten()
#X = X[np.where(disc > 0.)].flatten()
#Y = Y[np.where(disc > 0.)].flatten()
#Z = Z[np.where(disc > 0.)].flatten()

# Subsample for performance (optional)
#sample = np.random.choice(len(X), size=20000, replace=False)
#X, Y, Z = X[sample], Y[sample], Z[sample]

# Plot point cloud
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(X, Y, Z, c=Z, cmap='viridis', s=1)
sc = ax.scatter(xs, ys, zs, s=1)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
set_axes_equal(ax)
plt.title("3D Point Cloud from Depth Calculation")
plt.show()
