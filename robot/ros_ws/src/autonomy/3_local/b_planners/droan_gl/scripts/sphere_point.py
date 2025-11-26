#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

def test():
    c = np.array([3., 1.])
    ps = []
    
    for a in np.arange(-np.pi/2, np.pi/2.+.01, np.pi/8.):
        angle = a
        #angle = 0.
        p = c + np.array([np.cos(angle), np.sin(angle)])

        ang = np.arctan((p[1] - c[1])/(p[0] - c[0]))
        ang = (ang + np.pi/2.)/np.pi*1000. # 0 to 1000
        ang = int(ang)
        print(ang)
        ang = float(ang)
        ang = ang/1000.*np.pi - np.pi/2.
        print(angle, ang)

        ps.append(p)

    ps = np.array(ps)
    print(ps)
    plt.plot(c[0], c[1], 'o')
    plt.plot(ps[:, 0], ps[:, 1], '.')
    plt.axis('equal')
    plt.show()


test()

def orthonormal_equator_basis(n, r_ref=np.array([0.0,0.0,-1.0]), eps=1e-12):
    """
    Return orthonormal basis (e1, e2) spanning the plane orthogonal to n.
    Prefer to use the projection of r_ref into that plane. If projection is
    (near) zero (r_ref parallel/opposite to n), choose a stable fallback.
    n must be unit (or will be normalized).
    """
    n = np.array(n, dtype=float)
    n_norm = np.linalg.norm(n)
    if n_norm == 0:
        raise ValueError("n must be non-zero")
    n = n / n_norm

    # Try projection of r_ref
    r_ref = np.array(r_ref, dtype=float)
    u = r_ref - np.dot(r_ref, n) * n
    u_norm = np.linalg.norm(u)

    if u_norm > eps:
        e1 = u / u_norm
    else:
        # r_ref was parallel/opposite to n -> pick a fallback axis:
        # choose the coordinate axis least aligned with n
        abs_n = np.abs(n)
        # index of smallest component
        idx = int(np.argmin(abs_n))
        # unit vector along that coordinate axis
        axis = np.zeros(3)
        axis[idx] = 1.0
        # e1 is perpendicular to n: cross(n, axis) (and normalize)
        e1 = np.cross(n, axis)
        e1 = e1 / np.linalg.norm(e1)

    e2 = np.cross(n, e1)  # already unit if n,e1 are orthonormal
    #e2 *= -1
    e1 = np.array([0,0,-1])
    e2 = np.array([1,0,0])
    print('e', e1, e2)
    return e1, e2

def compute_center_equator(P, R, phi, n=np.array([0.0,0.0,1.0]), r_ref=np.array([0.0,0.0,-1.0])):
    """
    Compute sphere center from:
      P: point on equator (3,)
      R: radius
      phi: angle from 'top' (radians)
      n: sphere axis (unit ideally)
      r_ref: reference direction for 'top' (default camera-back)
    """
    e1, e2 = orthonormal_equator_basis(n, r_ref)
    C = P - R * (np.cos(phi) * e1 + np.sin(phi) * e2)
    return C, e1, e2

# ----------------------------
# Example: generate and recover
# ----------------------------
R = 1.0
phi_true = np.deg2rad(10.0)

# True center (camera coords) and axis
C_true = np.array([0.5, 0.0, 3.0])
n = np.array([0.0, 1.0, 0.0])   # upright sphere (this makes r_ref parallel/opposite)

# Build equator basis robustly (this will use fallback since r_ref is opposite n)
e1, e2 = orthonormal_equator_basis(n, r_ref=np.array([0.0,0.0,-1.0]))
# top and point P on equator at phi_true
P_top = C_true + R * e1
P = C_true + R * (np.cos(phi_true) * e1 + np.sin(phi_true) * e2)

# Recover center from P, R, phi_true
C_est, e1_est, e2_est = compute_center_equator(P, R, phi_true, n=n, r_ref=np.array([0.0,0.0,-1.0]))

print("True center:     ", C_true)
print("Estimated center:", C_est)
print("Reconstruction error norm:", np.linalg.norm(C_est - C_true))

# ----------------------------
# Plot for verification
# ----------------------------
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')

# Sphere surface
u_grid, v_grid = np.mgrid[0:2*np.pi:60j, 0:np.pi:30j]
xs = C_true[0] + R * np.cos(u_grid) * np.sin(v_grid)
ys = C_true[1] + R * np.sin(u_grid) * np.sin(v_grid)
zs = C_true[2] + R * np.cos(v_grid)
ax.plot_surface(xs, ys, zs, color='lightblue', alpha=0.4, linewidth=0)

# Points and vectors
ax.scatter(*C_true, color='k', label='True Center', s=60)
ax.scatter(*P_top, color='g', label='Top (phi=0)', s=50)
ax.scatter(*P, color='r', label='Equator point P', s=50)
ax.scatter(*C_est, color='orange', label='Recovered Center', s=80, marker='x')

# show e1/e2 basis at center
scale = 0.5
ax.quiver(*(C_true), *(e1*scale), color='g', length=scale, normalize=False)
ax.quiver(*(C_true), *(e2*scale), color='m', length=scale, normalize=False)

ax.set_xlabel('X (right)')
ax.set_ylabel('Y (down)')
ax.set_zlabel('Z (forward)')
ax.set_box_aspect([1,1,1])
ax.legend()
ax.set_title('Robust center recovery (handles r_ref || n)')
plt.show()
