#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

c = np.array([5.0, 1.0])
ps = []
r = 3.0

for a in np.arange(-np.pi / 2.0, np.pi / 2.0 + 0.01, np.pi / 8.0):
    p = np.array([c[0] + r * np.cos(a), c[1] + r * np.sin(a)])
    ps.append(p)

    angle = (np.arctan((p[1] - c[1]) / (p[0] - c[0])) + np.pi / 2.0) / np.pi * 1000.0
    new_angle = angle / 1000.0 * np.pi - np.pi / 2.0
    # point = np.array([c[0] + r*np.cos(new_angle), c[1] + r*np.sin(new_angle)])
    center = np.array([p[0] - r * np.cos(new_angle), p[1] - r * np.sin(new_angle)])

    print(a, angle, new_angle, c, center)


ps = np.array(ps)

plt.axis("equal")
plt.plot(c[0], c[1], "o")
plt.plot(ps[:, 0], ps[:, 1], "o")
plt.show()
