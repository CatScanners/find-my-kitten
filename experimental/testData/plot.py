import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

filename = sys.argv[1]

if not filename:
    raise RuntimeError("Provide filename via argument")

# Lists to store coordinates
xs, ys, zs = [], [], []

# Regular expression to extract numbers from lines like: { x: 0.0314, y: 0.02, z: 1.00 }
pattern = re.compile(r"x:\s*([-0-9.]+).*?y:\s*([-0-9.]+).*?z:\s*([-0-9.]+)")

with open(filename, "r") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            x, y, z = map(float, match.groups())
            xs.append(x)
            ys.append(y)
            zs.append(z)

# Plotting
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(xs, ys, zs, s=8, c='blue')

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("height locked path.")

plt.show()
