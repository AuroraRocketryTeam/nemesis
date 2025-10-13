import csv
import matplotlib.pyplot as plt

FILE_NAME = "rocket_positions.csv"

times = []
x_vals = []
y_vals = []
z_vals = []

with open(FILE_NAME, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row["time"]))
        x_vals.append(float(row["x"]))
        y_vals.append(float(row["y"]))
        z_vals.append(float(row["z"]))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_vals, y_vals, z_vals)

ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_zlabel("Z Position (m)")
ax.set_title("Rocket Trajectory")

plt.show()
