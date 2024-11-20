from matplotlib import pyplot as plt
import numpy as np
import os

# Define the file path
file_path = os.path.join("/home/zed/zed-ws/src/scout_robot_project/scout_robot__2dnav/gps_carts", 'xy.txt')
# file_path1 = os.path.join("/home/zed/zed-ws/src/lidar_clip/gps_carts", 'xy1.txt')

# Read the points from the file
points = np.loadtxt(file_path, delimiter=',')
# points1 = np.loadtxt(file_path1, delimiter=',')

# Separate the points into x and y coordinates
x_coords = points[:, 0]
# x_coords1 = points1[:, 0]
y_coords = points[:, 1]
# y_coords1 = points1[:, 1]

# Create a scatter plot
plt.scatter(x_coords, y_coords, color='blue', marker='o')
# plt.scatter(x_coords1, y_coords1, color='green', marker='o')
# plt.scatter(x_coords[:5], y_coords[:5], color='red', marker='x')
# Add labels and title
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('GPS Cartesian Coordinates')

# Show the plot
plt.grid(True)
plt.show()