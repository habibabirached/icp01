# test_open3d.py
import open3d as o3d
import numpy as np

# Simple test to verify that Open3D works
print("Open3D version:", o3d.__version__)

# Create a simple point cloud
point_cloud = o3d.geometry.PointCloud()
points = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
point_cloud.points = o3d.utility.Vector3dVector(points)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])
