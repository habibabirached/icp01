import open3d as o3d
import numpy as np

# Simple test to verify that Open3D works
print("Open3D versionnnnnn2233322:", o3d.__version__)

# Create a simple point cloud
point_cloud = o3d.geometry.PointCloud()
points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
point_cloud.points = o3d.utility.Vector3dVector(points)

print("After point_cloud.points = o3d.utility.Vector3dVector(points)")

# Start the WebRTC server
if o3d.visualization.webrtc_server.enable_webrtc():
    print("WebRTC server enabled successfully.")
else:
    print("Failed to enable WebRTC server.")

# Visualize the point cloud using WebRTC server
o3d.visualization.draw([point_cloud], show_ui=True)

print("I am done")
