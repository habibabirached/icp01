import open3d as o3d

def read_and_scale_stl(stl_path):
    try:
        print(f"Reading STL file from {stl_path}...")
        mesh = o3d.io.read_triangle_mesh(stl_path)
        print("STL file successfully read.")
        print (mesh)
        
        # Scale and translate the mesh
        print("Scaling the mesh...")
        mesh.scale(1.0 / 1000.0, center=(0, 0, 0))
        print("Scaling completed.")
        
        print("Translating the mesh...")
        mesh.translate((0, 0, 0))  # Adjust this translation as needed
        print("Translation completed.")

        return mesh
    except Exception as e:
        print(f"Error: {e}")
        return None

if __name__ == '__main__':
    stl_path = './models/cube.STL'
    mesh = read_and_scale_stl(stl_path)

    if mesh:
        # Optionally visualize the mesh to confirm
        o3d.visualization.draw_geometries([mesh])
