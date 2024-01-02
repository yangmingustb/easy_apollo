
python3 -c "import open3d as o3d; print(o3d.__version__)"

# Python API
python3 -c "import open3d as o3d; \
           mesh = o3d.geometry.TriangleMesh.create_sphere(); \
           mesh.compute_vertex_normals(); \
           o3d.visualization.draw(mesh, raw_mode=True)"

# Open3D CLI
open3d example visualization/draw