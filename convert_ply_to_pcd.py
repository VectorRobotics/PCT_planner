import open3d as o3d

# Define input and output file paths
ply_file_path = "/home/alex-lin/Downloads/office_building_1/map.ply"
pcd_file_path = "/home/alex-lin/Downloads/office_building_1/map.pcd"

# Read the PLY point cloud
print(f"Reading {ply_file_path}...")
pcd = o3d.io.read_point_cloud(ply_file_path)
print(f"Successfully read point cloud with {len(pcd.points)} points.")

# Write the point cloud to a PCD file
print(f"Writing to {pcd_file_path}...")
o3d.io.write_point_cloud(pcd_file_path, pcd)
print("Conversion complete.")