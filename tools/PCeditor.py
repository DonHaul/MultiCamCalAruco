import open3d

pcd = open3d.read_point_cloud("../tmp/wow.ply")


open3d.draw_geometries([pcd])

print("Downsample the point cloud with a voxel of 0.05")
downpcd = open3d.voxel_down_sample(pcd, voxel_size = 0.05)
open3d.draw_geometries([downpcd])

print("lol")

open3d.draw_geometries_with_editing([pcd])