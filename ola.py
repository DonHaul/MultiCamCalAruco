from libs import *
import open3d 
import numpy as np


print("Testing IO for meshes ...")
mesh = open3d.read_triangle_mesh("./static/camera.ply")
refe = open3d.create_mesh_coordinate_frame(0.1, origin = [0, 0, 0])
H=mmnip.Rt2Homo(mmnip.genRotMat([90,0,90]),np.array([-0.09,0,0]))
print(H)
mesh.transform(H)

open3d.draw_geometries([mesh]+[refe])

