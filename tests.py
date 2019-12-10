import libs.FileIO as FileIO
import libs.visu as visu
import sys
import open3d as o3d

location=sys.argv[1]

ola = FileIO.getFromPickle(location + "poses.pickle")

print(ola)


FileIO.SaveAsMat(ola,location + "poses.mat")

quit()

#string ="/./Logs/2019-09-07_23:26:54_mongoose.json"

#ola = string.split('/')

#print(ola[len(ola)-1])

aruco   = FileIO.getFromPickle(sys.argv[1])

corners = aruco['corners']
R = aruco['R']
t = aruco['t']

geo = visu.SeePositions(corners,view=False)
geo = visu.ViewRefs(R,t,0.03,view=False)
#visu.draw_geometry(geo)

lines = []
for i in range(corners.shape[0]):

    if (i+1)%4==0:
        lines.append([i,i-3])
        continue 
    lines.append([i,i+1])

colors = [[1, 0, 0] for i in range(len(lines))]

line_set = o3d.geometry.LineSet()
line_set.points = o3d.Vector3dVector(corners)
line_set.lines = o3d.Vector2iVector(lines)
line_set.colors = o3d.Vector3dVector(colors)
o3d.draw_geometries([line_set]+geo)