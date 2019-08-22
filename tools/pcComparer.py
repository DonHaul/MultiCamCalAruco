#HERE WILL BE the v1, but organized in a good fashion

import rospy

import message_filters
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image
import cv2
import open3d
import numpy as np

import time

import struct

import commandline

import StateManager

import json

import open3d


from sensor_msgs.msg import PointCloud2

from libs import *

import sys


def main(argv):
    
    freq=10

    name ="jotaro"


    intrinsics = FileIO.getKDs([name])
    
    K = intrinsics['K'][name]
    print(K)
    rospy.init_node('ora_ora_ora_ORAA', anonymous=True)


    rgbRos = rospy.wait_for_message(name+"/rgb/image_color", Image)
    depthRos =  rospy.wait_for_message(name+"/depth_registered/image_raw", Image)
    pcRos = rospy.wait_for_message(name+"/depth_registered/points", PointCloud2)

    rgb = IRos.rosImg2RGB(rgbRos)
    depth_reg = IRos.rosImg2Depth(depthRos)

    
    cv2.imshow("wow",rgb)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    offset=2
    print(rgb.shape)
    heigth=480
    width = 640
    print(heigth/2)
    print(width/2)

    for i in range(width/2-offset,width/2+offset):
        for j in  range(heigth/2-offset,heigth/2+offset):
            rgb[j,i,:]=[255,0,255]
            
    cv2.imshow("wow",rgb)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    rgb1 = rgb.reshape((480*640, 3))#colors



    points = mmnip.depthimg2xyz2(depth_reg,K)
    points = points.reshape((480*640, 3))
    pcMade = pointclouder.Points2Cloud(points,rgb1)


    pc = point_cloud2.read_points_list(pcRos, skip_nans=False)

    print("pc len")
    print(len(pc))

    x=[]
    y=[]
    z=[]
    r=[]
    g=[]
    b=[]
    for point in pc:
        x.append(point[0])
        y.append(point[1])
        z.append(point[2])

        rgb = point[3]

        ba = bytearray(struct.pack("f", rgb))  


        count = 0
        for bytte in ba:
            if(count==0):
                r.append(255-bytte)
            if(count==1):
                g.append(255-bytte)
            if(count==2):
                b.append(255-bytte)
                
            count=count+1
            
    r=np.asarray(r)
    g=np.asarray(g)
    b=np.asarray(b)

    rgb = np.vstack([r,g,b])
    print("lenght of pc")
    print(len(x))
    xyz = np.vstack([x,y,z])
    

    pc = pointclouder.Points2Cloud(xyz.T,rgb.T)
    

    refe = open3d.create_mesh_coordinate_frame(1)

    H= np.eye(4)
    H[1,3]=0
    print(H)
    pc.transform(H)

    things =[refe,pc,pcMade]

    visu.draw_geometry(things)
     


if __name__ == '__main__':
    main(sys.argv)