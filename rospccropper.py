
#runs in each camera
#HERE WILL BE the v1, but organized in a good fashion
import rospy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from sensor_msgs import point_cloud2

import cv2
import open3d
import numpy as np
import time

import struct


import tf

import sys

from libs import *

def main():
    
    #vol = o3d.visualization.read_selection_polygon_volume("../../TestData/Crop/cropped.json")

    rospy.init_node('echoes_act_III', anonymous=True)
    
    pchull = rospy.wait_for_message("/boxcast", PointCloud2)

    pc = point_cloud2.read_points_list(pchull, skip_nans=False)

  

    x=[]
    y=[]
    z=[]

    camname = "killerqueen"


    for point in pc:
        x.append(point[0])
        y.append(point[1])
        z.append(point[2])

    roi = np.vstack([x,y,z])


    tfer = tf.TransformListener()

    print(tfer.frameExists(camname))
    
    print(tfer.frameExists(pchull.header.frame_id))
    
    

    tfer.waitForTransform(camname,pchull.header.frame_id, rospy.Time(0),rospy.Duration(40.0))



    trans,rot = tfer.lookupTransform(camname,pchull.header.frame_id, rospy.Time(0))
    print(trans,rot)

    H = tf.transformations.quaternion_matrix(rot)

    #H = mmnip.Rt2Homo(H[0:3,0:3],trans)
    R = H[0:3,0:3]
    t = np.array(trans)

    print(type(t))
    #roi = mmnip.Transform(roi,R,t)
    roivec = open3d.Vector3dVector(roi.T)

    roipoly = open3d.visualization.SelectionPolygonVolume()

    roipoly.bounding_polygon = roivec
    roipoly.orthogonal_axis = "Y"
    roipoly.axis_max = 100.0
    roipoly.axis_min = -100.0
    
    print(np.asarray(roipoly.bounding_polygon))
    #receive transform

 

    pc2cropper = Pc2Crop(roipoly,tf,pchull.header.frame_id,camname)

    #pctopic="/camera/depth_registered/points"
    pctopic="/depth/points"

    rospy.Subscriber(camname+pctopic, PointCloud2,pc2cropper.PublishPC2callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut")



class Pc2Crop():

    def __init__(self,roipoly,tf,roireference,camname):
        self.roipoly = roipoly
        self.tf = tf
        self.roireference = roireference
        self.camname = camname

        self.pub = rospy.Publisher(camname+"/croppedpc",PointCloud2,queue_size=1)

    def PublishPC2callback(self,data):
        
        #data is the pc
        
        pc = point_cloud2.read_points_list(data, skip_nans=True)

        x=[]
        y=[]
        z=[]
        
        #r=[]
        #g=[]
        #b=[]

        for point in pc:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
            #print(point[0],point[1],point[2])

            '''
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
        #'''
        xyz = np.vstack([x,y,z])
        
        #this is the open pc pointcloud
        pco3d = pointclouder.Points2Cloud(xyz.T) #,rgb.T
        print(pco3d)
        pco3d.paint_uniform_color([1,0.706,0])

        
        open3d.visualization.draw_geometries_with_editing([pco3d])

        pcROI = self.roipoly.crop_point_cloud(pco3d)
        quit()
        print("pc")
        print(pcROI)

        points = []
        print(xyz)
        print(xyz.shape)
        
        for i in range(xyz.shape[1]):

            #rgb = struct.unpack('I', struct.pack('BBBB', colori[0], colori[1], colori[2], 255))[0]
            
            pt = xyz[:,i] #+ [rgb]
            points.append(pt)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                # PointField('rgb', 12, PointField.UINT32, 1),
                #PointField('rgba', 12, PointField.UINT32, 1),
                ]

        #print points

        header = data.header
        pc2 = point_cloud2.create_cloud(header, fields, points)
        
        #pctemp = data
       
        #while not rospy.is_shutdown():
        pc2.header.stamp = rospy.Time.now()
        self.pub.publish(pc2)
        print("Published")
        





if __name__ == '__main__':
    main()
    