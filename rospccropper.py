
#runs in each camera
#HERE WILL BE the v1, but organized in a good fashion

import rospy

#import ros  pointcloud stuff and headers 
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

#import delauney to check hull
from scipy.spatial import Delaunay

#used to read and write ros pc
from sensor_msgs import point_cloud2


import cv2
import open3d
import numpy as np

import time

import struct

import tf

import sys

from libs import *

def main(argv):
    

    #start node
    rospy.init_node('echoes_act_III', anonymous=True)

    #read pointcloud boundaries
    pchull = rospy.wait_for_message("/boxcast", PointCloud2)
    pc = point_cloud2.read_points_list(pchull, skip_nans=False)

  

    x=[]
    y=[]
    z=[]

    camname = argv[0]


    for point in pc:
        x.append(point[0])
        y.append(point[1])
        z.append(point[2])

    roi = np.vstack([x,y,z])


    tfer = tf.TransformListener()

    print(tfer.frameExists(camname))
    
    print(tfer.frameExists(pchull.header.frame_id))
    
    

    tfer.waitForTransform(camname,pchull.header.frame_id, rospy.Time(0),rospy.Duration(40.0))


    #gets transformation
    trans,rot = tfer.lookupTransform(camname,pchull.header.frame_id, rospy.Time(0))
    print(trans,rot)
    H = tf.transformations.quaternion_matrix(rot)
    #H = mmnip.Rt2Homo(H[0:3,0:3],trans)
    R = H[0:3,0:3]
    t = np.array(trans)

    #transform
    roi = mmnip.Transform(roi,R,t)

    #initialize callback receiver
    pc2cropper = Pc2Crop(roi.T,camname)

    #pctopic="/camera/depth_registered/points"
    pctopic="/depth/points"


    rospy.Subscriber(camname+pctopic, PointCloud2,pc2cropper.PublishPC2callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut")



class Pc2Crop():

    def __init__(self,roipoly,camname):
        self.roipoly = roipoly

        self.isRGB=False
        self.camname = camname


        #set up publisher
        self.pub = rospy.Publisher(camname+"/croppedpc",PointCloud2,queue_size=1)

    def PublishPC2callback(self,data):
        
        #data is the pc
        
        pc = point_cloud2.read_points_list(data, skip_nans=True)

        #fetch xyz        
        xyz = np.array(pc)[:,0:3]
       
        
        #check if it is inside hull
        isInHull = in_hull(xyz,self.roipoly)

        #pick indexes
        xyz = xyz[np.where(isInHull == True)]

        #setup pointfield
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                ]


        #print points
        header = data.header
        pc2 = point_cloud2.create_cloud(header, fields, xyz)
        
        #pctemp = data
       
        #while not rospy.is_shutdown():
        pc2.header.stamp = rospy.Time.now()
        self.pub.publish(pc2)
        print("Published")
        

def in_hull(p, hull):
    """
    Test if points in `p` are in `hull`

    `p` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    """

    #https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p)>=0



if __name__ == '__main__':

    main(sys.argv[1:])
    