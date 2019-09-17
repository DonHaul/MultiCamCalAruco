#!/usr/bin/env python

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import rospy
import numpy as np
import cv2

import open3d

from libs import *

import time
import struct
from scipy import spatial
#mat_contents = sio.loadmat('octave_a.mat')
import time


def main():

  
    halfedge=2.0
    dist = 3.0

    #vertices for the boundaries, it can be any shape, a cube is used for simplicity
    vertexhull=np.asarray([
        [-halfedge,-halfedge,0.3],
        [-halfedge,halfedge,0.3],
        [halfedge,-halfedge,0.3],
        [halfedge,halfedge,0.3],
        [-halfedge,-halfedge,dist],
        [-halfedge,halfedge,dist],
        [halfedge,-halfedge,dist],
        [halfedge,halfedge,dist]
    ])


    print(vertexhull)
 

    rospy.init_node('seven_page_muda', anonymous=True)

    #publisher box cast
    pub = rospy.Publisher("boxcast", PointCloud2, queue_size=2)

   

    print("Vertices loader")

    #configure header
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            ]

    

    #create point
    header = Header()
    header.frame_id = "/killerqueen"
    pc2 = point_cloud2.create_cloud(header, fields, vertexhull)
    

    while not rospy.is_shutdown():
        pc2.header.stamp = rospy.Time.now()
        pub.publish(pc2)
        rospy.sleep(1.0)


























if __name__ == '__main__':
    main()