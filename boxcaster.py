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

    '''
    vertexhull=[
        [-1,-1,0.3],
        [-1,1,0.3],
        [1,-1,0.3],
        [1,1,0.3],
        [-1,-1,2],
        [-1,1,2],
        [1,-1,2],
        [1,1,2]
    ]
    '''

    vertexhull=[
        [-100,0,-100],
        [-100,0,100],
        [100,0,-100],
        [100,0,100],
    ]

    

    rospy.init_node('seven_page_muda', anonymous=True)


    pub = rospy.Publisher("boxcast", PointCloud2, queue_size=2)

    points = []

    for i in range(len(vertexhull)):

        #print hex(rgb)
        pt = [vertexhull[i][0], vertexhull[i][1], vertexhull[i][2]]
        points.append(pt)

    print("Vertices loader")

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            # PointField('rgb', 12, PointField.UINT32, 1),
            #PointField('rgba', 12, PointField.UINT32, 1),
            ]

    #print points

    header = Header()
    header.frame_id = "/killerqueen"
    pc2 = point_cloud2.create_cloud(header, fields, points)
    

    while not rospy.is_shutdown():
        pc2.header.stamp = rospy.Time.now()
        pub.publish(pc2)
        rospy.sleep(1.0)


























if __name__ == '__main__':
    main()