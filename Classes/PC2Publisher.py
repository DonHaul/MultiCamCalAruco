#!/usr/bin/env python

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import open3d

from libs import *

import time
import struct
from scipy import spatial
#mat_contents = sio.loadmat('octave_a.mat')
import time




class PC2Publisher:
    def __init__(self,intrinsics,camNames):
        
        self.camNames=camNames
        self.intrinsics=intrinsics
        self.pub = rospy.Publisher("point_cloud2_222", PointCloud2, queue_size=1)

        

    def PublishPC2(self,data):
        deoth = mmnip.depthimg2xyz2(data['depth'][0],self.intrinsics[ self.camNames[0]]['rgb']['K'],(480,640))

        pointse = deoth.reshape((480*640, 3))
         #print(points.shape)
        
        rgbse = data['rgb'][0].reshape((480*640,3))


        points = []
        for colori,depthi in zip(rgbse,pointse):

            rgb = struct.unpack('I', struct.pack('BBBB', colori[0], colori[1], colori[2], 255))[0]
            #print hex(rgb)
            pt = [depthi[0], depthi[1], depthi[2], rgb]
            points.append(pt)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                # PointField('rgb', 12, PointField.UINT32, 1),
                PointField('rgba', 12, PointField.UINT32, 1),
                ]

        #print points

        header = Header()
        header.frame_id = "world"
        pc2 = point_cloud2.create_cloud(header, fields, points)

        print("boom")
        #while not rospy.is_shutdown():
        pc2.header.stamp = rospy.Time.now()
        self.pub.publish(pc2)
            #rospy.sleep(1.0)

        