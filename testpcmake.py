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


def main():



    

    cameraName = "emperorcrimson"

    intrinsics = FileIO.getIntrinsics([cameraName])

    K = intrinsics[cameraName]['rgb']['K']

    #bridge to convert ROS image into numpy array
    br = CvBridge()
    

    rospy.init_node('my_name_is_jeff', anonymous=True)

    topicRGB = "/rgb/image_rect_color"
    topicDepth ="/depth_registered/sw_registered/image_rect_raw"


    rgbros = rospy.wait_for_message(cameraName + topicRGB, Image)
    depthros = rospy.wait_for_message(cameraName + topicDepth, Image)

    #converts ros img to numpy array
    rgb = br.imgmsg_to_cv2(rgbros, desired_encoding="bgr8")
    depth= br.imgmsg_to_cv2(depthros, desired_encoding="passthrough")
    
    cv2.imshow("rgb",rgb)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    cv2.imshow("depth",depth)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    start = time.time()


    pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

    pointse = mmnip.depthimg2xyz2(depth,K,(480,640))

    pointse = pointse.reshape((480*640, 3))
    #print(points.shape)
        
    rgbse = rgb.reshape((480*640,3))


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
    end = time.time()
    print(end - start)
    print("boom")
    while not rospy.is_shutdown():
        pc2.header.stamp = rospy.Time.now()
        pub.publish(pc2)
        rospy.sleep(1.0)


























if __name__ == '__main__':
    main()