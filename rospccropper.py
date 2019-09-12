
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




import sys

from libs import *

def main(argv):
    
    #vol = o3d.visualization.read_selection_polygon_volume("../../TestData/Crop/cropped.json")

    rospy.init_node('echoes act III', anonymous=True)
    
    pchull = rospy.wait_for_message("/boxcast", PointCloud2)

    pc = point_cloud2.read_points_list(pchull, skip_nans=False)

    x=[]
    y=[]
    z=[]

    


    for point in pc:
        x.append(point[0])
        y.append(point[1])
        z.append(point[2])

    roi = np.vstack([x,y,z])

    pc2cropper = Pc2Crop(roi,"hello",pchull.header.frame_id)

    #receive transform

    camname = "diavolo"

    pctopic="/camera/depth_registered/points"
    #pctopic="/camera/depth/points"

    rospy.Subscriber(name+pctopic, PointCloud2,pc2cropper.PublishPC2callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut")



class Pc2Crop():

    def __init__(self,roi,tf,roireference):
        self.roi = roi
        self.tf = tf
        self.roireference = roireference

    def PublishPC2callback(self,data):
        
        #data is the pc
        pc = point_cloud2.read_points_list(data, skip_nans=False)

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
        xyz = np.vstack([x,y,z])
        
        #this is the open pc pointcloud
        pc = pointclouder.Points2Cloud(xyz.T,rgb.T)
        
        pcROI = self.roi.crop_point_cloud(pc )

        points = []
        for i in range(len (max(xyz.shape))):

            rgb = struct.unpack('I', struct.pack('BBBB', colori[0], colori[1], colori[2], 255))[0]
            
            pt = xyz[i,:] + [rgb]
            points.append(pt)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                # PointField('rgb', 12, PointField.UINT32, 1),
                PointField('rgba', 12, PointField.UINT32, 1),
                ]

        #print points

        header = Header()
        header.frame_id = "killerqueen"
        pc2 = point_cloud2.create_cloud(header, fields, points)

        print("boom")
        #while not rospy.is_shutdown():
        pc2.header.stamp = rospy.Time.now()
        self.pub.publish(pc2)
            #rospy.sleep(1.0)





if __name__ == '__main__':
    main()
    