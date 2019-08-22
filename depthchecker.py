#HERE WILL BE the v1, but organized in a good fashion
import rospy
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import struct
import ctypes

import cv2
import open3d
import numpy as np
import time
from sensor_msgs.msg import CameraInfo


import sys

from libs import *

def main(argv):
    

    
    freq=1

    camNames = IRos.getAllPluggedCameras()
    camName = "speedwagon"

    
    #fetch K of existing cameras on the files
    intrinsics = FileIO.getIntrinsics(['speedwagon'])

    rospy.init_node('ora_ora_ora_ORAA', anonymous=True)


    arucoData = FileIO.getJsonFromFile("./static/ArucoWand.json")

    arucoData['idmap'] = aruco.markerIdMapper(arucoData['ids'])

    #load aruco model
    arucoModel = FileIO.getFromPickle("static/ArucoModel_0875_yak_25-05-2019_16:23:12.pickle")

    #initializes class
    pcer = PCGetter(camName,intrinsics,arucoData,arucoModel)
    print(camName)
    camSub=[]
    #getting subscirpters to use message fitlers on
    print(camName+"/rgb/image_rect_color")
    camSub.append(message_filters.Subscriber(camName+"/rgb/image_rect_color", Image))
    camSub.append(message_filters.Subscriber(camName+"/depth_registered/sw_registered/image_rect_raw", Image))
    camSub.append(message_filters.Subscriber(camName+"/depth_registered/points", PointCloud2))
    camSub.append(message_filters.Subscriber("speedwagon/depth_registered/sw_registered/image_rect_raw", Image))
    camSub.append(message_filters.Subscriber("speedwagon/rgb/image_rect_color", Image))
    #camSub.append(message_filters.Subscriber(camName+"/depth/points", PointCloud2))


    ts = message_filters.ApproximateTimeSynchronizer(camSub,1, 1.0/freq, allow_headerless=True)
    ts.registerCallback(pcer.callback)
    print("callbacks registered")




    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut")


    print("FINISHED")


class PCGetter(object):

    def __init__(self,camName,intrinsics,arucoData,arucoModel):
        print("initiated")

        self.camName = camName
        
        #intrinsic Params
        self.intrinsics = intrinsics


        #assigning the model
        self.arucoModel = arucoModel

        #assigning data
        self.arucoData = arucoData

        print(intrinsics['speedwagon'])


    def callback(self,*args):

        print("HHEYGWY")
       

        rgb = IRos.rosImg2RGB(args[0])
        depth_reg = IRos.rosImg2Depth(args[1])

        print(rgb.shape)
        print(depth_reg.shape)


        #copy image
        hello = rgb.astype(np.uint8).copy() 

        cv2.imshow("wow",hello)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


        #finds markers
        det_corners, ids, rejected = aruco.FindMarkers(rgb,self.intrinsics['speedwagon']['rgb']['K'])

        #draw maerkers
        if ids is not None:
            hello = cv2.aruco.drawDetectedMarkers(hello,det_corners,np.asarray(ids))

        cv2.imshow("Detected Markers",hello)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


        #Marker-by-Marker WAY
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(det_corners,self.arucoData['size'],self.intrinsics['speedwagon']['rgb']['K'],np.asarray([0,0,0,0]))

        
        


        #turn 0D into 1D
        if len(tvecs.shape)==1:
            tvecs = np.expand_dims(tvecs,axis=0)
        sphs = []

        for i in range(0,tvecs.shape[0]):

            sphere = open3d.create_mesh_sphere(0.016)

            #converts in 3x3 rotation matrix
            Rr,_ = cv2.Rodrigues(rvecs[i])

            H = mmnip.Rt2Homo(Rr,tvecs[i,:])

            #prints marker position estimates
            refe = open3d.create_mesh_coordinate_frame(0.1, origin = [0, 0, 0])
            refe.transform(H)
            sphere.transform(H)
            sphere.paint_uniform_color([0,0,1])
            sphs.append(sphere)
            sphs.append(refe)

        print("DEPTH REG SHAPE IS")
        print(depth_reg.shape)
        points = mmnip.depthimg2xyz2(depth_reg,self.intrinsics['speedwagon']['rgb']['K'],(480,640))
        print(points.shape)
        points = points.reshape((480*640, 3))
        print(points.shape)
        
        pc1 = pointclouder.Points2Cloud(points,rgb.reshape((480*640,3)),clean=True)

        '''
        open3d.draw_geometries([pc1])

        #pc1 = pointclouder.Points2Cloud(points,rgb=rgb.resha)

        rgbd = mmnip.xyz2rgbd(points, rgb, self.intrinsics['speedwagon']['depth']['R'] , self.intrinsics['speedwagon']['depth']['P'][:,3] , self.intrinsics['speedwagon']['rgb']['K'])

        cv2.imshow("wow",rgbd)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        print("HOLA")
        print(rgbd.shape)
        rgbd=cv2.resize(rgbd,(480,360))
        print(rgbd.shape)
        cv2.imshow("wow",rgbd)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        rgb1=rgbd.reshape((360*480, 3))

        pc1 = pointclouder.Points2Cloud(points,rgb1)

        open3d.draw_geometries([pc1])

        
        #get matrix intrinsics
        #K = self.intrinsics[]['K'][self.camName]
        #D = self.intrinsics['D'][self.camName]





        pc = point_cloud2.read_points_list( args[2], skip_nans=True)

        print("pc len")
        print(len(pc))

        x=[]
        y=[]
        z=[]
        r=[]
        g=[]
        b=[]
        for point in pc:

            #print(point)

            x.append(point[0])
            y.append(point[1])
            z.append(point[2])

            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,point[3])
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r.append( (pack & 0x00FF0000)>> 16)
            g.append((pack & 0x0000FF00)>> 8)
            b.append((pack & 0x000000FF))

            print(len(r))

        xyz = np.vstack([x,y,z])
        rgbb = np.vstack([b,g,r])
        
            
        print("jeff")
        pc = pointclouder.Points2Cloud(xyz.T,rgbb.T)
        '''



        
        visu.draw_geometry([pc1]+sphs)

        

            


if __name__ == '__main__':
    main(sys.argv)