#HERE WILL BE the v1, but organized in a good fashion
import rospy
import message_filters
from sensor_msgs.msg import Image

import cv2
import open3d
import numpy as np
import time

import commandline
import StateManager


import sys

from libs import *

def main(argv):
    

    
    freq=50

    camNames = IRos.getAllPluggedCameras()
    camName = camNames[0]
    print(camName)

    #fetch K of existing cameras on the files
    intrinsics = FileIO.getKDs(camNames)

    rospy.init_node('ora_ora_ora_ORAA', anonymous=True)

    arucoData = FileIO.getJsonFromFile("./static/ArucoWand.json")

    arucoData['idmap'] = aruco.markerIdMapper(arucoData['ids'])

    #load aruco model
    arucoModel = FileIO.getFromPickle("arucoModels/ArucoModel_0875_yak_25-05-2019_16:23:12.pickle")

    #shows aruco model
    visu.ViewRefs(arucoModel['R'],arucoModel['T'],showRef=True,refSize=0.1)


    #initializes class
    pcer = PCGetter(camName,intrinsics,arucoModel,arucoData)

    camSub=[]
    #getting subscirpters to use message fitlers on
    camSub.append(message_filters.Subscriber(camName+"/rgb/image_color", Image))
    camSub.append(message_filters.Subscriber(camName+"/depth_registered/image_raw", Image))


    ts = message_filters.ApproximateTimeSynchronizer(camSub,1, 1.0/freq, allow_headerless=True)
    ts.registerCallback(pcer.callback)
    print("callbacks registered")




    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut")


    print("FINISHED")


class PCGetter(object):

    def __init__(self,camName,intrinsics,arucoModel,arucoData):
        print("initiated")

        self.camName = camName
        
        #intrinsic Params
        self.intrinsics = intrinsics

        #assigning the model
        self.arucoModel = arucoModel

        #assigning data
        self.arucoData = arucoData

    def callback(self,*args):


        rgb = IRos.rosImg2RGB(args[0])
        depth_reg = IRos.rosImg2Depth(args[1])

        #copy image
        hello = rgb.astype(np.uint8).copy() 

        cv2.imshow("wow",hello)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        #get matrix intrinsics
        K = self.intrinsics['K'][self.camName]
        D = self.intrinsics['D'][self.camName]

        #finds markers
        det_corners, ids, rejected = aruco.FindMarkers(rgb, K,D)

        #draw maerkers
        if ids is not None:
            hello = cv2.aruco.drawDetectedMarkers(hello,det_corners,np.asarray(ids))

        cv2.imshow("Detected Markers",hello)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


        if ids is None:
            return

        ids = ids.squeeze()

        #makes a single id into a list with only it self
        if (helperfuncs.is_empty(ids.shape)):
            ids=[int(ids)]

        #place where all geometries will be stores
        sphs = []

        #3D WAY (Scaled Procrustes)
        if  ids is not None and len(ids)>0:

            #filter ids and cornerds
            validids=[]
            validcordners= []

            #fetches only ids that are on the cangalho
            for i in range(0,len(ids)):
                if ids[i] in self.arucoData['ids']:
                    #print("Valid marker id: "+str(ids[i]))
                    validids.append(ids[i])
                    validcordners.append(det_corners[i]) 


            Rr,tt = aruco.GetCangalhoFromMarkersProcrustes(validids,validcordners,K,self.arucoData,self.arucoModel,depth_reg)


            
            if(Rr is not None):
                H = mmnip.Rt2Homo(Rr.T,tt)

                refe = open3d.create_mesh_coordinate_frame(0.16, origin = [0, 0, 0])
                refe.transform(H)
                               
                sphs.append(refe)
                sphere2 = open3d.create_mesh_sphere(0.02)
                sphere2.transform(H)
                sphere2.paint_uniform_color([1,0,1])

                sphs.append(sphere2)





        #PnP way
        if  ids is not None and len(ids)>0:

            #only fetches corners and ids, for the markers ids that exist in cangalho (2-13)
            validids=[]
            validcordners=[]
            for i in range(0,len(ids)):
                if ids[i] in self.arucoData['ids']:
  
                    validids.append(ids[i])
                    validcordners.append(det_corners[i]) 
       
            #calculates pose
            Rr,tt = aruco.GetCangalhoFromMarkersPnP(validids,validcordners,K,D,self.arucoData,self.arucoModel,None)#(Rr.T,tt)

            #converts in homogeneous
            H = mmnip.Rt2Homo(Rr,tt.T)
          
            #prints results, in green
            sphere1 = open3d.create_mesh_sphere(0.02)
            sphere1.transform(H)
            sphere1.paint_uniform_color([0,1,0])
            sphs.append(sphere1)
            refe = open3d.create_mesh_coordinate_frame(0.16, origin = [0, 0, 0])
            refe.transform(H)   #Transform it according tom p
            sphs.append(refe)

            
            


        

        
        pointsu = np.empty((3,0))
        
        #print(hello.shape)
        #print("detected cornerds")
        #print(det_corners)
        
        corneee = np.squeeze(det_corners)
        #print(corneee)

        #corn2paint = corneee[2,:]
        #offset=5
        #for ii in  range(int(corn2paint[0])-offset,int(corn2paint[0])+offset):
        #    for jj in range(int(corn2paint[1])-offset,int(corn2paint[1])+offset):
        #        hello[jj,ii,:]= [255,0,255]


        #cv2.imshow("wow",hello)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

   
        #map1,map2 = cv2.initUndistortRectifyMap(K,D,np.eye(3),K,(640,480),cv2.CV_32FC1)
        #print(wowl)
        #img2 = cv2.remap(hello, map1, map2,cv2.INTER_NEAREST)
        #cv2.imshow("woweeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",img2)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        

        #find detected corners in 3D and paint them
        for cor in det_corners:
                        
            for i in range(0,4):

                 
                point = mmnip.singlePixe2xyz(depth_reg,cor[0,i,:],K)

                point = np.expand_dims(point,axis=1)
                

                H = np.eye(4)
                H[0:3,3]=point.T

                #paints corners in 3D space
                sphere = open3d.create_mesh_sphere(0.006)
                sphere.transform(H)
                sphere.paint_uniform_color([1,0,1])
                sphs.append(sphere)
                pointsu=np.hstack((pointsu,point))



        #Marker-by-Marker WAY
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(det_corners,self.arucoData['size'],K,D)
    
        tvecs=np.squeeze(tvecs)

        #turn 0D into 1D
        if len(tvecs.shape)==1:
            tvecs = np.expand_dims(tvecs,axis=0)
        

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

        

        #converts image 2 depth
        points = mmnip.depthimg2xyz2(depth_reg,K)
        points = points.reshape((480*640, 3))

        #print(colors.shape)
        rgb1 = rgb.reshape((480*640, 3))#colors
        
        pc = pointclouder.Points2Cloud(points,rgb1)

        pc2 = pointclouder.Points2Cloud(pointsu.T)

        pc2.paint_uniform_color([1,0,1])
        

        open3d.draw_geometries([pc]+sphs)


            


if __name__ == '__main__':
    main(sys.argv)