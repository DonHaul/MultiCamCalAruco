#HERE WILL BE the v1, but organized in a good fashion
import rospy
import message_filters
from sensor_msgs.msg import Image

import cv2
import open3d
import numpy as np
import time



import sys

from libs import *
from libs import rosinterface as IRos
def main(argv):
    

    
    freq=50

    camNames =  IRos.getAllPluggedCameras()
    camName = camNames[0]
    print(camName)

    #fetch K of existing cameras on the files
    intrinsics = FileIO.getKDs(camNames)

    rospy.init_node('ora_ora_ora_ORAA', anonymous=True)

    arucoData = FileIO.getJsonFromFile("./static/ArucoWand.json")

    arucoData['idmap'] = aruco.markerIdMapper(arucoData['ids'])

    arucoCorners ={}#= FileIO.getFromPickle("pickles/corners_scorpion_07-06-2019_15:34:54.pickle")

    #load aruco model
    arucoModel = FileIO.getFromPickle("./static/arucoModelNEW.pickle")

    #shows aruco model
    #visu.ViewRefs(arucoModel['R'],arucoModel['T'],showRef=True,refSize=0.1)


    #initializes class
    pcer = PCGetter(camName,intrinsics,arucoModel,arucoData,arucoCorners)

    camSub=[]
    #getting subscirpters to use message fitlers on
    camSub.append(message_filters.Subscriber(camName+"/rgb/image_color", Image))
    #camSub.append(message_filters.Subscriber(camName+"/depth_registered/image_raw", Image))


    ts = message_filters.ApproximateTimeSynchronizer(camSub,1, 1.0/freq, allow_headerless=True)
    ts.registerCallback(pcer.callback)
    print("callbacks registered")




    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shut")


    print("FINISHED")


class PCGetter(object):

    def __init__(self,camName,intrinsics,arucoModel,arucoData,arucoCorners):
        
        print("initiated")




        self.camName = camName
        
        #intrinsic Params
        self.intrinsics = intrinsics

        #assigning the model
        self.arucoModel = arucoModel

        self.arucoCorners = arucoCorners

        #assigning data
        self.arucoData = arucoData

    def callback(self,*args):


        rgb = IRos.rosImg2RGB(args[0])
        #depth_reg = IRos.rosImg2Depth(args[1])

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
        
        
        print(det_corners)
        
        dettt = np.asarray(det_corners)
        print(dettt.shape)
        quit()

        dettt = dettt.reshape(-1,2) #piles up corners x,2
        print(dettt)

        #draw maerkers
        if ids is not None:
            #hello = cv2.aruco.drawDetectedMarkers(hello,det_corners,np.asarray(ids))


            for i in range(dettt.shape[0]):   
                
                hello = visu.paintImage(hello,[dettt[i,1],dettt[i,0]],offset=1,color=[0,0,255])
                


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


            projmtx = np.concatenate((Rr,tt),axis=1)

            print(projmtx.shape)
            print("WOW")

            
            valid3Dcorners = np.ones((len(validids)*4,4)) #x y z 1


            for i in range(0,len(validids)):

                mappedId = self.arucoData['idmap'][str(validids[i])]
                
                valid3Dcorners[i*4:i*4+4,0:3] = self.arucoCorners[mappedId*4:mappedId*4+4,:]

            print(valid3Dcorners)
            
            P = K.dot(projmtx)
            print(P)
            
            pts = P.dot(valid3Dcorners.T)

            print(pts.shape)
            print(pts)
            lambdas = pts[2,:]
            print("Vector:")
            print(lambdas)
            print("magic equation")
            print(lambdas[None,:])
            #None broadcasts element to all rows (this is called broadcasting https://www.w3resource.com/python-exercises/numpy/python-numpy-exercise-96.php)
            pts = pts / lambdas[None,:]
            print(pts)

            print("POINTS PAINTING")

            for i in range(pts.shape[-1]):   
                hello = visu.paintImage(hello,[pts[1,i],pts[0,i]],offset=1,color=[0,255,0])
                
        cv2.imwrite("corners.png",hello)
        cv2.imshow("Detected Markers",hello)
        cv2.waitKey(0)
        cv2.destroyAllWindows()





        
        pointsu = np.empty((3,0))
        

        corneee = np.squeeze(det_corners)


      

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

        

            


if __name__ == '__main__':
    main(sys.argv)