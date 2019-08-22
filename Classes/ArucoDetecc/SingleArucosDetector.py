from libs import *
import numpy as np
import cv2

class SingleArucosDetector:
    def __init__(self,data):


        self.arucoData=data['arucodata']
        self.arucoData['idmap'] = aruco.markerIdMapper(self.arucoData['ids'])



    def ArucoDetector(self,streamsingle,K,D):
        '''
        this function creates observations between this camera and every aruco marker it sees

        if the camera sees markers 1 2 and 3

        it will generate Rcam_1 Rcam_2 and Rcam_3

        THIS FUNCTION WILL GENERATE SAMPLES FOR A SINGLE CAMERA
        
        Args:
            K - intrinsic camera matrix
            D - distortion parameters
            det_corners - all detected corners
            hello - image that has the aruco detections added to it on top
            ids - all detected ids

        Returns:
            observations (dict array):All marker observations made by this camera
                obsId: observed aruco marker
                t: translation from obsId to camera (marker position in world coordinates)
                R: rotation from camera to obsId
        '''
        
        #fetches detected markers
        det_corners, ids, rejected = aruco.FindMarkers(streamsingle['rgb'], K)

        #changes image
        img = streamsingle['rgb'].astype(np.uint8).copy() 
        img = cv2.aruco.drawDetectedMarkers(img,det_corners,ids)
        
        #list of all observations generated
        observations =[]

        #if at least one marker is detected
        if  ids is not None and len(ids)>0:

            #finds rotations and vectors and draws referentials on image
            rots,tvecs,img = aruco.FindPoses(K,D,det_corners,img,len(ids),self.arucoData['size'])

            #squeeze
            ids = ids.squeeze()

            #special 1 element case
            ids = ids.tolist()
            if(type(ids)==int):
                ids=[ids]

            #generates samples
            for i in range(0,len(ids)):                
                    
                #only valid markers
                if ids[i] not in self.arucoData['ids']:
                    print("Invalid marker id: "+str(ids[i]))
                    continue 

                #initializes observation
                o ={"obsId":self.arucoData['idmap'][str(ids[i])]}

                #generate R observations
                o['R']=rots[i]


                #generate t observations
                o['t']=tvecs[i].T #WRONG - Not sure if this is the correct t
                #print(o['t'])
                observations.append(o)

        #print("observations are")
        #print(observations)
        #print("finish")
    
                
        return observations ,img
