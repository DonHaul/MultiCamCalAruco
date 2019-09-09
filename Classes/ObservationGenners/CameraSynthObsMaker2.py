from libs import *
import ObservationsMaker
import random
import numpy as np

import cv2

class CameraSynthObsMaker2(ObservationsMaker.ObservationsMaker):
    def __init__(self,data):

        self.noiset=data['noiset'] #in meters

        self.Rcangalho=data['synthmodel']['R']
        self.tcangalho=data['synthmodel']['t']

        self.modelcorners = data['synthmodel']['corners']


        visu.ViewRefs(data['synthmodel']['R'],data['synthmodel']['t'],refSize=0.1)
        visu.ViewRefs(data['modelscene']['R'],data['modelscene']['t'],refSize=0.1)

        self.Rcam=data['modelscene']['R']
        self.tcam=data['modelscene']['t']


        self.pixelnoisestd = data['pixelnoisestd']
        
        self.K = np.asarray(data['K']).reshape((3,3))

        print(self.K)

        self.n_obs = data['n_detectedarucos'] #arucos detected per camera
        self.noise = 0




        self.frames=data['samples']
        

        


    def GetObservations(self,streamData):

        
        observationsR=[]
        observationsT=[]



        #similar to output from ROS, gets observations from Marker in the camera coordinate
        #camsObs = synth.MultiCamSampleGeneratorFixed(self.Rcam,self.tcam,self.Rcangalho,self.tcangalho,nObs = self.n_obs,noise=self.noise, noiset=self.noiset)

        tmean=np.mean(self.tcam,axis=0)

        print("tmean")
        print(tmean)
        
        allObs = []#[ [] for z in range(len(self.Rcam)) ]

        for k in range(self.frames):

            #generate a random position for the aruco
            Rfull = mmnip.genRandRotMatrix(360)
            Tfull =(tmean+np.random.rand(3,1)*self.noiset)

            #calculates the corners on the transformed aruco
            cornersPos =  mmnip.Transform(self.modelcorners.T,Rfull,Tfull)

            
            

            #cycles every camera
            for i in range(len(self.Rcam)):

                detectedcorns = np.zeros((3,self.n_obs*4))

                #picks a set of detected arucos for this camera
                rnds = random.sample(range(0, len(self.Rcangalho)), self.n_obs)

                #for every observed aruco
                for j in range(len(rnds)):
                    detectedcorns[:,j*4:j*4+4] = cornersPos[:,rnds[j]*4:rnds[j]*4+4]            

                #fetches them from the corners
                #print(cornersPos.shape)


                #detectedcorns=cornersPos
                #REPLACE detectedcorns with cornersPos to see all corners
                cornersInCam=np.dot(self.Rcam[i].T,(detectedcorns-self.tcam[i]))

                #get the point in the image
                pts2D = self.K.dot(cornersInCam)
                lambdas = pts2D[2,:]
                pts2D = pts2D / lambdas[None,:]
                
                #quanityfy
                pts2D = pts2D.astype(int)
                
                #visualize
                img = np.zeros((480,640,3),dtype=np.uint8)

                for j in range(pts2D.shape[-1]):   
                    img = visu.paintImage(img,[pts2D[1,j],pts2D[0,j]],offset=1,color=[0,255,0])
                    
                cv2.imwrite("corners.png",img)
                cv2.imshow("Detected Markers",img)
                cv2.waitKey(300)
                cv2.destroyAllWindows()


                #reengineer points back in 3D, 
                #get po

                pts2D = pts2D[0:2,:].astype(np.float)



                pts2D=pts2D+np.random.randn(pts2D.shape[0],pts2D.shape[1])*self.pixelnoisestd
                #print(pts2D.shape)
                #print(detectedcorns.shape)
                #print(type(pts2D[0,0]))
                retval, orvec, otvec = cv2.solvePnP(detectedcorns.T,pts2D.T,self.K,np.array([0,0,0,0]), useExtrinsicGuess=False,flags = cv2.SOLVEPNP_ITERATIVE)

                #get the 3D matrix
                rvec,_ = cv2.Rodrigues(orvec)

                obs=[]
                
                #initializes observation
                o ={"obsId":0} #since it will always generate observation on id 0

                #generate R observations
                o['R']=rvec

                #generate t observations
                o['t']=otvec #WRONG - Not sure if this is the correct t

                #print(o['t'])
                obs.append(o)
                

                allObs.append(obs)


            print(len(allObs))

            #generate observations between cameras for a specific frame
            obsR, obsT = obsgen.GenerateCameraPairObs(allObs,self.Rcangalho,self.tcangalho)

            
            

            observationsR = observationsR + obsR
            observationsT = observationsT + obsT

            allObs = []

        return None,None,observationsR, observationsT
        
