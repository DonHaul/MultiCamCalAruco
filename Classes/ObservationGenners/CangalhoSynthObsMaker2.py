from libs import *
import ObservationsMaker
import random
import numpy as np

import cv2

class CangalhoSynthObsMaker2(ObservationsMaker.ObservationsMaker):
    def __init__(self,data):

        
        self.arucoData= data['arucodata']

        self.noiset=data['noiset'] #in meters

        self.Rcangalho=data['synthmodel']['R']
        self.tcangalho=data['synthmodel']['t']

        self.modelcorners = data['synthmodel']['corners']


        visu.ViewRefs(data['synthmodel']['R'],data['synthmodel']['t'],refSize=0.05)

        self.Rcam=np.eye(3)
        self.tcam=np.array([[0],[0],[-1]])


        self.pixelnoisestd = data['pixelnoisestd']
        
        self.K = np.asarray(data['K']).reshape((3,3))

        print(self.K)

        self.n_obs = data['n_detectedarucos'] #arucos detected per camera
        self.noise = 0




        self.frames=data['samples']
        

        


    def GetObservations(self,streamData):

        
        observationsR=[]
        observationsT=[]

        counter = 0

        out1 = cv2.VideoWriter('outputcorners.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 3, (640,480))

        outaxis = cv2.VideoWriter('outputaxis.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 3, (640,480))



        #similar to output from ROS, gets observations from Marker in the camera coordinate
        #camsObs = synth.MultiCamSampleGeneratorFixed(self.Rcam,self.tcam,self.Rcangalho,self.tcangalho,nObs = self.n_obs,noise=self.noise, noiset=self.noiset)

 
        
        allObs = []#[ [] for z in range(len(self.Rcam)) ]

        for k in range(self.frames):

            counter=counter+1

            #generate a random position for the aruco
            Rfull = mmnip.genRandRotMatrix(360)
            Tfull =np.array([0,0,0])#(np.random.rand(3,1)*self.noiset)

            #calculates the corners on the transformed aruco
            cornersPos =  mmnip.Transform(self.modelcorners.T,Rfull,Tfull)

            
            #visu.DrawScene(cornersPos,self.Rcam,self.tcam)

         

            detectedcorns = np.zeros((3,self.n_obs*4))

            #picks a set of detected arucos for this camera
            rnds = random.sample(range(0, len(self.Rcangalho)), self.n_obs)

            print(rnds)

            #for every observed aruco fetches the detected corners
            for j in range(len(rnds)):
                detectedcorns[:,j*4:j*4+4] = cornersPos[:,rnds[j]*4:rnds[j]*4+4]            

            #fetches them from the corners
            #print(cornersPos.shape)

            print("DETECTED CORNSs")
            print(detectedcorns.shape)

            #detectedcorns=cornersPos
            #REPLACE detectedcorns with cornersPos to see all corners
            #display 3D corners in cam coordinates
            cornersInCam=np.dot(self.Rcam.T,(detectedcorns-self.tcam))

            #get the point in the image same as cv2.projectPoints
            pts2D = self.K.dot(cornersInCam)
            lambdas = pts2D[2,:]

            pts2D = pts2D / lambdas[None,:]
            
            #reprojection error is calculated here
            
            #only quantify in order to display image
            pts2DISPLAY2D = pts2D.astype(int)

            #DONT QUANTIFY! - assume subpixel detection of positions in image
            
            #GROUND TRUTH IS HERE

            #visualize
            img = np.zeros((480,640,3),dtype=np.uint8)

            for j in range(pts2DISPLAY2D.shape[-1]):   
                img = visu.paintImage(img,[pts2DISPLAY2D[1,j],pts2DISPLAY2D[0,j]],offset=1,color=[0,255,0])
                
            cv2.imwrite("./Media/corners"+str(counter)+ ".png",img)
            cv2.imshow("Detected Markers",img)
            cv2.waitKey(300)
            cv2.destroyAllWindows()


            print("LOALOAL")
            print(img.shape)
            out1.write(img)
            
            pts2D = pts2D[0:2,:].astype(np.float)


            #readd noise later
            pts2D=pts2D+np.random.randn(pts2D.shape[0],pts2D.shape[1])*self.pixelnoisestd
            
            
            #print(pts2D.shape)
            #print(detectedcorns.shape)
            #print(type(pts2D[0,0]))

            print(pts2D.shape)

            #initialize conrers
            cornsformatted = np.zeros((len(rnds),1,4,2))

            for j in range(len(rnds)):
                aux=pts2D[:,j*4:j*4+4].T
                cornsformatted[j,0,:,:]=aux


           

            print(pts2D.shape)

            rots,tvecs,img = aruco.FindPoses(self.K,np.array([0.0,0.0,0.0,0.0]),cornsformatted,img,len(rnds),self.arucoData['size'])

            cv2.imwrite("./Media/cornersaxis"+str(counter)+".png",img)
            cv2.imshow("Detected Markers",img)
            cv2.waitKey(30)
            cv2.destroyAllWindows()

            outaxis.write(img)
            

            #generate observations
            observsR, observsT = obsgen.ArucoRealObsGenner(rnds,rots,tvecs,captureT=True,captureR=True)

            print(observsR)
            print(observsT)
            #retval, orvec, otvec = cv2.solvePnP(detectedcorns.T,pts2D.T,self.K,np.array([0,0,0,0]), useExtrinsicGuess=False,flags = cv2.SOLVEPNP_ITERATIVE)


            observationsR = observationsR + observsR
            observationsT = observationsT + observsT

        
        #print(observationsR)

        #print("HOLAA")

        out1.release()
        outaxis.release()

        return None,None,observationsR, observationsT
        
