from libs import *
import numpy as np
import random
import cv2
import datetime

def InitializeStats(statstext, measurestext):

    errorData={}

    for meas in measurestext:

        errorData[meas]={}

        for stat in statstext:
            errorData[meas][stat]=[]

    return errorData

            


def GenSimpleStats(errorMeasure,measurename,errorData):
       
    errorData[measurename]['singular'].append(errorMeasure)
    errorData[measurename]['total'].append(sum(errorMeasure))
    errorData[measurename]['avg'].append(np.mean(errorMeasure))
    errorData[measurename]['median'].append(np.median(errorMeasure))
    errorData[measurename]['std'].append(np.std(errorMeasure))

    return errorData






path = "./Logs/2019-09-01_18:07:51_coyote"
view=False
synth=True
frames=100

mediapath = FileIO.CreateFolder("./Media/reprojections",putDate=True)


data =  FileIO.getJsonFromFile( path + "/pipeline.json" )
if synth:
    K = np.asarray(data['model']['K']).reshape((3,3))
    pixelnoisestd=data['model']['pixelnoisestd'] 

measurestext=['corner','center','reprojection','angle','rodriguez']
statstext=['singular','total','avg','median','std']

errorData = FileIO.getJsonFromFile(path + "/errors.pickle")

if errorData is None:
    errorData = InitializeStats(statstext,measurestext)


print(errorData)

#get aligned poses

arucoGroundTruth = FileIO.getFromPickle(data['model']['arucomodel'])
arucoObtained =  FileIO.getFromPickle(path+"/poses.pickle")



groundcorners = arucoGroundTruth['corners']
curcorners = arucoObtained['corners']
groundcenters = np.squeeze(np.array(arucoGroundTruth['t']))
curcenters = np.squeeze(np.array(arucoObtained['t']))


#this lines should yield the smae but dont, they yield almost the same thing
Rcorners , tcorners =  algos.procrustesMatlabWrapper(groundcorners,curcorners) 
Rcenters , tcenters =  algos.procrustesMatlabWrapper(groundcenters,curcenters)



#procrustes thing corners
curcorners = mmnip.Transform(curcorners.T,Rcorners,tcorners).T

#procrustes thing centers
curcenters = mmnip.Transform(curcenters.T,Rcenters,tcenters).T


if view==True:
    ballz=[]
    ballz = ballz + visu.SeePositions(groundcorners,view=False)
    ballz = ballz + visu.SeePositions(curcorners,color=[0.1,0.1,1],view=False)
    visu.draw_geometry(ballz)


#corner error
cornererror = np.linalg.norm(groundcorners - curcorners,axis=1)
errorData = GenSimpleStats(cornererror,'corner',errorData)

#center error
centererror = np.linalg.norm(groundcenters - curcenters,axis=1)
errorData = GenSimpleStats(centererror,'center',errorData)

#reprojection error
allnoisypts2D=[]

if synth:

    reprojectionerrors= np.array([])

    count = 0
    

    for i in range(0,frames):

        img = np.zeros((480,640,3),dtype=np.uint8)
        #generate a random position for the aruco
        Rfull = mmnip.genRandRotMatrix(360)
        Tfull =np.array([0,0,0])#(np.random.rand(3,1)*self.noiset)

        #calculates the corners on the transformed aruco
        cornersPos =  mmnip.Transform(groundcorners.T,Rfull,Tfull)

        Rcam=np.eye(3)
        tcam=np.array([[0],[0],[-1]])


        #visu.DrawScene(cornersPos,self.Rcam,self.tcam)

        #number of observations is 12
        n_obs=12

        detectedcorns = np.zeros((3,n_obs*4))
        detectedcornsobtainedmodel = np.zeros((3,n_obs*4))

        #picks a set of detected arucos for this camera
        rnds = random.sample(range(0, len(arucoGroundTruth['t'])), n_obs)

        print(rnds)

        #for every observed aruco fetches the detected corners
        for j in range(len(rnds)):

            #this correspond to the groundtruth corners
            detectedcorns[:,j*4:j*4+4] = cornersPos[:,rnds[j]*4:rnds[j]*4+4]            

            #this correspond the the obtained model corners
            detectedcornsobtainedmodel[:,j*4:j*4+4] = curcorners.T[:,rnds[j]*4:rnds[j]*4+4]

        #fetches them from the corners
        #print(cornersPos.shape)

        print("DETECTED CORNSs")
        print(detectedcorns.shape)

        #detectedcorns=cornersPos
        #REPLACE detectedcorns with cornersPos to see all corners
        #display 3D corners in cam coordinates
        cornersInCam=np.dot(Rcam.T,(detectedcorns-tcam))



        pts2D = cv2.projectPoints(cornersInCam.T, cv2.Rodrigues(np.eye(3))[0],np.zeros((3,1)),K,np.zeros((5,1),dtype=float))[0].T


        #ground truth
        pts2D = np.squeeze(pts2D)
        print(pts2D)

        pts2DISPLAY2D = pts2D.astype(int)
        for j in range(pts2DISPLAY2D.shape[-1]):   
            img = visu.paintImage(img,[pts2DISPLAY2D[1,j],pts2DISPLAY2D[0,j]],offset=1,color=[0,255,0])
        

        #addnoise
        pts2Dnoisy=pts2D+np.random.randn(pts2D.shape[0],pts2D.shape[1])*pixelnoisestd

        #get pose
        retval, orvec, otvec = cv2.solvePnP(detectedcornsobtainedmodel.T,pts2Dnoisy.T,K,np.array([0,0,0,0]), useExtrinsicGuess=False,flags = cv2.SOLVEPNP_ITERATIVE)

        #reproject
        pts2Dobtained = cv2.projectPoints(detectedcornsobtainedmodel.T, orvec , otvec,K,np.zeros((5,1),dtype=float))[0].T
        pts2Dobtained = np.squeeze(pts2Dobtained)


        pts2DISPLAY2D = pts2Dobtained.astype(int)
        for j in range(pts2DISPLAY2D.shape[-1]):   
            img = visu.paintImage(img,[pts2DISPLAY2D[1,j],pts2DISPLAY2D[0,j]],offset=1,color=[250,0,0])

        

        cv2.imwrite(mediapath + "/errorz" + datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S") + str(count) + ".png",img)
        cv2.imshow("Detected Markers",img)

        cv2.destroyAllWindows()

        count = count + 1


        curreprojectionerror = np.linalg.norm(pts2Dobtained - pts2D,axis=0)



        reprojectionerrors = np.concatenate([reprojectionerrors,curreprojectionerror],axis=0)
else:
    print("GET IMAGE / S")


    
#visualize
print(reprojectionerrors.shape)
errorData = GenSimpleStats(reprojectionerrors,'reprojection',errorData)

#print(errorData['reprojection']['avg'])
#get the 3D matrix
#rvec,_ = cv2.Rodrigues(orvec)
#print(errorData)


quit()


FileIO.saveAsPickle('errors.pickle',errorData,path,False,False)


