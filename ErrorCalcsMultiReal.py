from libs import *
import numpy as np
import random
import cv2
import datetime
import sys
import csv
import time

def InitializeStats(statstext, measurestext):

    errorData={}

    for meas in measurestext:

        errorData[meas]={}

        for stat in statstext:
            errorData[meas][stat]=[]

    return errorData

            


def GenSimpleStats(errorMeasure,measurename,errorData,statstext=None):
    
    if measurename not in errorData and statstext is not None:
        errorData[measurename]= {}
        print("CRREATED")
        print(measurename)


        for stat in statstext:
            errorData[measurename][stat]=[]

    errorData[measurename]['singular'] = errorMeasure
    errorData[measurename]['total'] = sum(errorMeasure)
    errorData[measurename]['avg'] = np.mean(errorMeasure)
    errorData[measurename]['median'] = np.median(errorMeasure)
    errorData[measurename]['std'] = np.std(errorMeasure)

    return errorData





def main(path,imgdirectory=None):
    #path = "./Logs/2019-09-01_18:07:51_coyote"
    view=False

    synth = False

    print(imgdirectory)

    info =  FileIO.getJsonFromFile(imgdirectory+"info.json")

    print(info)
    frames = info['count']
    
    intrinsics = FileIO.getIntrinsics(info['camnames'])
    #K = intrinsics[camname]['rgb']['K']
    #D = intrinsics[camname]['rgb']['D']
        


 

    mediapath = FileIO.CreateFolder("../Media/reprojections",putDate=True)

    data =  FileIO.getJsonFromFile( path + "/pipeline.json" )

    arucoData =  FileIO.getJsonFromFile(data['model']['arucodata'])

    arucoModel = FileIO.getFromPickle( path + "/poses.pickle" )
    
    arucoData['idmap'] = aruco.markerIdMapper(arucoData['ids'])    
 


    measurestext=['corner','center','reprojection','translation','angle','rodriguez']
    statstext=['singular','total','avg','median','std']

    #errorData = FileIO.getJsonFromFile(path + "/errors.pickle")

    #if errorData is None:
    errorData = InitializeStats(statstext,measurestext)


    posesObtained =  FileIO.getFromPickle(path+"/poses.pickle")

    
    #reprojection error
    allnoisypts2D=[]

  

    reprojectionerrors= np.array([])
    translationerrors = []
    rodriguezerrors = []
    ndetectedarucos = []
    angleerrors = []

    count = 0
    imcount =0


    #go throuth every image or generate frames
    for i in range(0,frames):

        for camname in camn

        img = np.zeros((480,640,3),dtype=np.uint8)

        #set up a scene
        
        #real data

        #read image
        img = cv2.imread(imgdirectory+camname + "_rgb_" + str(count) + ".png")
        


        #pretty much a copy of cangalhoPnPDetector

        
        #finds markers
        det_corners, ids, rejected = aruco.FindMarkers(img, K)
        
    

        #convert (n_detecions,4,2) into (2,n_detecions*4)
        #pts2D = np.squeeze(det_corners).reshape(len(ids)*4,2).T



        validids=[]
        validcordners= []

        #in case there is only 1 id, convert it into a list with 1 element
        if ids is not None:

            ids = ids.squeeze()

            if (helperfuncs.is_empty(ids.shape)):
                ids=[int(ids)]

        #fetch valid corners
        if  ids is not None and len(ids)>0:

            #filter ids and cornerds
            validids=[]
            validcordners= []

            #fetch valid ids and corners
            for k in range(0,len(ids)):
                if ids[k] in arucoData['ids']:
    
                    validids.append(ids[k])
                    validcordners.append(det_corners[k]) 
        
            #fetch valiid 2D points
            pts2D = np.squeeze(validcordners).reshape(len(validids)*4,2).T

            pts2DISPLAY2D = pts2D.astype(int)
            for j in range(pts2DISPLAY2D.shape[-1]):   
                img = visu.paintImage(img,[pts2DISPLAY2D[1,j],pts2DISPLAY2D[0,j]],offset=1,color=[0,255,0])

            #fetch valid 3D Corners
            #holds the corner positions in the virgin model
            detectedcornsobtainedmodel = np.zeros((3,len(validids)*4))

            for j in range(len(validids)):
                idd = arucoData['idmap'][str(validids[j])]
                #print(curcorners.shape)
                #print(validids[j]*4,validids[j]*4+4)
                #print(curcorners.T[:,validids[j]*4:validids[j]*4+4].shape)
                #this correspond the the obtained model corners
                detectedcornsobtainedmodel[:,j*4:j*4+4] = curcorners.T[:,idd*4:idd*4+4]


            Rfull,otvec = aruco.GetCangalhoFromMarkersPnP(validids,validcordners,K,D,arucoData,arucoModel)

            #convert to rotation vector
            orvec,_ = cv2.Rodrigues(Rfull)




        #reproject
        pts2Dobtained = cv2.projectPoints(detectedcornsobtainedmodel.T, orvec , otvec,K,np.zeros((5,1),dtype=float))[0].T
        pts2Dobtained = np.squeeze(pts2Dobtained)



        if not synth:

            #get pose
            retval, orvec, otvec2 = cv2.solvePnP(detectedcornsobtainedmodel.T,pts2Dobtained.T,K,np.array([0,0,0,0]), useExtrinsicGuess=False,flags = cv2.SOLVEPNP_ITERATIVE)

            orvec = cv2.Rodrigues(orvec)[0]


        pts2DISPLAY2D = pts2Dobtained.astype(int)
        for j in range(pts2DISPLAY2D.shape[-1]):   
            img = visu.paintImage(img,[pts2DISPLAY2D[1,j],pts2DISPLAY2D[0,j]],offset=1,color=[0,0,250])

        #cv2.imshow('image',img)
        #cv2.waitKey(1000)

        #cv2.imwrite("./Media/Imgs" + "/errorz"+str(count) +"_"+ datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")  + ".png",img)
        #cv2.imshow("Detected Markers",img)
        count = count + 1


        print("COUNT:",count)
        #reprojection error 
        curreprojectionerror = np.linalg.norm(pts2Dobtained - pts2D,axis=0)

        #for the synth, orvec is the estimated rotation, and Rfull is the ground truth rotation
        # for the real, orvec is saved model pnp estimated rotation and Rfull is the observer rotation from the image
        #the error rotation matrix

        if synth:
            wecome = cv2.Rodrigues(orvec)[0].T
        else:
            wecome = orvec.T

        
        errorRot = np.dot(wecome,Rfull)


        val = ((np.trace(errorRot) - 1) / 2)
        if val > 1:
            val=1

        #angle error
        curangleerrors = np.rad2deg(np.arccos(val))

        rodriz = cv2.Rodrigues(errorRot)[0]
        print("Rodriz",rodriz)

        #rodriguez error (norm of rotation vector)
        currodriguezerror = np.linalg.norm(rodriz)


        #translation error

        #tfull in camera coordninate is
        
    
        if synth:
            #[0],[0],[-camDist]] is the fixed cangalho position
            curtranslationerrors = np.linalg.norm(np.array([[0],[0],[-camDist]])-otvec)
        elif len(validids)>0:
            #otvec is the observed position form the image (uses the realtime model, what it sees), otvec2 uses the saved model 
            curtranslationerrors = np.linalg.norm(np.array(otvec2-otvec))

        if (not synth and  len(validids)>0) or synth:
            reprojectionerrors = np.concatenate([reprojectionerrors,curreprojectionerror],axis=0)

            errorData = GenSimpleStats(curreprojectionerror.tolist(),'reprojection_'+str(i) , errorData,statstext)


            rodriguezerrors.append(currodriguezerror)

            translationerrors.append(curtranslationerrors)

            angleerrors.append(curangleerrors)
            print("ANGLE ERR")
            print(curangleerrors)

            ndetectedarucos.append(len(validids))

        else:
            #set all to -1
            errorData = GenSimpleStats([-1],'reprojection_'+str(i) , errorData,statstext)


            rodriguezerrors.append(-1)

            translationerrors.append(-1)

            angleerrors.append(-1)

            ndetectedarucos.append(0)



        
    #visualize

    errorData = GenSimpleStats(reprojectionerrors.tolist(),'reprojection',errorData)

    errorData = GenSimpleStats(translationerrors,'translation',errorData)

    errorData = GenSimpleStats(angleerrors,'angle',errorData)

    errorData = GenSimpleStats(rodriguezerrors,'rodriguez',errorData)
    

    errorData = GenSimpleStats(ndetectedarucos,'ndetectedarucos',errorData,statstext)



    #get the 3D matrix
    #rvec,_ = cv2.Rodrigues(orvec)
    #print(errorData)

    return errorData





if __name__ == "__main__":
    path = sys.argv[1]
    
    imagepath = None

    if (len(sys.argv)>2):
        imagepath = sys.argv[2]

    errorData = main(path,imagepath)

    frames =  FileIO.getJsonFromFile(imagepath+"/info.json")['count']


    #write csv
    with open(path+'/errors.csv', 'wb') as csvfile:
        filewriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
          
          

        #write to file
        fullmeasures = []
        
        
        count = 0
        
        statstext=['total','avg','median','std']

        m = 'reprojection'
        for stat in statstext:
            fullmeasures.append(m+"_"+stat+" [px]")
            
        filewriter.writerow(['frame','# arucos'] + fullmeasures + ['translation','|rotationvec|','angle'])

        for i in range(frames):

            
            repMeasures=[]
            for stat in statstext:
                repMeasures.append(errorData['reprojection_'+str(i)][stat])


            
            filewriter.writerow([i,errorData['ndetectedarucos']['singular'][i]] + repMeasures + [errorData['translation']['singular'][i],errorData['rodriguez']['singular'][i],errorData['angle']['singular'][i] ])


        #filewriter.writerow([] + repMeasures + [errorData['translation']['singular'][i],errorData['rodriguez']['singular'][i],errorData['angle']['singular'][i] ])




    print("Saving Json...")
    FileIO.saveAsPickle('errors',errorData,path,False,False)