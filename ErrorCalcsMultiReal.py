from libs import *
import numpy as np
import random
import cv2
import datetime
import sys
import csv
import time
import open3d

def InitializeStats(statstext, measurestext):

    errorData={}

    for meas in measurestext:

        errorData[meas]={}

        for stat in statstext:
            errorData[meas][stat]=[]

    return errorData

def MultTranslError(trans):
    brah = 0
    
    cumnorm = 0

    for m in range(len(trans)):
        for n in range(m+1,len(trans)):
            brah = brah+1

            cumnorm = cumnorm + np.linalg.norm(trans[n]-trans[m])

    return cumnorm/brah

def MultRotError(rot):
    brah = 0
    
    curanglz = 0

    riguezerror=0



    for m in range(len(rot)):
        for n in range(m+1,len(rot)):
            brah = brah+1

            errorRot = np.dot(rot[m],rot[n].T) 

            val = ((np.trace(errorRot) - 1) / 2)
            
            
            if val > 1:
                val=1

            #angle error
            curanglz = curanglz + np.rad2deg(np.arccos(val))

            rodrize = cv2.Rodrigues(errorRot)[0]
            

            #rodriguez error (norm of rotation vector)
            riguezerror = riguezerror + np.linalg.norm(rodrize)

    return riguezerror/brah , curanglz/brah


           
        


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





def main(path,imgdirectory=None,saveImgs=True):
    #path = "./Logs/2019-09-01_18:07:51_coyote"
    view=False

    synth = False

    print(imgdirectory)

    info =  FileIO.getJsonFromFile(imgdirectory+"info.json")

    print(info)
    frames = info['count']
    camnames = info['camnames']
    
    print(camnames)

    intrinsics = FileIO.getIntrinsics(camnames)
    #K = intrinsics[camname]['rgb']['K']
    #D = intrinsics[camname]['rgb']['D']
        


   

    mediapath = FileIO.CreateFolder("../Media2/reprojections",putDate=True)

    data =  FileIO.getJsonFromFile( path + "/pipeline.json" )

    print(data)

    arucoData =  FileIO.getJsonFromFile(data['model']['arucodata'])

    sceneModel = FileIO.getFromPickle( path + "/poses.pickle" )
    

    arucoModel = FileIO.getFromPickle( data['model']['arucomodel'])

    curcorners = arucoModel['corners']
    
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

    nactivecams = []

    count = 0
    imcount =0

    colors=[]
    colors.append([1, 1, 0])
    colors.append([1,0,1])
    colors.append([0,1,1])

    cangalhotranslerr = []
    cangalhorotriguezerr = []
    cangalhorotangleerr = []
    fullcrossreprojectionerror =[]
    crossreprojectionerr={}

    for cam in camnames:

        crossreprojectionerr[cam]=[]

    #go throuth every image or generate frames
    for i in range(0,frames):

        img={}
        reprojected = {}
        R ={}
        t={}
        detectcorns = {}

        othercounter=0

        #start self  reprojecting
        for camname in camnames:

            img[camname] = np.zeros((480,640,3),dtype=np.uint8)
            reprojected[camname]={}


            #read image
            img[camname] = cv2.imread(imgdirectory+camname + "_rgb_" + str(count) + ".png")

            #pretty much a copy of cangalhoPnPDetector

            
            #finds markers
            det_corners, ids, rejected = aruco.FindMarkers(img[camname], intrinsics[camname]['rgb']['K'])
            
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
            
                #fetch valiid 2D points, squeezes and reshaped so that they can be easily read by pts2Ddisplay
                pts2D = np.squeeze(validcordners).reshape(len(validids)*4,2).T

                #this corresponds to the projected
                reprojected[camname][camname]=pts2D

                pts2DISPLAY2D = pts2D.astype(int)
                for j in range(pts2DISPLAY2D.shape[-1]):   

                    img[camname] = visu.paintImage(img[camname],[pts2DISPLAY2D[1,j],pts2DISPLAY2D[0,j]],offset=1,color=colors[othercounter])

                #fetch valid 3D Corners
                #holds the corner positions in the virgin model 
                detectedcornsobtainedmodel = np.zeros((3,len(validids)*4))

                for j in range(len(validids)):
                    idd = arucoData['idmap'][str(validids[j])]
                    #print(curcorners.shape)
                    #print(validids[j]*4,validids[j]*4+4)
                    #print(curcorners.T[:,validids[j]*4:validids[j]*4+4].shape)

                    #this correspond the the obtained model corners, in the virgin model
                    detectedcornsobtainedmodel[:,j*4:j*4+4] = curcorners.T[:,idd*4:idd*4+4]

                #save in dict 
                if len(validids)>0:
                    detectcorns[camname] = detectedcornsobtainedmodel

                #get current seen model R and T
                Rfull,otvec = aruco.GetCangalhoFromMarkersPnP(validids,validcordners,intrinsics[camname]['rgb']['K'],intrinsics[camname]['rgb']['D'],arucoData,arucoModel)

                
                
                R[camname] = Rfull
                t[camname] = otvec

            

            othercounter=othercounter+1



        #
        #print(R,t)
        print("ROTIFIER")

        brr =[]
        btt = []

        geometries = []
        
        #GET 3D VIZUALIZATION + CANGALHO ERRORS
        camcount = 0
        for camname in camnames:
                    
            
            if camname not in detectcorns:
                camcount = camcount + 1
                continue

            thiscamId = sceneModel['camnames'].index(camname)
            rotation = np.dot(sceneModel['R'][thiscamId],R[camname])
            translation = mmnip.Transform(t[camname],sceneModel['R'][thiscamId],sceneModel['t'][thiscamId])
 

            brr.append(rotation)
            btt.append(translation)

            camref = open3d.create_mesh_coordinate_frame(0.2, origin = [0, 0, 0])
            camref.transform(mmnip.Rt2Homo(sceneModel['R'][thiscamId],np.squeeze(sceneModel['t'][thiscamId])))
            geometries.append(camref)

            camera = visu.DrawCamera(sceneModel['R'][thiscamId],sceneModel['t'][thiscamId],color=colors[camcount],view=False)
            sphere = open3d.create_mesh_sphere(0.016)
            sphere.compute_vertex_normals()

            H = mmnip.Rt2Homo(rotation,np.squeeze(translation))

            #prints marker position estimates
            refe = open3d.create_mesh_coordinate_frame(0.1, origin = [0, 0, 0])
            refe.transform(H)
            sphere.transform(H)
            sphere.paint_uniform_color(colors[camcount])

            geometries.append(refe)
            geometries.append(sphere)
            geometries.append(camera)


            camcount = camcount+1

        #visu.draw_geometry(geometries,saveImg=True,saveName=mediapath + "/"+"SCENE"+"_"+str(i)+".png")
        

        #geometries = visu.ViewRefs(brr,btt,refSize=0.1),view = False
        
        cangalhotranslerr.append(MultTranslError(brr))
        rotrig,rotangle = MultRotError(brr)
        cangalhorotriguezerr.append(rotrig)
        cangalhorotangleerr.append(rotangle)


        normerr = {}
        #number of acitves cams per cam
        activecamcount = {}
        #number of corners
        nactivecorns = {}

        #start cross reprojecting
        for camname in camnames:

            minicounter = 0
            activecamcounter = 0
            

            print("LOOP",camname)
            for othercam in camnames:
                
                #check if it is not self, and if both cameras have some detected corners
                if othercam == camname or camname not in detectcorns or othercam not in detectcorns:
                    minicounter = minicounter + 1
                    continue

                activecamcounter = activecamcounter + 1
         
                
                thiscamId = sceneModel['camnames'].index(camname)
                othercamId = sceneModel['camnames'].index(othercam)


                #convert from virgin corns detected by this to other Cam coordinates
                newcorns = mmnip.Transform(detectcorns[camname],R[othercam],t[othercam])
                
                

                #convert from other Cam to this cam coordinates
                Rbetweencams = np.dot(sceneModel['R'][thiscamId].T,sceneModel['R'][othercamId])
                tbetweencams = np.dot(sceneModel['R'][thiscamId].T, sceneModel['t'][othercamId] - sceneModel['t'][thiscamId])


                newcorns = mmnip.Transform(newcorns,Rbetweencams,tbetweencams)


                
                #we now need to project points from every camera into this camera

                #get R and t in the other cam (no transformation is dones since all that is handled above)
                pts2Dobtained = cv2.projectPoints(newcorns.T, np.eye(3) , np.zeros((3,1)),intrinsics[camname]['rgb']['K'],np.zeros((5,1),dtype=float))[0].T
                pts2Dobtained = np.squeeze(pts2Dobtained)

                reprojected[camname][othercam] = pts2Dobtained


                #paint the image
                pts2DISPLAY2D = pts2Dobtained.astype(int)

   

                for j in range(pts2DISPLAY2D.shape[-1]):   
                    img[camname] = visu.paintImage(img[camname],[pts2DISPLAY2D[1,j],pts2DISPLAY2D[0,j]],offset=1,color=colors[minicounter])



                minicounter = minicounter + 1


                normerr[camname] = np.linalg.norm(reprojected[camname][camname] - reprojected[camname][othercam],axis=0)
                activecamcount[camname] = activecamcounter
                nactivecorns[camname] = reprojected[camname][camname][1]


            #sums the errors in single image
            fullreperr = 0




            #camcount
            for cam in camnames:

                #if camera has no detected corners
                if camname not in normerr:
                        minicounter = minicounter + 1
                        continue

                fullreperr = normerr[camname]/(activecamcount[camname]*nactivecorns[camname]) + fullreperr

            fullcrossreprojectionerror.append(fullreperr)
         

            #cv2.imshow('image',img[camname])
            #cv2.imwrite(mediapath + "/"+camname+"_"+str(i)+".png",img[camname])

            

        #cv2.imshow("Detected Markers",img)
        count = count + 1


        print("COUNT:",count)
        #reprojection error 
        curreprojectionerror = np.linalg.norm(pts2Dobtained - pts2D,axis=0)



        #get pose
            
        #for the synth, orvec is the estimated rotation, and Rfull is the ground truth rotation
        # for the real, orvec is saved model pnp estimated rotation and Rfull is the observer rotation from the image
        #the error rotation matrix

        '''
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
        '''


    print("Remove later")
    quit()
        
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