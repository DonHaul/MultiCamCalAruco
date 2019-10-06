'''
Script used to extract error metrics and images of calibrated scene
'''

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
    '''Initializes the error data struct with the wanted metrics

    Args:
        statstext - array with text of statistics (std, mean, median)
        measurestext - array with what is meausered (angle, translation, rodriguez)
    Returns;
        errordata[measuretext][stat]
    '''

    errorData={}

    for meas in measurestext:

        errorData[meas]={}

        for stat in statstext:
            errorData[meas][stat]=[]

    return errorData

def MultTranslError(trans,prints=False):
    '''Measures translation error in a combinatorial way
        if 4 trnalations are given
        tests following norms:
        1-2
        1-3
        1-4
        2-3
        2-4
        3-4
        and then divides by the number of norms

        the result corresponds to the average distance between arucos in different cameras

    Returns:
        norm
    '''

    if prints:
        print("== ALL TRANS ==")
        print(trans)
        print("=== NORMS ===")
    brah = 0
    
    cumnorm = 0

    for m in range(len(trans)):
        for n in range(m+1,len(trans)):
            brah = brah+1

            cumnorm = cumnorm + np.linalg.norm(trans[n]-trans[m])

            if prints:
                print(np.linalg.norm(trans[n]-trans[m]))

    if prints:
        print("RESULT",brah)
        print(cumnorm/brah)

    return cumnorm/brah

def MultRotError(rot,prints=False):
    '''Measures rotation error in a combinatorial way
        if 4 trnalations are given
        tests following norms:
        1-2
        1-3
        1-4
        2-3
        2-4
        3-4
        and then divides by the number of norms

        the result corresponds to the average distance between arucos in different cameras

    Returns:
        averaged normalized rodriguez between rotations   
        average angle betwwen rotations
    '''
    brah = 0
    
    curanglz = 0

    riguezerror=0

    if prints:
        print("== ALL ROTS ==")
    
        print(rot)
        print("=== ANGLES ===")


    for m in range(len(rot)):
        for n in range(m+1,len(rot)):
            brah = brah+1

            errorRot = np.dot(rot[m],rot[n].T) 

            val = ((np.trace(errorRot) - 1) / 2)
            
            
            
            if val > 1:
                val=1

            if prints:
                print(np.rad2deg(np.arccos(val)))

            #angle error
            curanglz = curanglz + np.rad2deg(np.arccos(val))

            rodrize = cv2.Rodrigues(errorRot)[0]
            

            #rodriguez error (norm of rotation vector)
            riguezerror = riguezerror + np.linalg.norm(rodrize)

    if prints:
        print("== FULL ERR ==")
        print(curanglz/brah)
        visu.ViewRefs(R=rot)

    return riguezerror/brah , curanglz/brah


           
        


def GenSimpleStats(errorMeasure,measurename,errorData,statstext=None):
    '''
    Calculates basic statistics about the measurement
    '''
    
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

    #fetches info about images
    info =  FileIO.getJsonFromFile(imgdirectory+"info.json")
    frames = info['count']
    camnames = info['camnames']
    
    #fetch intrinsics
    intrinsics = FileIO.getIntrinsics(camnames)

    #get media path
    mediapath = FileIO.CreateFolder("../Media2/reprojections",putDate=True)

    #get basic pipeline information
    data =  FileIO.getJsonFromFile( path + "/pipeline.json" )

    #get basic arucoData
    arucoData =  FileIO.getJsonFromFile(data['model']['arucodata'])

    #get scene position
    sceneModel = FileIO.getFromPickle( path + "/poses.pickle" )
    
    #get estimated aruco model
    arucoModel = FileIO.getFromPickle( data['model']['arucomodel'])

    #get all corners
    curcorners = arucoModel['corners']
    
    #create the idmap
    arucoData['idmap'] = aruco.markerIdMapper(arucoData['ids'])    
 

    #measures that will be  used
    measurestext=['corner','center','reprojection','translation','angle','rodriguez']
    statstext=['singular','total','avg','median','std']

    #if errorData is None:
    errorData = InitializeStats(statstext,measurestext)

        
    #reprojection error
    allnoisypts2D=[]

  
    #initialized error metrics
    reprojectionerrors= np.array([])
    translationerrors = []
    rodriguezerrors = []
    ndetectedarucos = []
    angleerrors = []

    nactivecams = []

    #initialize counters
    count = 0
    imcount =0

    #colors correspondent to each camera
    colors=[]
    colors.append([1, 1, 0])
    colors.append([1,0,1])
    colors.append([0,1,1])
    colors.append([0.5,0.5,0])
    colors.append([0.5,0,0.5])


    #initialized more error metrics
    cangalhotranslerr = []
    cangalhorotriguezerr = []
    cangalhorotangleerr = []
    fullcrossreprojectionerror =[]

    crossreprojectionerr={}
    reprojectedcornes={}



    #OBSERVATION GENNER PART
    frameservations = []


    #initialize cross reprojection error
    #initialize number of corns per cam
    for cam in camnames:
        crossreprojectionerr[cam]=[]
        reprojectedcornes[cam] = []

    #go throuth every image or generate frames and corners detected on their own images
    for i in range(0,frames):

        print("FRAME",i)

        img={}
        reprojected = {}
        R ={}
        t={}
        detectcorns = {}

        othercounter=0

        #start self  reprojecting
        for camname in camnames:

            #initialize new empty image for a camera
            img[camname] = np.zeros((480,640,3),dtype=np.uint8)

            #initialize where corners will be saved in this frame for each camera
            reprojected[camname]={}


            #read image
            img[camname] = cv2.imread(imgdirectory+camname + "_rgb_" + str(i) + ".png")
            
            #if(camname == "diavolo"):
            #    cv2.imshow('image',img[camname])
            #    cv2.waitKey(0)

            #finds markers
            det_corners, ids, rejected = aruco.FindMarkers(img[camname], intrinsics[camname]['rgb']['K'])
            

            #FUCTION: fetch valid ids and corners
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

                #END_FUCTION

                #this corresponds to the projected self corners
                reprojected[camname][camname]=pts2D

                #FUNCTION paint image:
                pts2DISPLAY2D = pts2D.astype(int)
                for j in range(pts2DISPLAY2D.shape[-1]):   

                    img[camname] = visu.paintImage(img[camname],[pts2DISPLAY2D[1,j],pts2DISPLAY2D[0,j]],offset=1,color=colors[othercounter])                    
                #END_FUCTION




                #fetch valid 3D Corners
                #holds the corner positions in the virgin model 
                detectedcornsobtainedmodel = np.zeros((3,len(validids)*4))

                #Fetches all virgin valid corners
                for j in range(len(validids)):
                    idd = arucoData['idmap'][str(validids[j])]

                    #this correspond the the obtained model corners, in the virgin model
                    detectedcornsobtainedmodel[:,j*4:j*4+4] = curcorners.T[:,idd*4:idd*4+4]

                #save the 3D corners in the dictionary
                if len(validids)>0:
                    detectcorns[camname] = detectedcornsobtainedmodel

                #get current seen model R and T
                Rfull,otvec = aruco.GetCangalhoFromMarkersPnP(validids,validcordners,intrinsics[camname]['rgb']['K'],intrinsics[camname]['rgb']['D'],arucoData,arucoModel)

                #print(camname,Rfull)
                #save it
                R[camname] = Rfull
                t[camname] = otvec           
                
            
            #counter used for color switching
            othercounter=othercounter+1




        #FUNCTION 3D Visualization and cangalho errors

        #holders for rotations and translations
        brr =[]
        btt = []

        geometries = []
        
        #GET 3D VIZUALIZATION + CANGALHO ERRORS

        #counter for colors
        camcount = 0

        for camname in camnames:

            #get camera id
            thiscamId = sceneModel['camnames'].index(camname)

            if view:

                #create camera coordinate frame
                camref = open3d.create_mesh_coordinate_frame(0.2, origin = [0, 0, 0])
                camref.transform(mmnip.Rt2Homo(sceneModel['R'][thiscamId],np.squeeze(sceneModel['t'][thiscamId])))
                geometries.append(camref)

                #create camera model in scene
                camera = visu.DrawCamera(sceneModel['R'][thiscamId],sceneModel['t'][thiscamId],color=colors[camcount],view=False)
                geometries.append(camera)
                
            
            if camname not in detectcorns:
                camcount = camcount + 1
                continue
            


            #get cangalho rotation in this camera's coordinate system
            rotation = np.dot(sceneModel['R'][thiscamId],R[camname])
            
            #get cangalho translation in the worlds' camera coordinate system
            translation = mmnip.Transform(t[camname],sceneModel['R'][thiscamId],sceneModel['t'][thiscamId])
 
            #save 'em
            brr.append(rotation)
            btt.append(translation)

            #FUNCTION - VISUALIZATION

            if view:
            
                #create sphere at cangalho detected position
                sphere = open3d.create_mesh_sphere(0.016)
                sphere.compute_vertex_normals()
                H = mmnip.Rt2Homo(rotation,np.squeeze(translation))
                sphere.transform(H)
                sphere.paint_uniform_color(colors[camcount])

                #create coordinate system at cangalho detected position
                refe = open3d.create_mesh_coordinate_frame(0.1, origin = [0, 0, 0])
                refe.transform(H)

                

                #add them to scene
                geometries.append(refe)
                geometries.append(sphere)
                #open3d.draw_geometries(geometries)

            camcount = camcount+1

        
        #view 'em
        if view:
            visu.draw_geometry(geometries,saveImg=True,saveName=mediapath + "/"+"SCENE"+"_"+str(i)+".png")
        
        
        #save translation error for the frame        
        cangalhotranslerr.append(MultTranslError(btt))

        #save roation errors
        rotrig,rotangle = MultRotError(brr)
        cangalhorotriguezerr.append(rotrig)
        cangalhorotangleerr.append(rotangle)



        #FUNCTION:Cross ReProjection 

        normerr = {}
        #number of acitves cams per cam
        activecamcount = {}
        #number of corners
        nactivecorns = {}

        printRep=False

        if printRep:
            print("========================= REPROS ==")

        #start cross reprojecting
        for camname in camnames:

            minicounter = 0
            activecamcounter = 0

            normerr[camname] = []
            
            if printRep:
                print(camname)

            #print("LOOP",camname)
            for othercam in camnames:
                


                #check if it is not self, and if both cameras have some detected corners
                if othercam == camname or camname not in detectcorns or othercam not in detectcorns:
                    minicounter = minicounter + 1
                    continue
                
                #add active camera (used to normalize later)
                activecamcounter = activecamcounter + 1
         
                #get camera ids
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

                #and save them
                reprojected[camname][othercam] = pts2Dobtained


                #FUNCTION: Paint Image
                pts2DISPLAY2D = pts2Dobtained.astype(int)
   
                for j in range(pts2DISPLAY2D.shape[-1]):   
                    img[camname] = visu.paintImage(img[camname],[pts2DISPLAY2D[1,j],pts2DISPLAY2D[0,j]],offset=1,color=colors[minicounter])


                minicounter = minicounter + 1

                if printRep:
                    print("OTHERCAM",othercam)
                    print("== NORMS ==")
                    print(np.linalg.norm(reprojected[camname][camname] - reprojected[camname][othercam],axis=0))
                    print("== AVG NORM ==")
                    print(np.mean(np.linalg.norm(reprojected[camname][camname] - reprojected[camname][othercam],axis=0)))


                #add norms measure from this camera to all other cameras
                normerr[camname] = normerr[camname] + np.linalg.norm(reprojected[camname][camname] - reprojected[camname][othercam],axis=0).tolist()

            if printRep:               
                print("ALL CONTRIBS FOR", camname)
                print(np.mean(normerr[camname]))



            #number of corners detected per camera
            if camname in detectcorns:
                reprojectedcornes[camname].append(reprojected[camname][camname].shape[1])
            else:
                reprojectedcornes[camname].append(0)

            #sums the errors in single image
            fullreperr = 0


        #camcount
        togethererr = []
        for cam in camnames:

            #if camera has no detected corners
            if camname not in normerr:
                    minicounter = minicounter + 1
                    continue

            togethererr = togethererr + normerr[camname]

            #(activecamcount[camname]*nactivecorns[camname]) is used to normalize the obtained error to the 

        fullcrossreprojectionerror.append(np.sum(togethererr)/len(togethererr))
        


        allcamObs = [ [] for q in range(len(camnames)) ]
        #print(allcamObs)
        
        #iterate throguh cameras
        for cam  in camnames:

            camId = sceneModel['camnames'].index(cam)

            if cam in R:
                obs = {"obsId":0,"R":R[cam],"t":t[cam]}
                #print("Yare Yare daze",cam)
                #print(obs)
                #get new observations of that camera

                #only if contains valid observation
                if obs['R'] is not None:
                    allcamObs[camId]=[obs]  # WRONG SHOULD IT BE concantenate lists OR =?



        obsR , obsT = obsgen.GenerateCameraPairObs(allcamObs,arucoModel['R'],arucoModel['t'])



        frameservations.append({"frame":i,"obsR":obsR,"obsT":obsT})

        #print(frameservations)

        #get pose
            
        #for the synth, orvec is the estimated rotation, and Rfull is the ground truth rotation
        #for the real, orvec is saved model pnp estimated rotation and Rfull is the observer rotation from the image
        #the error rotation matrix

    #add everythin to return data structunre
    errorData["reprojectedcornes"] = reprojectedcornes
    errorData["cangalhotranslerr"] = cangalhotranslerr
    errorData["cangalhorotriguezerr"] = cangalhorotriguezerr
    errorData["cangalhorotangleerr"] = cangalhorotangleerr   
    errorData["reprojectionNormAvg"] = fullcrossreprojectionerror
    errorData["frameservations"] = frameservations

    return errorData





if __name__ == "__main__":
    path = sys.argv[1]
    
    imagepath = None

    if (len(sys.argv)>2):
        imagepath = sys.argv[2]

    errorData = main(path,imagepath)

    frames =  FileIO.getJsonFromFile(imagepath+"/info.json")['count']

    #get camnames
    camnames =  FileIO.getJsonFromFile(imagepath+"info.json")['camnames']




    moremetrics = []



    #get scene position
    sceneModel = FileIO.getFromPickle( path + "/poses.pickle" )


    ndetectmeas = []
    
    for cam in sceneModel['camnames']:
        ndetectmeas.append(cam + "_" + "N_detectedCorns")




    cametrics = ["terr","Rerrangle","Rriguez"]

    for met in cametrics:
        for l in range(0,len(camnames)):
            for m in range(l+1,len(camnames)):
                moremetrics.append(met + "_" + camnames[sceneModel['camnames'].index(camnames[l])]+"_"+ camnames[sceneModel['camnames'].index(camnames[m])])
                    
    

    

    #write csv
    with open(path+'/errors.csv', 'wb') as csvfile:
        filewriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
          
          

        #write to file
        fullmeasures = []
        
        
        count = 0
        

        m = 'reprojection'
        
        #gen n detected arucos
        statstext=['total','avg','median','std']
        for stat in statstext:
            fullmeasures.append(m+"_"+stat+" [px]")
       
            
        filewriter.writerow(['frame'] + ndetectmeas + ['tcangalho','Rcangalhoangle','|Rcangalhorodrigues|' , "reprojectionNormAvg"] + moremetrics)

        for i in range(frames):

            #load cornes for this one
            ndetectedN = []
            for cam in sceneModel['camnames']:
                ndetectedN.append(errorData["reprojectedcornes"][cam][i])


            #initialize error matrices
            combosRotErrRiguez = np.zeros((len(camnames),len(camnames)))
            combosRotErrAngle = np.zeros((len(camnames),len(camnames)))
            combosTranslErr = np.zeros((len(camnames),len(camnames)))


            #process observations
            #this are already in scenemodel id format
            for  obsR,obsT in zip(errorData["frameservations"][i]['obsR'],errorData["frameservations"][i]['obsT']):
                
                #print("===== CUR ANGLE ======",obsR['to'],obsR['from'])
                
                #get camera ids
                Rbetweencams = np.dot(sceneModel['R'][obsR['to']].T,sceneModel['R'][obsR['from']])

                errorRot = np.dot(Rbetweencams.T,obsR['R'])

                val = ((np.trace(errorRot) - 1) / 2)
            
            
                if val > 1:
                    val=1

                curanglz = np.rad2deg(np.arccos(val))
                
                #print(curanglz)

                rodrize = cv2.Rodrigues(errorRot)[0]
                riguezerror = np.linalg.norm(rodrize)

                combosRotErrRiguez[obsR['to'],obsR['from']] = riguezerror
                combosRotErrRiguez[obsR['from'],obsR['to']] = riguezerror

                combosRotErrAngle[obsR['to'],obsR['from']] = curanglz
                combosRotErrAngle[obsR['from'],obsR['to']] = curanglz

                #translationthingz
                if (obsR['to'] != obsT['to']) or  (obsR['from'] != obsT['from']):
                    print("OBSERVATIONS DO NOT MATCH")
                    print("EXITING EARLY")
                    quit()

                        
                
                tbetweencams = np.dot(sceneModel['R'][obsR['to']].T, sceneModel['t'][obsR['from']] - sceneModel['t'][obsR['to']])
                translerr = np.linalg.norm(np.squeeze(tbetweencams) - obsT['t'])                

                #print("T tests")
                #print(np.squeeze(tbetweencams))
                #print(obsT['t'])
                #print(np.squeeze(tbetweencams) - obsT['t'])
                #print("Result")
                #print(translerr)


                combosTranslErr[obsR['from'],obsR['to']] = translerr
                combosTranslErr[obsR['to'],obsR['from']] = translerr

            #   cametrics = ["terr","Rerrangle","Rriguez"]
            #assing the roations
            terr = []
            RerrAng = []
            RerrRod = []
            for l in range(0,len(camnames)):
                for m in range(l+1,len(camnames)):
                    #print(sceneModel['camnames'].index(camnames[l]))
                    #print(camnames[sceneModel['camnames'].index(camnames[l])])

                    terr.append(combosTranslErr[sceneModel['camnames'].index(camnames[l]),sceneModel['camnames'].index(camnames[m])])
                    RerrAng.append(combosRotErrAngle[sceneModel['camnames'].index(camnames[l]),sceneModel['camnames'].index(camnames[m])])
                    RerrRod.append(combosRotErrRiguez[sceneModel['camnames'].index(camnames[l]),sceneModel['camnames'].index(camnames[m])])
                    
       

                    
       

                   
            filewriter.writerow([i] +  ndetectedN  + 
            [ errorData["cangalhotranslerr"][i], errorData["cangalhorotangleerr"][i],errorData["cangalhorotriguezerr"][i]] + 
            [errorData["reprojectionNormAvg"][i]] + 
            terr + RerrAng + RerrRod)


        #filewriter.writerow([] + repMeasures + [errorData['translation']['singular'][i],errorData['rodriguez']['singular'][i],errorData['angle']['singular'][i] ])




    print("Saving Json...")
    FileIO.saveAsPickle('errors',errorData,path,False,False)