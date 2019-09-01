"""
observationgenner.py

This module contains functions that make pairs of observations
"""
import numpy as np
import matmanip as mmnip
import aruco
import cv2
import libs.helperfuncs as helperfuncs
import visu


def ArucoRealObsGenner(ids,rots,tvecs,arucoData=None,captureT=True,captureR=True):
    #TURN THIS INTO A FUNCTION IN THE FUTURE NOW

    observsR=[]
    observsT=[]
    #make observations
    #generates samples
    for i in range(0,len(ids)):                
        for j in range(i+1,len(ids)):
            
            if arucoData is not None:             
                
                #only valid markers
                if ids[i] not in arucoData['ids']:
                    print("Invalid marker id: "+str(ids[i]))

                    continue 

                                #only valid markers
                if ids[j] not in arucoData['ids']:
                    print("Invalid marker id: "+str(ids[j]))

                    continue 

            #print("observing "+str(i)+" and "+str(j))

    
            #generate R observations
            if(captureR):
                
                #for real data
                if arucoData is not None:
                    obsR={"to":arucoData['idmap'][str(ids[i])],"from":arucoData['idmap'][str(ids[j])],"R":np.dot(rots[i].T,rots[j])}
                else:
                    #synth data
                    obsR={"to":ids[i],"from":ids[j],"R":np.dot(rots[i].T,rots[j])}
                    
                observsR.append(obsR) 
            
            
            
            if(captureT):
                
                
                
                #generate t observations
                if arucoData is not None:
                    obsT={"from":arucoData['idmap'][str(ids[i])],"to":arucoData['idmap'][str(ids[j])],"t":np.squeeze(np.dot(rots[j].T,(tvecs[i]-tvecs[j]).T))} 
                else:
                    #synth data
                    obsT={"to":ids[j],"from":ids[i],"t":np.squeeze(np.dot(rots[j].T,(tvecs[i]-tvecs[j]).T))}


                observsT.append(obsT)

        return observsR,observsT



def ObsViewer(obs,key="R",fro="from",to="to",pause=True,show=False):
    '''
    Used to view observations

    Args:
        obs: observations to view
        key: what property of that observation do you want to see
        fro: from - in case u want other indexing
        to: to - in case u want other indexing
        pause: if you wish to see one observation at a time
        shpow: if you with to see an image of that obeservation
    '''
    
    for o in obs:
        print("from:" + str(o[fro])+" to:"+str(o[to]))
        print("Det is:" + str(np.linalg.det(o[key])))
        print(o[key])
        if show:
            visu.ViewRefs([np.eye(3),o[key]])
        if pause:
            raw_input()



def Cam2ArucoObsMaker(img,K,D,markerIDoffset,Nmarkers):
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
    det_corners, ids, rejected = aruco.FindMarkers(img, K)

    #changes image
    hello = img.astype(np.uint8).copy() 
    hello = cv2.aruco.drawDetectedMarkers(hello,det_corners,ids)
    
    #list of all observations generated
    observations =[]

    #if more than one marker was detected
    if  ids is not None and len(ids)>1:

        #finds rotations and vectors and draws referentials on image
        rots,tvecs,img = aruco.FindPoses(K,D,det_corners,hello,len(ids),arucoData['size'])

        #squeeze
        ids = ids.squeeze()


        #generates samples
        for i in range(0,len(ids)):                
                 
                 #only valid markers
                if ids[i] not in arucoData['ids']:
                    print("Invalid marker id: "+str(ids[i]))
                    continue 

                #initializes observation
                o ={"obsId":arucoData['idmap'][str(ids[i])]}

                #generate R observations
                o['R']=rots[i]

                #generate t observations
                o['t']=np.squeeze(tvecs[i]) 
                
                observations.append(o)
 
    return observations ,img


def CreateObservation(fromm,to,key,value):
    
    #initializes observation
    o ={"from":fromm,"to":to}

    #generate R observations
    o[key]=value
 

    return o

def CamArucoProcrustesObsMaker(img,K,D,arucoData,arucoModel,depth):
    '''
    Generates cangalho observations based on the aruco corners

    Args:
        img: image to extract markers from
        K: intrinsic parameters
        D: distortion
        arucoData: ids and siz of arucos
        arucoModel: poses of arucos in cangalho
        depth: depth image
    '''


    obs = []

    #finds markers
    det_corners, ids, rejected = aruco.FindMarkers(img, K)
    
    
    #in case there is only 1 id, convert it into a list with 1 element
    if ids is not None:

        ids = ids.squeeze()

        if (helperfuncs.is_empty(ids.shape)):
            ids=[int(ids)]



    if  ids is not None and len(ids)>0:

        #filter ids and cornerds
        validids=[]
        validcordners= []
   
        #fetches only ids that are on the cangalho
        for i in range(0,len(ids)):
            if ids[i] in arucoData['ids']:
                #print("Valid marker id: "+str(ids[i]))
                validids.append(ids[i])
                validcordners.append(det_corners[i]) 

        #solves the procrustes problem for the markers given
        result = aruco.GetCangalhoFromMarkersProcrustes(validids,validcordners,K,arucoData,arucoModel,depth)
        
        #if no R and t are retrieved, dont create observation
        if(result[0] is None):
            return obs,img


        #fetch result
        Rr = result[0]
        tt = result[1]

        Rr=Rr.T



        #initializes observation
        #o ={"obsId":arucoData['idmap'][str(ids[0])]}
        o ={"obsId":0} #since it will always generate observation on id 0

                #generate R observations
        o['R']=Rr


        #generate t observations
        o['t']=np.expand_dims(tt,axis=1) #WRONG - Not sure if this is the correct t

        #print(o['t'])
        obs.append(o)

    return obs,img

def CamArucoPnPObsMaker(img,K,D,arucoData,arucoModel):

    obs = []

    #finds markers
    det_corners, ids, rejected = aruco.FindMarkers(img, K)
    
    

    #in case there is only 1 id, convert it into a list with 1 element
    if ids is not None:

        ids = ids.squeeze()

        if (helperfuncs.is_empty(ids.shape)):
            ids=[int(ids)]


    if  ids is not None and len(ids)>0:

        #filter ids and cornerds
        validids=[]
        validcordners= []

        for i in range(0,len(ids)):
            if ids[i] in arucoData['ids']:
  
                validids.append(ids[i])
                validcordners.append(det_corners[i]) 
    
        Rr,tt = aruco.GetCangalhoFromMarkersPnP(validids,validcordners,K,D,arucoData,arucoModel)

        if(Rr is None):
            return obs,img

        #initializes observation
        o ={"obsId":0} #since it will always generate observation on id 0

                #generate R observations
        o['R']=Rr


        #generate t observations
        o['t']=tt #WRONG - Not sure if this is the correct t

        #print(o['t'])
        obs.append(o)

    return obs,img


def GenerateCameraPairObs(camsObs,R,t):
    '''
    Generate observations between 2 cameras, by doing Transformations throught the aruco

    camObs (list of list of dicts) - first list dimensions tells us the camera, the second list is all the observations for that camera
    R - rotations of the aruco model
    t - translation of the aruco model
    '''

    #initialize observation lists
    obsR = []
    obsT = []


    #this double for loop makes all camera combinations
    #between one camera
    for i in range(0,len(camsObs)):
        #and another camera
        for j in range(i+1,len(camsObs)):
            
            #this double loop matches every possible observation in each camera        
            #go through all the obs of one camera
            for obsiR in camsObs[i]:
                #and through all the obs of the other
                for obsjR in camsObs[j]:
                
                    #confusing as fuck i, know
                    # pretty much we have Rcam_i -> obsId_i and Rcam_j -> obsId_j   - to what each camera is observating is alwaying
                    # 'ObsId' = 'to' , and the cameraId on the array is the 'from'


                    obsR.append({"from":i,"to":j,"R": (np.linalg.multi_dot([obsiR['R'],R[obsiR['obsId']].T,R[obsjR['obsId']],obsjR['R'].T])).T})

                    #Get aruco transformation parameters
                    Rbetweenaruco = np.dot(R[obsjR['obsId']].T,R[obsiR['obsId']])
                    tbetweenaruco = np.dot(R[obsjR['obsId']].T, t[obsiR['obsId']] - t[obsjR['obsId']])

                    #transform from marker1  coordinates to marker2 coordinates
                    new_t =  mmnip.Transform(mmnip.InvertT(obsiR['R'], obsiR['t']),Rbetweenaruco, tbetweenaruco)



                    #transform from marker2 coordinates to camera j coordinates                    
                    tij = mmnip.Transform(new_t, obsjR['R'], obsjR['t'] )

                    print(i,j)

                    obsT.append({"from":i,"to":j,"t": np.squeeze(tij)})

    return obsR,obsT

def FilterGoodObservationMarkerIds(obs,R,t,N,t_threshold=0.08,R_threshold=0.5):
    '''
    DEPRECATED,
    used to eliminate aruco observations that are bad
    
    '''

    oopsies = np.zeros((N,))
    observed = np.zeros((N,))

    for i in range(0,len(obs)):
        #and another camera
        for j in range(i+1,len(obs)):
            
            #print("M1: " + str(obs[j]['obsId'])+ " M2: " + str(obs[i]['obsId']))


            Raux = np.linalg.multi_dot([obs[i]['R'],R[obs[i]['obsId']].T,R[obs[j]['obsId']],obs[j]['R'].T])

            #Get aruco transformation parameters
            Rbetweenaruco = np.dot(R[obs[j]['obsId']].T,R[obs[i]['obsId']])
            tbetweenaruco = np.dot(R[obs[j]['obsId']].T, t[obs[i]['obsId']] - t[obs[j]['obsId']])
            #transform from marker1  coordinates to marker2 coordinates
            new_t =  mmnip.Transform(mmnip.InvertT(obs[i]['R'], obs[i]['t']),Rbetweenaruco, tbetweenaruco)
            #transform from marker2 coordinates to camera j coordinates                    
            taux = mmnip.Transform(new_t, obs[j]['R'], obs[j]['t'] )

            observed[obs[i]['obsId']]=1
            observed[obs[j]['obsId']]=1
            

            if np.linalg.norm(taux) > t_threshold or (np.linalg.norm(np.eye(3) - Raux))>R_threshold:
                oopsies[obs[i]['obsId']] = oopsies[obs[i]['obsId']] + 1
                oopsies[obs[j]['obsId']] = oopsies[obs[j]['obsId']] + 1

    goodObservations=[]

    activeMarkersCount = np.count_nonzero(observed)

    for o in obs:
        if oopsies [ o['obsId']]<activeMarkersCount-1:
            #print("good sample found")
            goodObservations.append(o)
            

    return goodObservations

def GenerateCameraPairObsSelf(camsObs,R,t):
    '''
    Generate observations between 2 cameras, by doing Transformations throught the aruco

    camObs (list of list of dicts) - first list dimensions tells us the camera, the second list is all the observations for that camera
    R - rotations of the aruco model
    t - translation of the aruco model
    '''

    #initialize observation lists
    obsR = []
    obsT = []

    


    #this double for loop makes all camera combinations
    #between one camera
    for i in range(0,len(camsObs)):
        #and another camera
        for j in range(0,len(camsObs)):
            
            #this double loop matches every possible observation in each camera        
            #go through all the obs of one camera
            for obsiR in camsObs[i]:
                #and through all the obs of the other
                for obsjR in camsObs[j]:

                    if obsjR['obsId']==obsiR['obsId']:
                        print("skip")
                        continue
                    print("M1: " + str(obsjR['obsId'])+ " M2: " + str(obsiR['obsId']))


                    #confusing as fuck i, know
                    # pretty much we have Rcam_i -> obsId_i and Rcam_j -> obsId_j   - to what each camera is observating is alwaying
                    # 'ObsId' = 'to' , and the cameraId on the array is the 'from'
                    
                    #print("from camera:"+str(j)+" to camera:"+str(i))
                    #print(np.linalg.multi_dot([obsiR['R'].T,R[obsiR['obsId']],R[obsjR['obsId']].T,obsjR['R']]))
                    #raw_input()

                    obsR.append({"from":j,"to":i,"R": np.linalg.multi_dot([obsiR['R'],R[obsiR['obsId']].T,R[obsjR['obsId']],obsjR['R'].T])})

                    #Get aruco transformation parameters
                    Rbetweenaruco = np.dot(R[obsjR['obsId']].T,R[obsiR['obsId']])
                    tbetweenaruco = np.dot(R[obsjR['obsId']].T, t[obsiR['obsId']] - t[obsjR['obsId']])

                    #transform from marker1  coordinates to marker2 coordinates
                    new_t =  mmnip.Transform(mmnip.InvertT(obsiR['R'], obsiR['t']),Rbetweenaruco, tbetweenaruco)

                    
                    #transform from marker2 coordinates to camera j coordinates                    
                    tij = mmnip.Transform(new_t, obsjR['R'], obsjR['t'] )

                    obsT.append({"from":i,"to":j,"t": tij})
                    print("from:" + str(i) + " to: " +str(j))
                    print("rot")
                    print( np.linalg.multi_dot([obsiR['R'],R[obsiR['obsId']].T,R[obsjR['obsId']],obsjR['R'].T]))
                    print(np.linalg.norm(np.eye(3) - np.linalg.multi_dot([obsiR['R'],R[obsiR['obsId']].T,R[obsjR['obsId']],obsjR['R'].T])))
                    print("tij")
                    print(tij)
                    print(np.linalg.norm(tij))


    return obsR,obsT