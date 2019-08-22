from libs import *
import numpy as np

class CangalhoProcrustesDetector:
    def __init__(self,data):


        self.arucoData=data['arucodata']
        self.arucoModel=data['arucomodel']
        self.arucoData['idmap'] = aruco.markerIdMapper(self.arucoData['ids'])



    def ArucoDetector(self,streamsingle,K,D):
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
        det_corners, ids, rejected = aruco.FindMarkers(streamsingle['rgb'], K)
        
        
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
                if ids[i] in self.arucoData['ids']:
                    #print("Valid marker id: "+str(ids[i]))
                    validids.append(ids[i])
                    validcordners.append(det_corners[i]) 

            #solves the procrustes problem for the markers given
            result = aruco.GetCangalhoFromMarkersProcrustes(validids,validcordners,K,self.arucoData,self.arucoModel,streamsingle['depth'])
            
            #if no R and t are retrieved, dont create observation
            if(result[0] is None):
                return obs,streamsingle['rgb']


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

        return obs,streamsingle['rgb']