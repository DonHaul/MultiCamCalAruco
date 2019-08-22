from libs import *

class CangalhoPnPDetector:
    def __init__(self,data):


        self.arucoData=data['arucodata']
        self.arucoModel=data['arucomodel']
        self.arucoData['idmap'] = aruco.markerIdMapper(self.arucoData['ids'])



    def ArucoDetector(self,streamsingle,K,D):
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

            for i in range(0,len(ids)):
                if ids[i] in self.arucoData['ids']:
    
                    validids.append(ids[i])
                    validcordners.append(det_corners[i]) 
        
            Rr,tt = aruco.GetCangalhoFromMarkersPnP(validids,validcordners,K,D,self.arucoData,self.arucoModel)

            if(Rr is None):
                return obs,streamsingle['rgb']

            #initializes observation
            o ={"obsId":0} #since it will always generate observation on id 0

                    #generate R observations
            o['R']=Rr


            #generate t observations
            o['t']=tt #WRONG - Not sure if this is the correct t

            #print(o['t'])
            obs.append(o)

        return obs,streamsingle['rgb']