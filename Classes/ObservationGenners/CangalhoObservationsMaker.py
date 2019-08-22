from libs import *
import ObservationsMaker

class CangalhoObservationMaker(ObservationsMaker.ObservationsMaker):
    def __init__(self,data):

        print("Getting Cangalho")

        self.K=data['K']
        self.D=data['D']
        self.arucoData=data['arucodata']
        

        self.estimating ="R"

        #list of empty lists where observations will be saved (first dim tells camera, second dim is the observations for that cam)
        self.Allobs = []

        #N_objects
        self.N_objects = len(self.arucoData['ids'])

        self.count = 0

        self.arucoData['idmap'] = aruco.markerIdMapper(self.arucoData['ids'])

    def GetObservations(self,streamData):

        img = streamData['rgb'][0]

        img,ids,obsR,obsT = aruco.ArucoObservationMaker(img,self.K,self.D,self.N_objects,self.arucoData,captureR=True,captureT=True)


        return img,ids,obsR,obsT
        
