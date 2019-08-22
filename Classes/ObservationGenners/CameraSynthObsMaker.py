from libs import *
import ObservationsMaker

class CameraSynthObsMaker(ObservationsMaker.ObservationsMaker):
    def __init__(self,data):

        self.noiset=data['noiset']

        self.Rcangalho=data['synthmodel'][0]
        self.tcangalho=data['synthmodel'][1]

        self.Rcam=data['modelscene'][0]
        self.tcam=data['modelscene'][1]
        

        self.n_obs = data['samples'] 
        self.noise = data['noise']
        self.noiset = data['noiset']
        

        


    def GetObservations(self,streamData):



        #similar to output from ROS, gets observations from Marker in the camera coordinate
        camsObs = synth.MultiCamSampleGeneratorFixed(self.Rcam,self.tcam,self.Rcangalho,self.tcangalho,nObs = self.n_obs,noise=self.noise, noiset=self.noiset)

        obsR, obsT = obsgen.GenerateCameraPairObs(camsObs,self.Rcangalho,self.tcangalho)



        return None,None,obsR,obsT
        
