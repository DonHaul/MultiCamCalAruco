from libs import *
import ObservationsMaker

class CangalhoSynthObsMaker(ObservationsMaker.ObservationsMaker):
    def __init__(self,data):

        

        self.R=data['synthmodel'][0]
        self.t=data['synthmodel'][1]
        

        self.n_obs = data['samples'] 
        self.noise = data['noise']
        self.noiset = data['noiset']
        

        



    def GetObservations(self,streamData):


        obsR,obst = synth.SampleGenerator(self.R,self.t,noise=self.noise,samples=self.n_obs,noiset=self.noiset)


        return None,None,obsR,obst
        
