import PosesCalculator
import numpy as np
import time

class OulierRemovalPoseCalculator(PosesCalculator.PosesCalculator):

    
    def __init__(self,data):

        PosesCalculator.PosesCalculator.__init__(self,data)

        self.n_obs = np.zeros((self.N_objects,),dtype=np.int32)

        self.obsthreshold=data['observations']
        self.Rcutoff = data['Rcutoff']
        self.Tcutoff = data['Tcutoff']

        self.Rguess=[]
        self.Tguess=[]



        self.records = self.recordRT
        self.recordRT=False

        self.initialguess=True

    def AddObservations(self,obsR,obsT):
        

        for o in obsR:
            self.n_obs[o['to']]=self.n_obs[o['to']]+1
            self.n_obs[o['from']]=self.n_obs[o['from']]+1

        if self.initialguess==True: 


            
            super(OulierRemovalPoseCalculator,self).AddObservations(obsR,obsT)

            if np.count_nonzero(self.n_obs<self.obsthreshold) ==0:

                #reset
                self.n_obs = np.zeros((self.N_objects,),dtype=np.int32)

                if(self.estimating =='R'):
                    self.Rguess = self.CalcRthenStartT()
                elif (self.estimating =='t'):
                    self.Tguess = self.CalcT()
                    
                    self.initialguess=False
                    self.estimating='R'
                    print("STARTING CAPTURE NOW")
                    time.sleep(1)
                    

                    if self.records:
                        self.recordRT=True
                    super(OulierRemovalPoseCalculator,self).Clear()
                    
        else:

            obsRR=[]
            obsTT=[]

            for oR,oT in zip(obsR,obsT):
                    


                    RR =  oR['R']-np.dot(self.Rguess[oR['to']].T,self.Rguess[oR['from']])
                    Rnorm = np.linalg.norm(RR)

                    if Rnorm<self.Rcutoff:
                        obsRR.append(oR)

                    ttt = np.dot(self.Rguess[oT['to']].T,(self.Tguess[oT['from']]-self.Tguess[oT['to']]))
                    ttnorm = np.linalg.norm(np.squeeze(oT['t'])-np.squeeze(ttt))

                    if ttnorm<self.Tcutoff:
                        obsTT.append(oT)
        
            
            super(OulierRemovalPoseCalculator,self).AddObservations(obsRR,obsTT)
            
                


