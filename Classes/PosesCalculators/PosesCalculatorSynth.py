import PosesCalculator
import numpy as np

class PosesCalculatorSynth(PosesCalculator.PosesCalculator):
    
    
    def __init__(self,data):

        PosesCalculator.PosesCalculator.__init__(self,data)


    def AddObservations(self,obsR,obsT):
        
        #add observations for R
        super(PosesCalculatorSynth,self).AddObservations(obsR,obsT)
        super(PosesCalculatorSynth,self).CalcRthenStartT()

        #add observations for t
        super(PosesCalculatorSynth,self).AddObservations(obsR,obsT)
        super(PosesCalculatorSynth,self).CalcT()