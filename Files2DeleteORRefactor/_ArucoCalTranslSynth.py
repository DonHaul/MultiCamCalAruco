import numpy as np
from libs import *

def main():

    R,t = synth.FakeArucoReal() #in world coordinates

    
    visu.ViewRefs(R,t,showRef=True)

    #importing R from the json file
    R1 = FileIO.getJsonFromFile("./tmp/R_badger_06-06-2019_00:17:54.json")['R']
    R1 = np.asarray(R1)
    R1 = np.split(R1,12,axis=0)

    RR = []

    for r in R1:
        RR.append(np.squeeze(r))

    R1=RR

    #Generate observations (Here u use R, which is coordinate of the markers relative to the camera )
    obsR,obst = synth.SampleGenerator(R,t,noise=1,samples=1000)


    # Translation Problem Definition
    A,b = probdefs.translationProbDef(obst,R1,len(t))

    #Solves Least Squres
    x= algos.LeastSquares(A,b)
    
    

    sol = np.split(x,len(t))

    newT = mmnip.Transl_fromWtoRef(R,sol)

    print(newT)
    visu.ViewRefs(R1,newT,showRef=True)
   

if __name__ == '__main__':
    main()