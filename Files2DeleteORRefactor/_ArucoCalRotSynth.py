import numpy as np
from libs import *

def main():
    

    R,t = synth.FakeArucoReal() #in world coordinates

    #compare coordinates relative to ref 1 with output
    Raux = mmnip.genRotRelLeft(R)
    visu.ViewRefs(Raux)

    #sample observations between arucos
    obsR,obst = synth.SampleGenerator(R,t,noise=0.0,samples=30)

    #view observations
    #obsgen.ObsViewer(obsR,pause=False,show=False)

    #put in matrix form
    B = probdefs.rotationProbDef(obsR,len(R))

    C = np.dot(B.T,B) #C = B'B


    #solve svd
    rotSols = algos.RProbSolv1(C,3,len(R))
    #visu.ViewRefs(rotSols)

    #converts to world coordinates or into them
    rotSolsNotUsed = mmnip.Transposer(rotSols)
    
    #converts in first ref coordinates , 
    rr = mmnip.genRotRelLeft(rotSolsNotUsed)

    #print(rr)
    visu.ViewRefs(rr)

    #this is done so that it can be converted into json
    qrr=[]
    for r in rr:
        qrr.append(r.tolist())


    FileIO.putFileWithJson({'R':qrr},'R')


if __name__ == '__main__':
    main()