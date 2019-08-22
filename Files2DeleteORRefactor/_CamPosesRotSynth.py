import open3d
import numpy as np
from libs import *


def main():


    Rcam, tcam = synth.TestScene51()

    #just to compare
    Raux = mmnip.genRotRelLeft(Rcam)
    visu.ViewRefs(Raux)


    #ARUCO MODEL (It can be whatever you want)
    R,t = synth.TiltedCams()

    #similar to output from ROS, gets observations from Marker in the camera coordinate
    camsObs = synth.MultiCamSampleGeneratorFixed(Rcam,tcam,R,t,noise=1)

    obsR, obsT = obsgen.GenerateCameraPairObs(camsObs,R,t)

    B = probdefs.rotationProbDef(obsR,len(Rcam))  #95% confidence that it is correct

    C = np.dot(B.T,B) #C = B'B

    
    rotSols = algos.RProbSolv1(C,3,len(Rcam),canFlip=True) 

    #converts to world coordinates or into them
    rotSolsNotUsed = mmnip.Transposer(rotSols)
    #visu.ViewRefs(rotSolsNotUsed)

    #converts in first ref coordinates , 
    rr = mmnip.genRotRelLeft(rotSolsNotUsed)

    #saves in file
    qrr=[]
    for r in rr:
        qrr.append(r.tolist())

    visu.ViewRefs(rr)

    FileIO.putFileWithJson({'R':qrr},'R')





if __name__ == '__main__':
    main()