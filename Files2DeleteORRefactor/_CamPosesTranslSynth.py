import open3d
import numpy as np
from libs import *

def main():

    #mesh = read_triangle_mesh("models/filmCamera.ply")
    #mesh.compute_vertex_normals()
    #mesh.paint_uniform_color([1, 0.706, 0])
    #draw_geometries([mesh])

    R,t = synth.TiltedCams()
    
    Rcam, tcam = synth.TestScene51()

    #Rcam = mmnip.genRotRelLeft(Rcam)

    visu.ViewRefs(Rcam,tcam,showRef=True)

    Rcam1 = FileIO.getJsonFromFile("./tmp/R_finch_28-05-2019_20:14:07.json")['R']

    Rcam1 = np.asarray(Rcam1)

    Rcam1 = np.split(Rcam1,4,axis=0)
    
    RR = []

    for r in Rcam1:
        RR.append(np.squeeze(r))


    Rcam1=RR

    #visu.ViewRefs(Rcam+R,tcam+t)

    #similar to output from ROS (I think)
    camsObs =synth.MultiCamSampleGeneratorFixed(Rcam,tcam,R,t)


    obsR, obsT = obsgen.GenerateCameraPairObs(camsObs,R,t)


    # TRANSLATION STUFF
    A,b = probdefs.translationProbDef(obsT,Rcam1,len(Rcam1))

    #x, res, rank, s = np.linalg.lstsq(A,b,rcond=None) #(A'A)^(-1) * A'b
    x= algos.LeastSquares(A,b)
    
    sol = np.split(x,len(Rcam1))
    visu.ViewRefs(Rcam1,sol,showRef=True)





















if __name__ == '__main__':
    main()