from libs import *
import numpy as np
import pprint
from scipy.linalg import orthogonal_procrustes
from scipy.spatial import procrustes



arucoCorners = FileIO.getFromPickle("pickles/corners_scorpion_07-06-2019_15:34:54.pickle")

print(arucoCorners)



for i in range(4):
        print("wow")
        indexStart = 4*3*i+0
        indexEnd =  indexStart+10
        print(indexStart)
        print(indexEnd)
        
        cornerstart = arucoCorners[indexStart]
        cornerend = arucoCorners[indexEnd]



        norm = np.linalg.norm(cornerstart-cornerend)

        print(norm)