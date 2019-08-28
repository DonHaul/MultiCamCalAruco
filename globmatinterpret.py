
import sys, os
sys.path.append('./libs')


import numpy as np
import scipy.io
from libs import *

import matlab.engine
import numpy as np
import scipy.io

def CalculateGlobICP():
        eng = matlab.engine.start_matlab()


        #create a list of numpy arrays
        #50

        eng.globalProcrustesWrapper(modelpcs,5, nargout=0)   #sending input to the function
        eng.cd("./GlobalProcrustesICP")

        return RetrieveGlobICPOutput()

def RetrieveGlobICPOutput(outputpath='./GlobalProcrustesICP/globalIcpOut.mat'):

        mat = scipy.io.loadmat(outputpath)


        print(mat['R'].shape)

        #first dimension is number of cameras, second is number of steps
        Hs = [[] for i in range(mat['R'].shape[0])]

        for i in range(mat['R'].shape[0]):

        #cuz last step returns no rotation
        for k in range(mat['R'].shape[1]-1):
        
                Hs[i].append(matmanip.Rt2Homo(mat['R'][i,k],np.squeeze(mat['t'][i,k])))
                

        actualHs = [np.eye(4) for i in range(mat['R'].shape[0])]

        print(len(actualHs),actualHs[0].shape)

        for i in range(mat['R'].shape[0]):

        for k in range(mat['R'].shape[1]-1):



                actualHs[i] = np.dot(Hs[i][k] , actualHs[i])


        print(actualHs[0].shape)

        registeredModel = []

        #registeredmodel[0][x][0] cointains the array of points of pointcloud x
        for i in range(len(actualHs)):
                registeredModel.append(print(mat['registeredModel'][0][i][0]))



        #for i in range()
        # Rt2Homo(R=None,t=None)


        return actualHs,registeredModel