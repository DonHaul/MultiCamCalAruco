
import sys, os
sys.path.append('./libs')


import numpy as np
import scipy.io
from libs import *

mat = scipy.io.loadmat('.\GlobalProcrustesICP\globalIcpOut.mat')


print(mat['R'].shape)

#first dimension is number of cameras, second is number of steps
Hs = [[] for i in range(mat['R'].shape[0])]

for i in range(mat['R'].shape[0]):

    #cuz last step returns no rotation
    for k in range(mat['R'].shape[1]-1):

        Hs[i].append = matmanip.Rt2Homo(mat['R'][i,k],mat['t'][i,k])


actualHs = [np.eye(4) for i in range(mat['R'].shape[0])]

for i in range(mat['R'].shape[0]):

    for k in range(mat['R'].shape[1]-1):

        actualHs[i] = np.dot(Hs[k] , actualHs[i])



#for i in range()
# Rt2Homo(R=None,t=None)