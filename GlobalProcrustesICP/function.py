import matlab.engine
import numpy as np
import scipy.io
mat = scipy.io.loadmat('bunny.mat')
model = mat['modelSimply']

model = model.squeeze()

print(model[2][0].shape[0])

modelpcs = []

for i in range(model.shape[0]):
    modelpcs.append(matlab.double(model[i][0].tolist()))

    


eng = matlab.engine.start_matlab()


#create a list of numpy arrays
#50

eng.globalProcrustesWrapper(modelpcs,5, nargout=0)   #sending input to the function

print('R')
print(R)

print('t')
print(t)
print('s')
print(s)
print('centroid')
print(Centroid)
print('corr')
print(corr)
print('model')
print(model)
