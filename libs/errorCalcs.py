import numpy as np

def Hello():
    print("Hello")

def MatrixListError(A,B):
    '''
    Calculates errors
    '''
    C=[]

    for a, b in zip(A,B):
        C.append(a-b)

    actualNorm=[]
    for r in C:
        print(r.shape)
        actualNorm.append(np.linalg.norm(r,ord='fro'))

    inds = np.squeeze(np.indices((len(actualNorm),)))

    return actualNorm,inds
