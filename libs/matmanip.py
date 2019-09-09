"""
matmanip.py

This module is used to 
    manipulate matrices
    compare them
    check their properties
    Make rigid transformations
    and generate them
"""
import sys
sys.path.append(".")

import math
import numpy as np
import cv2

def test():
    print("this is a test")

def CompareMatLists(matListA,matListB):
    '''
    Compares 2 matrx lists

    Does:
        matListA - matListB
    '''
    
    #comparing with ground truth
    for i in range(0,len(matListA)):
        print(i)
        print("first")
        print(matListA[i])
        print("second")
        print(matListB[i])
        print("first - seconds")
        print(matListA[i]-matListB[i])

def PrintMatList(matlist):

    for i in range(0,len(matlist)):
        print("Mat "+str(i))
        print(matlist[i])



def genRandRotMatrix(noise):
    '''
    Generates a random matrix having into account the noise

    Args:
        noise: scale of the noise of the matrix to generate
    '''
    

    #generate noise
    a = np.random.rand(3,1)*noise

    #make it have 0 mean
    b =np.ones((3,1))*(noise/2)
    c=a-b


    return genRotMat(np.squeeze(c))

def genRotMat(angle):
    '''Generates a rotation matrix

    Args:
        angles [3x1]list: x,y,z euler angles in degrees of matrix to be generated

    Returns:
        R [3x3] float array :Rotation Matrix
    '''

    #converts to numpy array
    angle = np.asarray(angle)   

    #convert to radians
    angle = angle*math.pi/180

    #x angle rotation matrix
    Rx = [[1,0,0],[0,math.cos(angle[0]),-math.sin(angle[0])],[0,math.sin(angle[0]),math.cos(angle[0])]]
    #y angle rotation matrix
    Ry = [[math.cos(angle[1]),0,math.sin(angle[1])],[0,1,0],[-math.sin(angle[1]),0,math.cos(angle[1])]]
    #z angle rotation matrix
    Rz = [[math.cos(angle[2]),-math.sin(angle[2]),0],[math.sin(angle[2]),math.cos(angle[2]),0],[0,0,1]] 

    #mount the final rotation
    aux = np.dot(Rx,Ry)
    return np.dot(aux,Rz)

def isRotation(rotsols):
    '''
    Prints the properties of the rotation matrix
    '''
    #from world coordinates to ref coordinates

    #generate R between each things
    for j in range(0,len(rotsols)):
        print(np.linalg.det(rotsols[j]))
        print(np.dot(rotsols[j].T,rotsols[j]))
        
def genRotRelLeft(rotsols,ref=0):
    '''
    Multiplies matrix list by one of them on the left side

    Args:
        rotsols: list of matrices
        ref: which of those matrices will be multiplied on the left
    Returns:
        Rrelations: the multiplied matrices
    '''
    #from world coordinates to ref coordinates
    
    Rrelations = []

    #generate R between each things
    for j in range(0,len(rotsols)):
        Rrelations.append(np.dot(rotsols[ref].T,rotsols[j]))

    return Rrelations

def genRotRelRight(rotsols,ref=0):
    '''Rotates given matrics to be rotations relative to one them

    Does:
        R[i] = R w->i  Tranforms it into   R[i]= R ref->i 

    Args:
        rotsols list([3x3]) rotation matrices: list of all the rotation matrices to be rotated/transformed
        ref (int,optional): which of the matrices should they all be relative to? 
    '''
    
    Rrelations = []

    #generate R between each things
    for j in range(0,len(rotsols)):
        Rrelations.append(np.dot(rotsols[j],rotsols[ref].T)) #Rw2*R1w' = R12

    return Rrelations

def globalRotateRotsl(rotsols,ref=0):
    '''Rotates given matrics to be rotations relative to one them

    Does:
        R[i] = R w->i  Tranforms it into   R[i]= R ref->i 

    Args:
        rotsols list([3x3]) rotation matrices: list of all the rotation matrices to be rotated/transformed
        ref (int,optional): which of the matrices should they all be relative to? 
    '''
    
    Rrelations = []

    #generate R between each things
    for j in range(0,len(rotsols)):
        Rrelations.append(np.dot(rotsols[ref].T,rotsols[j])) #Rw2*R1w' = R12

    return Rrelations


def CheckSymmetric(a, tol=1e-8):
    '''
    Verifies if a matrix is symettric

    Args:
        a [matrix]: matrix to check symmetry on
        tol (float,optional): How close does it have to be

    Returns:
        Bool isSymetric
    '''
    return np.allclose(a, a.T, atol=tol)

def Homo2Rt(H):
    '''Creates Homography matrix from R and t

    Args:
        H [4x4] - homography matrix
    Returns:
        R [3x3]- rotation
        t [3x1]- tranlations

    '''
    R = H[0:3,0:3]   #sets R
    t = H[0:3,3]      #sets t

    return R,t

def Rt2Homo(R=None,t=None):
    '''Creates Homography matrix from R and t

    Args:
        R [3x3]- rotation
        t [3]- tranlations
    Returns;
        H [4x4] - homography matrix
    '''
    
    if(R is None):
        R=np.eye(3)
    if(t is None):
        t= np.zeros((3,))

    H = np.eye(4)   #initializes H matrix
    H[0:3,0:3]=R    #sets R
    H[0:3,3]=t      #sets t

    return H

def Transposer(M):
    '''
    Receives matrix list where each matrix will be Transposed
    '''
    transposed = []
    for m in M:
        transposed.append(m.T)
    
    return transposed
    

def InvertT(R,t):
    '''Inverts translation, that is: t(1->2) transforms into t(2->1)
    
    Args:
        R [3x3]: Rotation of it
        t [3x1]: Translation of it
    Returns:
        inverted t
    '''
    return -np.dot(R.T,t)

def Transform(totransform,R,t):
    '''Transforms a t into another referential that is:
    t is in the i referential coordinates, it converts it to be in j referential coordinates    
    Args:
        totransform [3x1]: t to transform
        R[3x3]: R from i to j 
        t[3x1]: t from i to j
    Returns:
        transformedt[3x1]: in j coordinates
    '''

    if(len(totransform.shape)==1):
        totransform=np.expand_dims(totransform,axis=1)

    if(len(t.shape)==1):
        t=np.expand_dims(t,axis=1)

    return np.dot(R,totransform)+t

def InverseTransform(totransform,R,t):
    '''Inverts a certain transformation that is:
    R and t are from the i referential to the j onem it converts  to be from the j referential to the i one    
    Args:
        R[3x3]: R from i to j 
        t[3x1]: t from i to j
    Returns:
        R[3x3]: R from j to i 
        t[3x1]: t from j to i
    '''
    return np.dot(R.T,totransform)-np.dot(R.T,t)

def singlePixe2xyz(depth,coords,K):
    '''
    Gets the 3D position of a pixel in the image
    
    Args:
        depth: depth image
        coords: 2D coordinates to fetch the 3D from [x,y]
        K: intrinsics
    Returns:
        xyz: 3D position of that 2D point
    '''

    fx=K[0,0]
    fy=K[1,1]
    cx=K[0,2]
    cy=K[1,2]

    coords = np.round(coords)

    coords = coords.astype('int') 

    #notice that X and Y must be switched around
    Z=depth[coords[1],coords[0]]/1000.0
    


    X=Z*(coords[0]-cx)/fx
    Y=Z*(coords[1]-cy)/fy
    #print("gayy")
    #print(np.array([X,Y,Z]))
    xyz= np.array([X,Y,Z])

    return xyz

def xyz2rgbd(xyz, rgb, R, T, K_rgb):
    
    Kx=K_rgb[0,0]
    Ky=K_rgb[1,1]
    Cx=K_rgb[0,2]
    Cy=K_rgb[1,2]

    print(xyz.shape)
    print(T)
    T= np.expand_dims(T,0)
    print(T)
    xyz_rgb = (np.dot(R,xyz.T)).T #dont know if order is correct ERROR WRONG
    xyz_rgb = xyz + T
    xyz_rgb=xyz_rgb.T

    x = xyz_rgb[0,:]
    y = xyz_rgb[1,:]
    z = xyz_rgb[2,:]

    u = np.round(Kx * x/z + Cx)
    v = np.round(Ky * y/z + Cy)

    
    u=u.astype(np.int)
    v=v.astype(np.int)
    

    rgb_size = rgb.shape

    n_pixels=rgb.shape[0]*rgb.shape[1]

    #set to 1 elements out of bounds
    v = np.where(v>=rgb_size[0],0,v) #it is set to 1 if mask is true else it is set to what it was befor v[i]
    v = np.where(v<0,0,v)
    
    u=np.where(u>=rgb_size[1],1,u)
    u = np.where(u<0,0,u)

    print(type(u[0]))
    print(type(v[0]))

    print("RGBSHAAPE")
    print(rgb_size[0:2])

    print("invalids?")
    print(u[u > rgb_size[1]])
    print(v[v > rgb_size[0]])
    print(u[u <0])
    print(v[v <0])
    
    #for y1,y2 in zip(u,v):
    #    print(y1,y2)

    print(u.shape)
    print(v.shape)

    
    #gets index of where point is
    rgb_inds = np.ravel_multi_index((v,u),np.array(rgb_size[0:2],dtype=np.int)) #might be WRONG ORDER ERROR

    print(rgb_inds.shape)
    print(rgb_inds)
    rgbd = np.zeros((n_pixels,3))
    rgb_aux = rgb.reshape(480*640,3)

    print(rgb_aux.shape)

    for id in rgb_inds:
        rgbd[id,:]=rgb_aux[id]
    #colors corresponding to those pixels only on the ids that exist on the depth image
    #rgbd = [0:n_pixels,:]= rgb_aux[rgb_inds]
    #rgbd((1:n_pixels).T,:) = rgb_aux(rgb_inds,:);
    
    
    #rgbd = np.where(xyz[:,2]==0,0,rgbd)
    #rgbd(xyz(:,1) == 0 & xyz(:,2) == 0 & xyz(:,3) == 0,:) = 0;

    rgbd=rgbd.reshape(rgb_size).astype(np.uint8)

    return rgbd


def coords2img(K,Rs,Ts):

    if len(Rs)!=len(Ts):
         raise Exception('Rs and Ts must have the same lenght')

    coords=[]

    for R,T in zip(Rs,Ts):
        coords=K.dot(np.concatenate((R,t),axis=1))

    


def depthimg2xyz2(depthimg,K,size=(480,640)):
    '''
    Convert full depth image
    '''

    fx=K[0,0]
    fy=K[1,1]
    cx=K[0,2]
    cy=K[1,2]
    
    depthcoords = np.zeros((size[0], size[1],3)) #height by width  by 3(X,Y,Z)

    u,v =np.indices((size[0], size[1]))
    u=u-cy
    v=v-cx

    depthcoords[:,:,2]= depthimg/1000.0
    depthcoords[:,:,0]= depthcoords[:,:,2]*v/fx
    depthcoords[:,:,1]= depthcoords[:,:,2]*u/fy
    


    return depthcoords

def Transl_fromWtoRef(R,T,ref=0):
    '''
    Converts translation reference to one of a ref one

    Args:
        R: list of rotations
        T: list of translations
        ref: which referential shall be the new reference
    Returns:
        newT: translations in the new reference
    '''
    print("NEEDS UNIT TESTING FOR OTHER REFS")

    #from_to
    #tw_1 is  the invert of t1_w
    newT=[]
    # is is making ti_1 Rw_1*ti_w + tw_1
    for t in T:

        auxT = Transform(t,R[ref].T,InvertT(R[ref],T[ref]))
        newT.append(auxT)

    #print("doing shit")
    return newT


def depthimg2xyz(depthimg,rgb,K):

    fx=K[0,0]
    fy=K[1,1]
    cx=K[0,2]
    cy=K[1,2]
    
    depthcoords = np.zeros((480, 640,3)) #height by width  by 3(X,Y,Z)

    #u,v =np.indices((480,640))
    points=[]
    colors = []

    for v in range(depthimg.shape[1]):
        for u in range(depthimg.shape[0]):
            color = rgb[u,v,:] 
            Z = depthimg[u,v] / 1000.0
            if Z==0: continue

            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            points.append([X,Y,Z])
            colors.append(color)



    return np.asarray(points),np.asarray(colors) 